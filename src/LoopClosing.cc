/**
 * @file LoopClosing.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 回环检测线程
 * @version 0.1
 * @date 2019-05-05
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM2
{

// 构造函数
LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0)
{
    // 连续性阈值
    mnCovisibilityConsistencyTh = 3;
}

// 设置追踪线程句柄
void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}
// 设置局部建图线程的句柄
void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

// 线程主函数
void LoopClosing::Run()
{
    mbFinished =false;

    // 线程主循环
    while(1)
    {
        // Check if there are keyframes in the queue
        // Loopclosing中的关键帧是LocalMapping发送过来的，LocalMapping是Tracking中发过来的
        // 在LocalMapping中通过 InsertKeyFrame 将关键帧插入闭环检测队列mlpLoopKeyFrameQueue
        // 闭环检测队列mlpLoopKeyFrameQueue中的关键帧不为空
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            if(DetectLoop())
            {
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
               if(ComputeSim3())
               {
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }
            }
        }

        // 查看是否有外部线程请求复位当前线程
        ResetIfRequested();

        // 查看外部线程是否有终止当前线程的请求,如果有的话就跳出这个线程的主函数的主循环
        if(CheckFinish())
            break;

        //usleep(5000);
		std::this_thread::sleep_for(std::chrono::milliseconds(5));

	}

    // 运行到这里说明有外部线程请求终止当前线程,在这个函数中执行终止当前线程的一些操作
    SetFinish();
}

// 将某个关键帧加入到回环检测的过程中,由局部建图线程调用
void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    // NOTICE 这里第0个关键帧不能够参与到回环检测的过程中,因为第0关键帧定义了整个地图的世界坐标系
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

/*
 * 查看列表中是否有等待被插入的关键帧
 * @return 如果存在，返回true
 */
bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

// 检测回环
bool LoopClosing::DetectLoop()
{
    {
        // 从队列中取出一个关键帧,作为当前关键帧
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    // STEP 1：如果距离上次闭环没多久（小于10帧），或者map中关键帧总共还没有10帧，则不进行闭环检测
    // 后者的体现是当mLastLoopKFid为0的时候
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    // VIII-A
    // STEP 2：遍历所有共视关键帧，计算当前关键帧与每个共视关键的bow相似度得分，并得到最低得分minScore
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;
        // 计算当前遍历到的这个关键帧，和前面的这个当前关键帧计算相似度得分；得分越低,相似度越低
        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);
        // 更新最低得分
        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    // STEP 3：在所有关键帧中找出闭环备选帧
    // 说白了就是认为和当前关键帧具有回环关系的关键帧,不应该低于当前关键帧的相邻关键帧的最低的相似度
    // 得到的这些关键帧,和当前关键帧具有较多的共视单词,并且相似度评分都挺高
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;           // 没有检测到回环
    }

    // 接下来就属于检测到回环啦

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    // STEP 4：在候选帧中检测具有连续性的候选帧
    // 1、每个候选帧将与自己相连的关键帧构成一个“子候选组spCandidateGroup”， vpCandidateKFs-->spCandidateGroup
    // 2、检测“子候选组”中每一个关键帧是否存在于“连续组”，如果存在 nCurrentConsistency++，则将该“子候选组”放入“当前连续组vCurrentConsistentGroups”
    // 3、如果nCurrentConsistency大于等于3，那么该”子候选组“代表的候选帧过关，进入mvpEnoughConsistentCandidates
    
    // 相关的概念说明:
    // 组(group): 对于某个关键帧, 其和其具有共视关系的关键帧组成了一个"组";
    // 子候选组(CandidateGroup): 对于某个候选的回环关键帧, 其和其具有共视关系的关键帧组成的一个"组";
    // 连续(Consistent):  不同的组之间如果共同拥有一个及以上的关键帧,那么称这两个组之间具有连续关系
    // 连续性(Consistency):称之为连续长度可能更合适,表示累计的连续的链的长度:A--B 为1, A--B--C--D 为3等;具体反映在数据类型 ConsistentGroup.second上
    // 连续组(Consistent group): mvConsistentGroups存储了上次执行回环检测时, 新的被检测出来的具有连续性的多个组的集合.由于组之间的连续关系是个网状结构,因此可能存在
    //                          一个组因为和不同的连续组链都具有连续关系,而被添加两次的情况(当然连续性度量是不相同的)
    // 连续组链:我guoqing自造的称呼,类似于菊花链A--B--C--D这样形成了一条连续组链.对于这个例子中,由于可能E,F都和D有连续关系,因此连续组链会产生分叉;为了简化计算,连续组中将只会保存
    //         最后形成连续关系的连续组们(见下面的连续组的更新)
    // 子连续组: 上面的连续组中的一个组
    // 连续组的初始值: 在遍历某个候选帧的过程中,如果该子候选组没有能够和任何一个上次的子连续组产生连续关系,那么就将添加自己组为连续组,并且连续性为0(相当于新开了一个连续链)
    // 连续组的更新: 当前次回环检测过程中,所有被检测到和之前的连续组链有连续的关系的组,都将在对应的连续组链后面+1,这些子候选组(可能有重复,见上)都将会成为新的连续组;
    //              换而言之连续组mvConsistentGroups中只保存连续组链中末尾的组

    mvpEnoughConsistentCandidates.clear();// 最终筛选后得到的闭环帧

    // ConsistentGroup数据类型为pair<set<KeyFrame*>,int>
    // ConsistentGroup.first对应每个“连续组”中的关键帧，ConsistentGroup.second为每个“连续组”的序号
    // 这个下标是每个"子连续组"的下标,bool表示当前的候选组中是否有和该组相同的一个关键帧
    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    // 遍历刚才得到的每一个候选关键帧
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

        // 将自己以及与自己相连的关键帧构成一个“子候选组”
        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        // 遍历之前的“子连续组s”
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            // 取出一个之前的子连续组
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            // 遍历每个“子候选组”，检测候选组中每一个关键帧在“子连续组”中是否存在
            // 如果有一帧共同存在于“子候选组”与之前的“子连续组”，那么“子候选组”与该“子连续组”连续
            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;// 该“子候选组”与该“子连续组”相连
                    bConsistentForSomeGroup=true;// 该“子候选组”至少与一个”子连续组“相连
                    break;
                }
            }

            if(bConsistent)
            {
                // 和当前的候选组发生"连续"关系的子连续组的"已连续id"
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                // 如果当前遍历到的"子连续组"还没有和"子候选组有相同的关键帧的记录,那么就+
                if(!vbConsistentGroup[iG])// 这里作者原本意思是不是应该是vbConsistentGroup[i]而不是vbConsistentGroup[iG]呢？（wubo???）
                                          //! 应该就是iG,vbConsistentGroup声明时候的大小和mvConsistentGroups是相同的,而在遍历mvConsistentGroups中的每个元素的时候使用的下标是iG
                                          //! 而i是在遍历vpCandidateKFs也就是经过KeyFrameDatabase得到的一堆初始的闭环候选帧的时候才使用的  --- guoqing
                {
                    // 将该“子候选组”的该关键帧打上连续编号加入到“当前连续组”
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                // 如果已经连续得足够多了,那么当前的这个候选关键帧是足够靠谱的
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    // 在遍历每个候选关键帧的过程中的、遍历之前的“子连续组”过程中,如果遍历到某个"子连续组"中符合要求的关键帧,就把当前的候选关键帧也添加进去
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }

                //这里是不是缺一个break来提高效率呢？(wubo???)
                // 师兄说的对 (guoqing)
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        // 如果该“子候选组”的所有关键帧都不存在于“子连续组”，那么 vCurrentConsistentGroups 将为空，
        // 于是就把“子候选组”全部拷贝到 vCurrentConsistentGroups， 并最终用于更新mvConsistentGroups，计数器设为0，重新开始
        if(!bConsistentForSomeGroup)
        {
            // 这个地方是不是最好clear一下 vCurrentConsistentGroups 呢？(wubo???)
            //! 师兄不可以啊! 因为上面的if条件满足的时候,只能够说明当前遍历到的这个候选关键帧形成的子候选组和所有的子连续组没有连续关系,所以当前子候选组是一个新出现的连续组链;
            //! 但是这不能够代表其他的候选关键帧的子候选组和那些子连续组们没有连续关系!所以 vCurrentConsistentGroups 不能清空
            //! 如果清空了的话,那么只要候选关键帧中有一帧比较孤立的,和当前已有的所有连续组都不产生共视关系,那么完蛋了,在遍历当前候选关键帧之前的所有连续关系
            //! 都会被清空    ----guoqing

            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }// 遍历得到的初级的候选关键帧

    // Update Covisibility Consistent Groups
    // 当前的变上一次的
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    mpKeyFrameDB->add(mpCurrentKF);

    // 如果不过关
    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else // 过关
    {
        return true;
    }

    // 多余的代码,执行不到
    mpCurrentKF->SetErase();
    return false;
}


bool LoopClosing::ComputeSim3()
{
    // 说明：计算当前帧与闭环帧的Sim3变换等
    // 1. 通过Bow加速描述子的匹配，利用RANSAC粗略地计算出当前帧与闭环帧的Sim3（当前帧---闭环帧）          \n
    // 2. 根据估计的Sim3，对3D点进行投影找到更多匹配，通过优化的方法计算更精确的Sim3（当前帧---闭环帧）     \n
    // 3. 将闭环帧以及闭环帧相连的关键帧的MapPoints与当前帧的点进行匹配（当前帧---闭环帧+相连关键帧）      \n
    // 注意以上匹配的结果均都存在成员变量mvpCurrentMatchedPoints中，实际的更新步骤见CorrectLoop()步骤3：Start Loop Fusion \n
    // 对于双目或者是RGBD输入的情况,计算得到的尺度=1

    // Input:
    // do   : 
    //       1. 遍历闭环候选帧集，筛选出与当前帧的匹配特征点数大于20的候选帧集合，并为每一个候选帧构造一个Sim3Solver
    //       2. 对每一个候选帧进行 RANSAC 迭代匹配，直到有一个候选帧匹配成功，或者全部失败
    //       3. 取出闭环匹配上关键帧的相连关键帧，得到它们的MapPoints放入 mvpLoopMapPoints
    //       4. 将闭环匹配上关键帧以及相连关键帧的MapPoints投影到当前关键帧进行投影匹配
    //       5. 判断当前帧与检测出的所有闭环关键帧是否有足够多的MapPoints匹配
    //       6. 清空mvpEnoughConsistentCandidates
    // output:

    // Step 0. 准备工作
    // For each consistent loop candidate we try to compute a Sim3
    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    // 用 vector 存储每一个候选帧的Sim3Solver求解器
    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    // 用 vector 存储每个候选帧的匹配地图点信息
    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    // 用 vector 存储每个候选帧应该被放弃(True）或者 保留(False)
    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    // 完成 Step 1.0 的匹配后，被保留的候选帧数量
    int nCandidates=0;

    // Step 1. 遍历闭环候选帧集，筛选出与当前帧的匹配特征点数大于20的候选帧集合，并为每一个候选帧构造一个Sim3Solver
    for(int i=0; i<nInitialCandidates; i++)
    {
        // Step 1.1 从筛选的闭环候选帧中取出一帧关键帧pKF
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // 避免在LocalMapping中KeyFrameCulling函数将此关键帧作为冗余帧剔除
        pKF->SetNotErase();

        // 如果候选帧质量不高，直接PASS
        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        // Step 1.2 将当前帧 mpCurrentKF 与闭环候选关键帧pKF匹配
        // 通过bow加速得到 mpCurrentKF 与 pKF 之间的匹配特征点
        // vvpMapPointMatches 是匹配特征点对应的 MapPoints,本质上来自于候选闭环帧
        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        // 匹配的特征点数太少，该候选帧剔除
        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            // Step 1.3 为保留的候选帧构造Sim3求解器
            // 如果 mbFixScale 为 true，则是6DoFf优化（双目 RGBD）
            // 如果是false，则是7DoF优化（单目）
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);

            // Sim3Solver Ransac 过程至少20个inliers 300次迭代
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver;
        }

        // 保留的候选帧数量
        nCandidates++;
    }

    // 用于标记是否有一个候选帧通过Sim3Solver的求解与优化
    bool bMatch = false;

    // Step 2.0 对每一个候选帧进行 RANSAC 迭代匹配，直到有一个候选帧匹配成功，或者全部失败
    while(nCandidates>0 && !bMatch)
    {
        // 遍历每一个候选帧
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // 内点（Inliers）标志
            // 即标记经过RANSAC sim3 求解后,vvpMapPointMatches中的哪些作为内点
            vector<bool> vbInliers; 
        
            // 内点（Inliers）数量
            int nInliers;

            // 是否到达了最优解
            bool bNoMore;

            // Step 2.1 取出从 Step 1.3 中为当前候选帧构建的 Sim3Solver 并开始迭代
            Sim3Solver* pSolver = vpSim3Solvers[i];

            // 最多迭代5次，返回的Scm是候选帧pKF到当前帧mpCurrentKF的Sim3变换（T12）
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            // 经过n次循环，每次迭代5次，总共迭代 n*5 次
            // 总迭代次数达到最大限制还没有求出合格的Sim3变换，该候选帧剔除
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    // 保存inlier的MapPoint
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                // Step 2.2 通过步骤2.1求取的Sim3变换引导关键帧匹配弥补步骤1中的漏匹配
                // 候选帧pKF到当前帧mpCurrentKF的R（R12），t（t12），变换尺度s（s12）
                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();

                // 查找更多的匹配（成功的闭环匹配需要满足足够多的匹配特征点数，之前使用SearchByBoW进行特征点匹配时会有漏匹配）
                // 通过Sim3变换，确定pKF1的特征点在pKF2中的大致区域，同理，确定pKF2的特征点在pKF1中的大致区域
                // 在该区域内通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点，更新匹配vpMapPointMatches
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                // Step 2.3 Sim3优化，只要有一个候选帧通过Sim3的求解与优化，就跳出停止对其它候选帧的判断
                // OpenCV的Mat矩阵转成Eigen的Matrix类型
                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
            
                // 如果mbFixScale为true，则是6DoFf优化（双目 RGBD），如果是false，则是7DoF优化（单目）
                // 优化mpCurrentKF与pKF对应的MapPoints间的Sim3，得到优化后的量gScm
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                // 如果优化成功，则停止 ransacs（即while循环）并继续下一步
                if(nInliers>=20)
                {
                    // 为True时将不再进入 while循环
                    bMatch = true;
                    // mpMatchedKF就是最终闭环检测出来与当前帧形成闭环的关键帧
                    mpMatchedKF = pKF;

                    // 得到从世界坐标系到该候选帧的Sim3变换，Scale=1
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);

                    // 得到g2o优化后从世界坐标系到当前帧的Sim3变换
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);
                    mvpCurrentMatchedPoints = vpMapPointMatches;

                    // 只要有一个候选帧通过Sim3的求解与优化，就跳出停止对其它候选帧的判断
                    break;
                }
            }
        }
    }

    // 退出上面while循环的原因有两种,一种是求解到了bMatch置位后出的,另外一种是nCandidates耗尽为0
    // 没有一个闭环匹配候选帧通过Sim3的求解与优化
    if(!bMatch)
    {
        // 清空mvpEnoughConsistentCandidates
        for(int i=0; i<nInitialCandidates; i++)
            // 卧槽,这么严格吗?这些关键帧以后都不会在再参加回环检测过程了
            mvpEnoughConsistentCandidates[i]->SetErase();
        // 当前关键帧也是,将不会再参加回环检测的过程了
        mpCurrentKF->SetErase();
        return false;
    }

    // Step 3.0：取出闭环匹配上关键帧的相连关键帧，得到它们的MapPoints放入 mvpLoopMapPoints
    // 注意是匹配上的那个关键帧：mpMatchedKF
    // 将mpMatchedKF相连的关键帧全部取出来放入 vpLoopConnectedKFs
    // 将vpLoopConnectedKFs的MapPoints取出来放入mvpLoopMapPoints
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();

    // 包含闭环匹配关键帧本身,形成一个组
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();

    // 遍历这个组中的每一个关键帧
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        // 遍历其中一个关键帧的所有地图点
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);

                    // 标记该MapPoint被mpCurrentKF闭环时观测到并添加，避免重复添加
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    // Step 4.0：将闭环匹配上关键帧以及相连关键帧的MapPoints投影到当前关键帧进行投影匹配
    // 根据投影查找更多的匹配（成功的闭环匹配需要满足足够多的匹配特征点数）
    // 根据Sim3变换，将每个mvpLoopMapPoints投影到mpCurrentKF上，并根据尺度确定一个搜索区域，
    // 根据该MapPoint的描述子与该区域内的特征点进行匹配，如果匹配误差小于TH_LOW即匹配成功，更新mvpCurrentMatchedPoints
    // mvpCurrentMatchedPoints将用于SearchAndFuse中检测当前帧MapPoints与匹配的MapPoints是否存在冲突
    // 搜索范围系数为10
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    // Step 5.0 判断当前帧与检测出的所有闭环关键帧是否有足够多的MapPoints匹配
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    // Step 6.0 清空mvpEnoughConsistentCandidates
    if(nTotalMatches>=40)
    {
        // 如果当前回环可靠,那么就取消 候选的优质连续关键帧中的所有帧 参与回环检测的资格,除了当前的回环关键帧
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else    // 同,只不过这个时候回环帧不靠谱.也清除
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }
}

/* 
 * @brief 闭环纠正
 * @detials \n
 * 1. 通过求解的Sim3以及相对姿态关系，调整与当前帧相连的关键帧位姿以及这些关键帧观测到的MapPoints的位置（相连关键帧---当前帧） \n
 * 2. 将闭环帧以及闭环帧相连的关键帧的MapPoints和与当前帧相连的关键帧的点进行匹配（相连关键帧+当前帧---闭环帧+相连关键帧）     \n
 * 3. 通过MapPoints的匹配关系更新这些帧之间的连接关系，即更新covisibility graph                                      \n
 * 4. 对Essential Graph（Pose Graph）进行优化，MapPoints的位置则根据优化后的位姿做相对应的调整                         \n
 * 5. 创建线程进行全局Bundle Adjustment
 */
void LoopClosing::CorrectLoop()
{

    cout << "Loop detected!" << endl;
    // STEP 0：请求局部地图停止，防止在回环矫正时局部地图线程中InsertKeyFrame函数插入新的关键帧
    // STEP 1：根据共视关系更新当前帧与其它关键帧之间的连接
    // STEP 2：通过位姿传播，得到Sim3优化后，与当前帧相连的关键帧的位姿，以及它们的MapPoints
    // STEP 3：检查当前帧的MapPoints与闭环匹配帧的MapPoints是否存在冲突，对冲突的MapPoints进行替换或填补
    // STEP 4：通过将闭环时相连关键帧的mvpLoopMapPoints投影到这些关键帧中，进行MapPoints检查与替换
    // STEP 5：更新当前关键帧之间的共视相连关系，得到因闭环时MapPoints融合而新得到的连接关系
    // STEP 6：进行EssentialGraph优化，LoopConnections是形成闭环后新生成的连接关系，不包括步骤7中当前帧与闭环匹配帧之间的连接关系
    // STEP 7：添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
    // STEP 8：新建一个线程用于全局BA优化

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    // STEP 0：请求局部地图停止，防止在回环矫正时局部地图线程中InsertKeyFrame函数插入新的关键帧
    mpLocalMapper->RequestStop();

    // 终止全局BA线程
    if(isRunningGBA())
    {
        // 这个标志位仅用于控制输出提示，可忽略
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            // 停止全局BA线程
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        //usleep(1000);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Ensure current keyframe is updated
    // STEP 1：根据共视关系更新当前帧与其它关键帧之间的连接
    // 猜测这里还要更新的原因应该是,当前处理的这个关键帧应该不是当前时刻最新插入的关键帧,它可能和后面新来的一些关键帧也产生了共视关系,虽然
    // 在那些关键帧中也在这个中添加了更新的操作但是,有些图(本征图?)没法被他们更新吧,还是得这个关键帧自己来
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    // STEP 2：通过位姿传播，得到Sim3优化后，与当前帧相连的关键帧的位姿，以及它们的MapPoints
    // 当前帧与世界坐标系之间的Sim变换在ComputeSim3函数中已经确定并优化，
    // 通过相对位姿关系，可以确定这些相连的关键帧与世界坐标系之间的Sim3变换

    // 取出与当前帧相连的关键帧，包括当前关键帧 -- 获取当前关键帧组
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    // 先将mpCurrentKF的Sim3变换存入，固定不动
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();

    // 对地图点操作的临界区
    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        // STEP 2.1：通过位姿传播，得到Sim3调整后其它与当前帧相连关键帧的位姿（只是得到，还没有修正）
        // 遍历"当前关键帧组""
        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            // currentKF在前面已经添加
            if(pKFi!=mpCurrentKF)
            {
                // 得到当前帧到pKFi帧的相对变换
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                // 当前帧的位姿固定不动，其它的关键帧根据相对关系得到Sim3调整的位姿
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                // Pose corrected with the Sim3 of the loop closure
                // 得到闭环g2o优化后各个关键帧的位姿
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            // Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        // STEP 2.2：得到调整相连帧位姿后，修正这些关键帧的MapPoints
        // 遍历当前关键帧组中的每一个关键帧
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            // 遍历这个关键帧中的每一个地图点
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId) // 防止重复修正
                    continue;

                // 修正过程开始,本质上也是基于当前关键帧的优化后的位姿展开的
                // Project with non-corrected pose and project back with corrected pose
                // 将该未校正的eigP3Dw先从世界坐标系映射到未校正的pKFi相机坐标系，然后再反映射到校正后的世界坐标系下
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                // 勿忘更新
                pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            // STEP 2.3：将Sim3转换为SE3，根据更新的Sim3，更新关键帧的位姿
            // 其实是现在已经有了更新后的关键帧组中关键帧的位姿,但是在上面的操作时只是暂时存储到了 KeyFrameAndPose 类型的变量中,还没有写回到关键帧对象中
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();  //调用toRotationMatrix来去除尺度信息的
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();                   //目测是其中还包含有尺度信息,还得去除
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Make sure connections are updated
            // STEP 2.4：根据共视关系更新当前帧与其它关键帧之间的连接
            // 地图点的位置改变了,可能会引起共视关系\权值的改变 
            pKFi->UpdateConnections();
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        // STEP 3：检查当前帧的MapPoints与闭环匹配帧的MapPoints是否存在冲突，对冲突的MapPoints进行替换或填补
        // mvpCurrentMatchedPoints 是刚才当前关键帧和闭环关键帧组的所有地图点进行投影得到的匹配点
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                //下标越界问题应该不存在的吧,存储地图点的数组应该是和特征点的数目长度是相同的,即使是mvpCurrentMatchedPoints中也不会有超过特征点个数的地图点啊
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i); 
                if(pCurMP)
                    // 如果有重复的MapPoint（当前帧和匹配帧各有一个），则用匹配帧的代替现有的
                    // 因为现有的帧往往已经有累计的误差了
                    pCurMP->Replace(pLoopMP);
                else// 如果当前帧没有该MapPoint，则直接添加
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        } 

    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    // STEP 4：通过将闭环时相连关键帧的mvpLoopMapPoints投影到这些关键帧中，进行MapPoints检查与替换
    // 当前线程中的函数
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    // STEP 5：更新当前关键帧之间的共视相连关系，得到因闭环时MapPoints融合而新得到的连接关系
    // 这个变量中将会存储那些因为闭环关系的形成,而新形成的链接关系
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

    // STEP 5.1：遍历当前帧相连关键帧（一级相连）
    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        // STEP 5.2：得到与当前帧相连关键帧的相连关键帧（二级相连）
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        // STEP 5.3：更新一级相连关键帧的连接关系(会把当前关键帧添加进去,因为地图点已经更新和替换了)
        pKFi->UpdateConnections();
        // STEP 5.4：取出该帧更新后的连接关系
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        // STEP 5.5：从连接关系中去除闭环之前的二级连接关系，剩下的连接就是由闭环得到的连接关系
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        // STEP 5.6：从连接关系中去除闭环之前的一级连接关系，剩下的连接就是由闭环得到的连接关系
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    // STEP 6：进行EssentialGraph优化，LoopConnections是形成闭环后新生成的连接关系，不包括步骤7中当前帧与闭环匹配帧之间的连接关系
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    // Add loop edge
    // STEP 7：添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
    // 这两句话应该放在OptimizeEssentialGraph之前，因为OptimizeEssentialGraph的步骤4.2中有优化
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    // STEP 8：新建一个线程用于全局BA优化
    // OptimizeEssentialGraph只是优化了一些主要关键帧的位姿，这里进行全局BA可以全局优化所有位姿和MapPoints
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    cout << "Loop Closed!" << endl;

    mLastLoopKFid = mpCurrentKF->mnId;
}

// 通过将闭环时相连关键帧的MapPoints投影到当前关键帧组中的这些关键帧中
// 进行MapPoints检查与替换(将回环帧中的地图点代替当前关键帧组中关键帧中的地图点)
// 因为回环关键帧处的时间比较久远,而当前关键帧组中的关键帧的地图点会有累计的误差啊
void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{

    // 定义ORB匹配器
    ORBmatcher matcher(0.8);

    // 遍历闭环相连的关键帧 
    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        // 将闭环相连帧的MapPoints坐标变换到pKF帧坐标系，然后投影，检查冲突并融合
        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        // vpReplacePoints中存储的将会是这个关键帧中的地图点（也就是需要替换掉的新的地图点）,原地图点的id则对应这个变量的下标
        // 搜索区域系数为4
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        // 之所以不在前面的 Fuse 函数中进行地图点融合更新的原因是需要对地图加锁,而这里的设计中matcher中并不保存地图的指针
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        // 遍历闭环帧组的所有的地图点
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                // 用mvpLoopMapPoints替换掉之前的
                pRep->Replace(mvpLoopMapPoints[i]);
            }
        }
    }
}

// 由外部线程调用,请求复位当前线程
void LoopClosing::RequestReset()
{
    // 标志置位
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    // 堵塞,直到回环检测线程复位完成
    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
		//usleep(5000);
		std::this_thread::sleep_for(std::chrono::milliseconds(5));

    }
}

// 当前线程调用,检查是否有外部线程请求复位当前线程,如果有的话就复位回环检测线程
void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    // 如果有来自于外部的线程的复位请求,那么就复位当前线程
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();   // 清空参与和进行回环检测的关键帧队列
        mLastLoopKFid=0;                // 上一次没有和任何关键帧形成闭环关系
        mbResetRequested=false;         // 复位请求标志复位
    }
}
// 全局BA线程,这个是这个线程的主函数; 输入的函数参数看上去是闭环关键帧,但是在调用的时候给的其实是当前关键帧的id
void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;

    // 记录当前迭代id,用来检查全局BA过程是否是因为意外结束的
    int idx =  mnFullBAIdx;
    // 牛逼,直接调优化器中的函数了
    // 10是迭代次数
    // mbStopGBA直接传引用过去了,这样当有外部请求的时候这个优化函数能够及时相应并且结束掉
    //? 提问:进行完这个过程后我们能够获得哪些信息?
    // 目测是能够得到全部关键帧优化后的位姿,以及部分地图点优化之后的位姿
    Optimizer::GlobalBundleAdjustemnt(mpMap,        // 地图点对象
                                      10,           // 迭代次数
                                      &mbStopGBA,   // 外界控制 GBA 停止的标志
                                      nLoopKF,      // 形成了闭环的当前关键帧的id
                                      false);       // 不使用鲁棒核函数

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        // 如果全局BA过程是因为意外结束的,那么后面的内容就都不用管了
        if(idx!=mnFullBAIdx)
            return;

        // 如果没有中断当前次BA的请求
        // 这里和上面那句话的功能还有些不同,因为如果一次全局优化被中断,往往意味又要重新开启一个新的全局BA;为了中断当前正在执行的优化过程mbStopGBA将会被置位,同时会有一定的时间
        // 使得该线程进行响应;而在开启一个新的全局优化进程之前 mbStopGBA 将会被置为False
        // 因此,如果被强行中断的线程退出时已经有新的线程启动了,mbStopGBA=false,为了避免进行后面的程序,所以有了上面的程序;
        // 而如果被强行终端的线程退出时新的线程还没有启动,那么上面的条件就不起作用了(虽然概率很小,前面的程序中mbStopGBA置位后很快mnFullBAIdx就++了,保险起见),所以这里要再判断一次
        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
				//usleep(1000);
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            // 看上去是一个向量，但是要知道这个变量中其实只保存了第一个关键帧
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

            // 遍历全局地图中的所有关键帧
            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
                // 遍历当前关键帧的子关键帧
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    // 避免重复设置
                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        // (对于坐标系中的点的话)从父关键帧到当前子关键帧的位姿变换
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        // （对于坐标系中的点）再利用优化后的父关键帧的位姿，转换到世界坐标系下 --  //? 算是得到了这个子关键帧的优化后的位姿啦？
                        // 这种最小生成树中除了根节点，其他的节点都会作为其他关键帧的子节点，这样做可以使得最终所有的关键帧都得到了优化
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }
                // 更新当前关键帧的位姿
                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                // 从列表中移除
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

            // 遍历每一个地图点
            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                // NOTICE 并不是所有的地图点都会直接参与到全局BA优化中,但是大部分的地图点需要根据全局BA优化后的结果来重新纠正自己的位姿
                // 如果这个地图点直接参与到了全局BA优化的过程,那么就直接重新设置器位姿即可
                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else // 如故这个地图点并没有直接参与到全局BA优化的过程中,那么就使用器参考关键帧的新位姿来优化自己的位姿
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    // 说明这个关键帧，在前面的过程中也没有因为“当前关键帧”得到全局BA优化 
                    //? 可是,为什么会出现这种情况呢? 难道是因为这个地图点的参考关键帧设置成为了bad?
                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    // 转换到其参考关键帧相机坐标系下的坐标
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    // 然后使用已经纠正过的参考关键帧的位姿,再将该地图点变换到世界坐标系下
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }

            // 释放,使得LocalMapping线程重新开始工作
            mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    } // 更新(几乎)所有的关键帧和地图点
}

// 由外部线程调用,请求终止当前线程
void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

// 当前线程调用,查看是否有外部线程请求当前线程
bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

// 有当前线程调用,执行完成该函数之后线程主函数退出,线程销毁
void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

// 由外部线程调用,判断当前回环检测线程是否已经正确终止了
bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
