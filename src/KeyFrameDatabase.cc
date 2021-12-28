/**
 * @file KeyFrameDatabase.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 关键帧数据库,用于回环检测和重定位
 * @version 0.1
 * @date 2019-04-25
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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

// 构造函数
KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc)
{
    // 数据库的主要内容了
    mvInvertedFile.resize(voc.size()); // number of words
}

/**
 * @brief 数据库有新的关键帧，根据关键帧的词袋向量，更新数据库的倒排索引
 * 
 * @param[in] pKF   新添加到数据库的关键帧
 */
void KeyFrameDatabase::add(KeyFrame *pKF)
{
    // 线程锁
    unique_lock<mutex> lock(mMutex);

    // 将该关键帧词袋向量里每一个单词更新倒排索引
    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

/**
 * @brief 关键帧被删除后，更新数据库的倒排索引
 * 
 * @param[in] pKF   删除的关键帧
 */
void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    // 线程锁，保护共享数据
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    // 每一个KeyFrame包含多个words，遍历mvInvertedFile中的这些words，然后在word中删除该KeyFrame
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        // 取出包含该单词的所有关键帧列表
        list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];
        // 如果包含待删除的关键帧，则把该关键帧从列表里删除
        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

// 清空关键帧数据库
void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();// mvInvertedFile[i]表示包含了第i个word id的所有关键帧
    mvInvertedFile.resize(mpVoc->size());// mpVoc：预先训练好的词典
}

/**
 * @brief 在闭环检测中找到与该关键帧可能闭环的关键帧（注意不和当前帧连接）
 * Step 1：找出和当前帧具有公共单词的所有关键帧，不包括与当前帧连接的关键帧
 * Step 2：只和具有共同单词较多的（最大数目的80%以上）关键帧进行相似度计算 
 * Step 3：计算上述候选帧对应的共视关键帧组的总得分，只取最高组得分75%以上的组
 * Step 4：得到上述组中分数最高的关键帧作为闭环候选关键帧
 * @param[in] pKF               需要闭环检测的关键帧
 * @param[in] minScore          候选闭环关键帧帧和当前关键帧的BoW相似度至少要大于minScore
 * @return vector<KeyFrame*>    闭环候选关键帧
 */
vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)
{
    // 取出与当前关键帧相连（>15个共视地图点）的所有关键帧，这些相连关键帧都是局部相连，在闭环检测的时候将被剔除
    // 相连关键帧定义见 KeyFrame::UpdateConnections()
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();

    // 用于保存可能与当前关键帧形成闭环的候选帧（只要有相同的word，且不属于局部相连（共视）帧）
    list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    // Step 1：找出和当前帧具有公共单词的所有关键帧，不包括与当前帧连接的关键帧
    {
        unique_lock<mutex> lock(mMutex);

        // words是检测图像是否匹配的枢纽，遍历该pKF的每一个word
        // mBowVec 内部实际存储的是std::map<WordId, WordValue>
        // WordId 和 WordValue 表示Word在叶子中的id 和权重
        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            // 提取所有包含该word的KeyFrame
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];
            // 然后对这些关键帧展开遍历
            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                
                if(pKFi->mnLoopQuery!=pKF->mnId)    
                {
                    // 还没有标记为pKF的闭环候选帧
                    pKFi->mnLoopWords=0;
                    // 和当前关键帧共视的话不作为闭环候选帧
                    if(!spConnectedKeyFrames.count(pKFi))
                    {
                        // 没有共视就标记作为闭环候选关键帧，放到lKFsSharingWords里
                        pKFi->mnLoopQuery=pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;// 记录pKFi与pKF具有相同word的个数
            }
        }
    }

    // 如果没有关键帧和这个关键帧具有相同的单词,那么就返回空
    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    // Step 2：统计上述所有闭环候选帧中与当前帧具有共同单词最多的单词数，用来决定相对阈值 
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    // 确定最小公共单词数为最大公共单词数目的0.8倍
    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    // Step 3：遍历上述所有闭环候选帧，挑选出共有单词数大于minCommonWords且单词匹配度大于minScore存入lScoreAndMatch
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        // pKF只和具有共同单词较多（大于minCommonWords）的关键帧进行比较
        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;// 这个变量后面没有用到

            // 用mBowVec来计算两者的相似度得分
            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    // 如果没有超过指定相似度阈值的，那么也就直接跳过去
    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();


    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧相连（权值最高，共视程度最高）的前十个关键帧归为一组，计算累计得分
    // Step 4：计算上述候选帧对应的共视关键帧组的总得分，得到最高组得分bestAccScore，并以此决定阈值minScoreToRetain
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first; // 该组最高分数
        float accScore = it->first;  // 该组累计得分
        KeyFrame* pBestKF = pKFi;    // 该组最高分数对应的关键帧
        // 遍历共视关键帧，累计得分 
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            // 只有pKF2也在闭环候选帧中，且公共单词数超过最小要求，才能贡献分数
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;
                // 统计得到组里分数最高的关键帧
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        // 记录所有组中组得分最高的组，用于确定相对阈值
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    // 所有组中最高得分的0.75倍，作为最低阈值
    float minScoreToRetain = 0.75f*bestAccScore;

    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    // Step 5：只取组得分大于阈值的组，得到组中分数最高的关键帧们作为闭环候选关键帧
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            // spAlreadyAddedKF 是为了防止重复添加
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpLoopCandidates;
}

/*
 * @brief 在重定位中找到与该帧相似的候选关键帧组
 * Step 1. 找出和当前帧具有公共单词的所有关键帧
 * Step 2. 只和具有共同单词较多的关键帧进行相似度计算
 * Step 3. 将与关键帧相连（权值最高）的前十个关键帧归为一组，计算累计得分
 * Step 4. 只返回累计得分较高的组中分数最高的关键帧
 * @param F 需要重定位的帧
 * @return  相似的候选关键帧数组
 */
vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
    list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    // Step 1：找出和当前帧具有公共单词(word)的所有关键帧
    {
        unique_lock<mutex> lock(mMutex);

        // mBowVec 内部实际存储的是std::map<WordId, WordValue>
        // WordId 和 WordValue 表示Word在叶子中的id 和权重
        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            // 根据倒排索引，提取所有包含该wordid的所有KeyFrame
            list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                // pKFi->mnRelocQuery起标记作用，是为了防止重复选取
                if(pKFi->mnRelocQuery!=F->mnId)
                {
                    // pKFi还没有标记为F的重定位候选帧
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    // 如果和当前帧具有公共单词的关键帧数目为0，无法进行重定位，返回空
    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    // Only compare against those keyframes that share enough words
    // Step 2：统计上述关键帧中与当前帧F具有共同单词最多的单词数maxCommonWords，用来设定阈值1
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    // 阈值1：最小公共单词数为最大公共单词数目的0.8倍
    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    // Step 3：遍历上述关键帧，挑选出共有单词数大于阈值1的及其和当前帧单词匹配得分存入lScoreAndMatch
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        // 当前帧F只和具有共同单词较多（大于minCommonWords）的关键帧进行比较
        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;  // 这个变量后面没有用到
            // 用mBowVec来计算两者的相似度得分
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    // Step 4：计算lScoreAndMatch中每个关键帧的共视关键帧组的总得分，得到最高组得分bestAccScore，并以此决定阈值2
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧共视程度最高的前十个关键帧归为一组，计算累计得分
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        // 取出与关键帧pKFi共视程度最高的前10个关键帧
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        // 该组最高分数
        float bestScore = it->first; 
        // 该组累计得分
        float accScore = bestScore;  
        // 该组最高分数对应的关键帧
        KeyFrame* pBestKF = pKFi;   
        // 遍历共视关键帧，累计得分 
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;
            // 只有pKF2也在重定位候选帧中，才能贡献分数
            accScore+=pKF2->mRelocScore;

            // 统计得到组里分数最高的KeyFrame
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));

        // 记录所有组中最高的得分
        if(accScore>bestAccScore) 
            bestAccScore=accScore; 
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    // Step 5：得到所有组中总得分大于阈值2的，组内得分最高的关键帧，作为候选关键帧组
    //阈值2：最高得分的0.75倍
    float minScoreToRetain = 0.75f*bestAccScore; 
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        // 只返回累计得分大于阈值2的组中分数最高的关键帧
        if(si>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            // 判断该pKFi是否已经添加在队列中了
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

} //namespace ORB_SLAM
