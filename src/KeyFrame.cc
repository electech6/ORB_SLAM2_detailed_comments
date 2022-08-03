/**
 * @file KeyFrame.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 关键帧
 * @version 0.1
 * @date 2019-04-24
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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

namespace ORB_SLAM2
{

// 下一个关键帧的id
long unsigned int KeyFrame::nNextId=0;

//关键帧的构造函数
KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), 
    mHalfBaseline(F.mb/2),      // 计算双目相机长度的一半
    mpMap(pMap)
{
    // 获取id
    mnId=nNextId++;

    // 根据指定的普通帧, 初始化用于加速匹配的网格对象信息; 其实就把每个网格中有的特征点的索引复制过来
    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    // 设置当前关键帧的位姿
    SetPose(F.mTcw);
}

// Bag of Words Representation 计算词袋表示
void KeyFrame::ComputeBoW()
{
    // 只有当词袋向量或者节点和特征序号的特征向量为空的时候执行
    if(mBowVec.empty() || mFeatVec.empty())
    {
        // 那么就从当前帧的描述子中转换得到词袋信息
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise  //?
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

// 设置当前关键帧的位姿
void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    // 和普通帧中进行的操作相同
    Ow = -Rwc*tcw;

    // 计算当前位姿的逆
    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));

    // center为相机坐标系（左目）下，立体相机中心的坐标
    // 立体相机中心点坐标与左目相机坐标之间只是在x轴上相差mHalfBaseline,
    // 因此可以看出，立体相机中两个摄像头的连线为x轴，正方向为左目相机指向右目相机 (齐次坐标)
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    // 世界坐标系下，左目相机中心到立体相机中心的向量，方向由左目相机指向立体相机中心
    Cw = Twc*center;
}

// 获取位姿
cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

// 获取位姿的逆
cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

// 获取(左目)相机的中心在世界坐标系下的坐标
cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

// 获取双目相机的中心,这个只有在可视化的时候才会用到
cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}

// 获取姿态
cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

// 获取位置
cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

/**
 * @brief 为当前关键帧新建或更新和其他关键帧的连接权重
 * 
 * @param[in] pKF       和当前关键帧共视的其他关键帧
 * @param[in] weight    当前关键帧和其他关键帧的权重（共视地图点数目）
 */
void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        // 互斥锁，防止同时操作共享数据产生冲突
        unique_lock<mutex> lock(mMutexConnections);

        // 新建或更新连接权重
        if(!mConnectedKeyFrameWeights.count(pKF)) 
            // count函数返回0，说明mConnectedKeyFrameWeights中没有pKF，新建连接
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight) 
            // 之前连接的权重不一样了，需要更新
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    // 连接关系变化就要更新最佳共视，主要是重新进行排序
    UpdateBestCovisibles();
}

/**
 * @brief 按照权重从大到小对连接（共视）的关键帧进行排序
 * 
 * 更新后的变量存储在mvpOrderedConnectedKeyFrames和mvOrderedWeights中
 */
void KeyFrame::UpdateBestCovisibles()
{
    // 互斥锁，防止同时操作共享数据产生冲突
    unique_lock<mutex> lock(mMutexConnections);
    // http://stackoverflow.com/questions/3389648/difference-between-stdliststdpair-and-stdmap-in-c-stl (std::map 和 std::list<std::pair>的区别)
    
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    // 取出所有连接的关键帧，mConnectedKeyFrameWeights的类型为std::map<KeyFrame*,int>，而vPairs变量将共视的地图点数放在前面，利于排序
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    // 按照权重进行排序（默认是从小到大）
    sort(vPairs.begin(),vPairs.end());

    // 为什么要用链表保存？因为插入和删除操作方便，只需要修改上一节点位置，不需要移动其他元素
    list<KeyFrame*> lKFs;   // 所有连接关键帧
    list<int> lWs;          // 所有连接关键帧对应的权重（共视地图点数目）
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        // push_front 后变成从大到小
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    // 权重从大到小排列的连接关键帧
    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    // 从大到小排列的权重，和mvpOrderedConnectedKeyFrames一一对应
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

// 得到与该关键帧连接（>15个共视地图点）的关键帧(没有排序的)
set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);

    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

// 得到与该关键帧连接的关键帧(已按权值排序)
vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}


/**
 * @brief 得到与该关键帧连接的前N个最强共视关键帧(已按权值排序)
 * 
 * @param[in] N                 设定要取出的关键帧数目              
 * @return vector<KeyFrame*>    满足权重条件的关键帧集合
 */
vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);

    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        // 如果总数不够，就返回所有的关键帧
        return mvpOrderedConnectedKeyFrames;
    else
        // 取前N个最强共视关键帧
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);
}


/**
 * @brief 得到与该关键帧连接的权重超过w的关键帧
 * 
 * @param[in] w                 权重阈值
 * @return vector<KeyFrame*>    满足权重条件的关键帧向量
 */
vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    // 如果没有和当前关键帧连接的关键帧，直接返回空
    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    // http://www.cplusplus.com/reference/algorithm/upper_bound/
    // 从mvOrderedWeights找出第一个大于w的那个迭代器
    vector<int>::iterator it = upper_bound( mvOrderedWeights.begin(),   //起点
                                            mvOrderedWeights.end(),     //终点
                                            w,                          //目标阈值
                                            KeyFrame::weightComp);      //比较函数从大到小排序
    
    // 如果没有找到，说明最大的权重也比给定的阈值小，返回空
    if(it==mvOrderedWeights.end() && *mvOrderedWeights.rbegin()<w)
        return vector<KeyFrame*>();
    else
    {
        // 如果存在，返回满足要求的关键帧
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

// 得到该关键帧与pKF的权重
int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        // 没有连接的话权重也就是共视点个数就是0
        return 0;
}

// Add MapPoint to KeyFrame
void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}


/**
 * @brief 由于其他的原因,导致当前关键帧观测到的某个地图点被删除(bad==true)了,将该地图点置为NULL
 * 
 * @param[in] idx   地图点在该关键帧中的id
 */
void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    // NOTE 使用这种方式表示其中的某个地图点被删除
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

// 同上
void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    //获取当前地图点在某个关键帧的观测中，对应的特征点的索引，如果没有观测，索引为-1
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

// 地图点的替换
void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

// 获取当前关键帧中的所有地图点
set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);

    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        // 判断是否被删除了
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        // 如果是没有来得及删除的坏点也要进行这一步
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

// 关键帧中，大于等于最少观测数目minObs的MapPoints的数量.这些特征点被认为追踪到了
int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    // 是否检查数目
    const bool bCheckObs = minObs>0;
    // N是当前帧中特征点的个数
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)     //没有被删除
        {
            if(!pMP->isBad())   //并且不是坏点
            {
                if(bCheckObs)
                {
                    // 满足输入阈值要求的地图点计数加1
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++; //!bug
            }
        }
    }

    return nPoints;
}

// 获取当前关键帧的具体的地图点
vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

// 获取当前关键帧的具体的某个地图点
MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

/*
 * 更新关键帧之间的连接图
 * 
 * 1. 首先获得该关键帧的所有MapPoint点，统计观测到这些3d点的每个关键帧与其它所有关键帧之间的共视程度
 *    对每一个找到的关键帧，建立一条边，边的权重是该关键帧与当前关键帧公共3d点的个数。
 * 2. 并且该权重必须大于一个阈值，如果没有超过该阈值的权重，那么就只保留权重最大的边（与其它关键帧的共视程度比较高）
 * 3. 对这些连接按照权重从大到小进行排序，以方便将来的处理
 *    更新完covisibility图之后，如果没有初始化过，则初始化为连接权重最大的边（与其它关键帧共视程度最高的那个关键帧），类似于最大生成树
 */
void KeyFrame::UpdateConnections()
{
    // 关键帧-权重，权重为其它关键帧与当前关键帧共视地图点的个数，也称为共视程度
    map<KeyFrame*,int> KFcounter; 
    vector<MapPoint*> vpMP;

    {
        // 获得该关键帧的所有地图点
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    // Step 1 通过地图点被关键帧观测来间接统计关键帧之间的共视程度
    // 统计每一个地图点都有多少关键帧与当前关键帧存在共视关系，统计结果放在KFcounter
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        // 对于每一个地图点，observations记录了可以观测到该地图点的所有关键帧
        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            // 除去自身，自己与自己不算共视
            if(mit->first->mnId==mnId)
                continue;
            // 这里的操作非常精彩！
            // map[key] = value，当要插入的键存在时，会覆盖键对应的原来的值。如果键不存在，则添加一组键值对
            // mit->first 是地图点看到的关键帧，同一个关键帧看到的地图点会累加到该关键帧计数
            // 所以最后KFcounter 第一个参数表示某个关键帧，第2个参数表示该关键帧看到了多少当前帧的地图点，也就是共视程度
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    // 没有共视关系，直接退出 
    if(KFcounter.empty())
        return;

    // If the counter is greater than threshold add connection
    // In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0; // 记录最高的共视程度
    KeyFrame* pKFmax=NULL;
    // 至少有15个共视地图点才会添加共视关系
    int th = 15;

    // vPairs记录与其它关键帧共视帧数大于th的关键帧
    // pair<int,KeyFrame*>将关键帧的权重写在前面，关键帧写在后面方便后面排序
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    // Step 2 找到对应权重最大的关键帧（共视程度最高的关键帧）
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }

        // 建立共视关系至少需要大于等于th个共视地图点
        if(mit->second>=th)
        {
            // 对应权重需要大于阈值，对这些关键帧建立连接
            vPairs.push_back(make_pair(mit->second,mit->first));
            // 对方关键帧也要添加这个信息
            // 更新KFcounter中该关键帧的mConnectedKeyFrameWeights
            // 更新其它KeyFrame的mConnectedKeyFrameWeights，更新其它关键帧与当前帧的连接权重
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    //  Step 3 如果没有超过阈值的权重，则对权重最大的关键帧建立连接
    if(vPairs.empty())
    {
	    // 如果每个关键帧与它共视的关键帧的个数都少于th，
        // 那就只更新与其它关键帧共视程度最高的关键帧的mConnectedKeyFrameWeights
        // 这是对之前th这个阈值可能过高的一个补丁
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    //  Step 4 对满足共视程度的关键帧对更新连接关系及权重（从大到小）
    // vPairs里存的都是相互共视程度比较高的关键帧和共视权重，接下来由大到小进行排序
    sort(vPairs.begin(),vPairs.end());         // sort函数默认升序排列
    // 将排序后的结果分别组织成为两种数据类型
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        // push_front 后变成了从大到小顺序
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        // 更新当前帧与其它关键帧的连接权重
        // ?bug 这里直接赋值，会把小于阈值的共视关系也放入mConnectedKeyFrameWeights，会增加计算量
        // ?但后续主要用mvpOrderedConnectedKeyFrames来取共视帧，对结果没影响
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        // Step 5 更新生成树的连接
        if(mbFirstConnection && mnId!=0)
        {
            // 初始化该关键帧的父关键帧为共视程度最高的那个关键帧
            mpParent = mvpOrderedConnectedKeyFrames.front();
            // 建立双向连接关系，将当前关键帧作为其子关键帧
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }
    }
}


// 添加子关键帧（即和子关键帧具有最大共视关系的关键帧就是当前关键帧）
void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

// 删除某个子关键帧
void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

// 改变当前关键帧的父关键帧
void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    // 添加双向连接关系
    mpParent = pKF;
    pKF->AddChild(this);
}

//获取当前关键帧的子关键帧
set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

//获取当前关键帧的父关键帧
KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

// 判断某个关键帧是否是当前关键帧的子关键帧
bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}


// 给当前关键帧添加回环边，回环边连接了形成闭环关系的关键帧
void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

// 获取和当前关键帧形成闭环关系的关键帧
set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

// 设置当前关键帧不要在优化的过程中被删除. 由回环检测线程调用
void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

/**
 * @brief 删除当前的这个关键帧,表示不进行回环检测过程;由回环检测线程调用
 * 
 */
void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);

        // 如果当前关键帧和其他的关键帧没有形成回环关系,那么就删吧
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    // mbToBeErased：删除之前记录的想要删但时机不合适没有删除的帧
    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

/**
 * @brief 真正地执行删除关键帧的操作
 * 需要删除的是该关键帧和其他所有帧、地图点之间的连接关系
 * 
 * mbNotErase作用：表示要删除该关键帧及其连接关系但是这个关键帧有可能正在回环检测或者计算sim3操作，这时候虽然这个关键帧冗余，但是却不能删除，
 * 仅设置mbNotErase为true，这时候调用setbadflag函数时，不会将这个关键帧删除，只会把mbTobeErase变成true，代表这个关键帧可以删除但不到时候,先记下来以后处理。
 * 在闭环线程里调用 SetErase()会根据mbToBeErased 来删除之前可以删除还没删除的帧。
 */
void KeyFrame::SetBadFlag()
{   
    // Step 1 首先处理一下删除不了的特殊情况
    {
        unique_lock<mutex> lock(mMutexConnections);

        // 第0关键帧不允许被删除
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            // mbNotErase表示不应该删除，于是把mbToBeErased置为true，假装已经删除，其实没有删除
            mbToBeErased = true;
            return;
        }
    }

    // Step 2 遍历所有和当前关键帧相连的关键帧，删除他们与当前关键帧的联系
    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this); // 让其它的关键帧删除与自己的联系

    // Step 3 遍历每一个当前关键帧的地图点，删除每一个地图点和当前关键帧的联系
    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this); 

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        // 清空自己与其它关键帧之间的联系
        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree 
        // Step 4 更新生成树，主要是处理好父子关键帧，不然会造成整个关键帧维护的图断裂，或者混乱
        // 候选父关键帧
        set<KeyFrame*> sParentCandidates;
        // 将当前帧的父关键帧放入候选父关键帧
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        // 每迭代一次就为其中一个子关键帧寻找父关键帧（最高共视程度），找到父的子关键帧可以作为其他子关键帧的候选父关键帧
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            // Step 4.1 遍历每一个子关键帧，让它们更新它们指向的父关键帧
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                // 跳过无效的子关键帧
                if(pKF->isBad())    
                    continue;

                // Check if a parent candidate is connected to the keyframe
                // Step 4.2 子关键帧遍历每一个与它共视的关键帧    
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();

                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    // sParentCandidates 中刚开始存的是这里子关键帧的“爷爷”，也是当前关键帧的候选父关键帧
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        // Step 4.3 如果孩子和sParentCandidates中有共视，选择共视最强的那个作为新的父
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            // 寻找并更新权值最大的那个共视关系
                            if(w>max)
                            {
                                pC = pKF;                   //子关键帧
                                pP = vpConnected[i];        //目前和子关键帧具有最大权值的关键帧（将来的父关键帧） 
                                max = w;                    //这个最大的权值
                                bContinue = true;           //说明子节点找到了可以作为其新父关键帧的帧
                            }
                        }
                    }
                }
            }

            // Step 4.4 如果在上面的过程中找到了新的父节点
            // 下面代码应该放到遍历子关键帧循环中?
            // 回答：不需要！这里while循环还没退出，会使用更新的sParentCandidates
            if(bContinue)
            {
                // 因为父节点死了，并且子节点找到了新的父节点，就把它更新为自己的父节点
                pC->ChangeParent(pP);
                // 因为子节点找到了新的父节点并更新了父节点，那么该子节点升级，作为其它子节点的备选父节点
                sParentCandidates.insert(pC);
                // 该子节点处理完毕，删掉
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        // Step 4.5 如果还有子节点没有找到新的父节点
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                // 直接把父节点的父节点作为自己的父节点 即对于这些子节点来说,他们的新的父节点其实就是自己的爷爷节点
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        // mTcp 表示原父关键帧到当前关键帧的位姿变换，在保存位姿的时候使用
        mTcp = Tcw*mpParent->GetPoseInverse();
        // 标记当前关键帧已经挂了
        mbBad = true;
    }  

    // 地图和关键帧数据库中删除该关键帧
    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

// 返回当前关键帧是否已经完蛋了
bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

// 删除当前关键帧和指定关键帧之间的共视关系
void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    // 其实这个应该表示是否真的是有共视关系
    bool bUpdate = false;

    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    // 如果是真的有共视关系,那么删除之后就要更新共视关系
    if(bUpdate)
        UpdateBestCovisibles();
}

// 获取某个特征点的邻域中的特征点id,其实这个和 Frame.cc 中的那个函数基本上都是一致的; r为边长（半径）
vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    // 计算要搜索的cell的范围

    // floor向下取整，mfGridElementWidthInv 为每个像素占多少个格子
    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    // ceil向上取整
    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    // 遍历每个cell,取出其中每个cell中的点,并且每个点都要计算是否在邻域内
    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

// 判断某个点是否在当前关键帧的图像中
bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

/**
 * @brief 在双目和RGBD情况下将特征点反投影到空间中得到世界坐标系下三维点
 * 
 * @param[in] i                 第i个特征点
 * @return cv::Mat              返回世界坐标系下三维点
 */
cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        // 由2维图像反投影到相机坐标系
        // 双目中mvDepth是在ComputeStereoMatches函数中求取的，rgbd中是直接测量的
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        // 由相机坐标系转换到世界坐标系
        // Twc为相机坐标系到世界坐标系的变换矩阵
        // Twc.rosRange(0,3).colRange(0,3)取Twc矩阵的前3行与前3列
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

// Compute Scene Depth (q=2 median). Used in monocular. 评估当前关键帧场景深度，q=2表示中值. 只是在单目情况下才会使用
// 其实过程就是对当前关键帧下所有地图点的深度进行从小到大排序,返回距离头部其中1/q处的深度值作为当前场景的平均深度
float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    // 遍历每一个地图点,计算并保存其在当前关键帧下的深度
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw; // (R*x3Dw+t)的第三行，即z
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
