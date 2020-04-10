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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
//? 记得查看它都在什么地方被使用
mutex MapPoint::mGlobalMutex;

/*
 * @brief 给定坐标与keyframe构造MapPoint
 *
 * 双目：StereoInitialization()，CreateNewKeyFrame()，LocalMapping::CreateNewMapPoints()
 * 单目：CreateInitialMapMonocular()，LocalMapping::CreateNewMapPoints()
 * @param Pos    MapPoint的坐标（wrt世界坐标系）
 * @param pRefKF KeyFrame
 * @param pMap   Map
 */
MapPoint::MapPoint(const cv::Mat &Pos,  //地图点的世界坐标
                   KeyFrame *pRefKF,    //生成地图点的关键帧
                   Map* pMap):          //地图点所存在的地图
    mnFirstKFid(pRefKF->mnId),              //第一次观测/生成它的关键帧 id
    mnFirstFrame(pRefKF->mnFrameId),        //创建该地图点的帧ID(因为关键帧也是帧啊)
    nObs(0),                                //被观测次数
    mnTrackReferenceForFrame(0),            //放置被重复添加到局部地图点的标记
    mnLastFrameSeen(0),                     //是否决定判断在某个帧视野中的变量
    mnBALocalForKF(0),                      //?
    mnFuseCandidateForKF(0),                //?
    mnLoopPointForKF(0),                    //?
    mnCorrectedByKF(0),                     //?
    mnCorrectedReference(0),                //?
    mnBAGlobalForKF(0),                     //? 
    mpRefKF(pRefKF),                        //? 参考关键帧? 是什么意思
    mnVisible(1),                           //在帧中的可视次数
    mnFound(1),                             //被找到的次数 和上面的相比要求能够匹配上
    mbBad(false),                           //坏点标记
    mpReplaced(static_cast<MapPoint*>(NULL)), //替换掉当前地图点的点
    mfMinDistance(0),                       //当前地图点在某帧下,可信赖的被找到时其到关键帧光心距离的下界
    mfMaxDistance(0),                       //上界
    mpMap(pMap)                             //从属地图
{
    Pos.copyTo(mWorldPos);
    //平均观测方向初始化为0
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

/*
 * @brief 给定坐标与frame构造MapPoint
 *
 * 双目：UpdateLastFrame()
 * @param Pos    MapPoint的坐标（wrt世界坐标系）
 * @param pMap   Map     
 * @param pFrame Frame
 * @param idxF   MapPoint在Frame中的索引，即对应的特征点的编号
 */
MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;// 世界坐标系下相机到3D点的向量 (当前关键帧的观测方向)
    mNormalVector = mNormalVector/cv::norm(mNormalVector);// 单位化

    //这个算重了吧
    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);    //到相机的距离
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    // 另见 PredictScale 函数前的注释
    /* 666,因为在提取特征点的时候, 考虑到了图像的尺度问题,因此在不同图层上提取得到的特征点,对应着特征点距离相机的远近
       不同, 所以在这里生成地图点的时候,也要再对其进行确认
       虽然我们拿不到每个图层之间确定的尺度信息,但是我们有缩放比例这个相对的信息哇
    */
    mfMaxDistance = dist*levelScaleFactor;                              //当前图层的"深度"
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];    //该特征点上一个图层的"深度""

    // 见 mDescriptor 在MapPoint.h中的注释 ==> 其实就是获取这个地图点的描述子
    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    // TODO 不太懂,怎么个冲突法? 
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

//设置地图点在世界坐标系下的坐标
void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    //TODO 为什么这里多了个线程锁
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}
//获取地图点在世界坐标系下的坐标
cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}
//设置地图点的平均观测方向
cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}
//获取地图点的参考关键帧
KeyFrame* MapPoint::GetReferenceKeyFrame()
{
     unique_lock<mutex> lock(mMutexFeatures);
     return mpRefKF;
}

/**
 * @brief 添加观测
 *
 * 记录哪些 KeyFrame 的那个特征点能观测到该 MapPoint \n
 * 并增加观测的相机数目nObs，单目+1，双目或者grbd+2
 * 这个函数是建立关键帧共视关系的核心函数，能共同观测到某些MapPoints的关键帧是共视关键帧
 * @param pKF KeyFrame
 * @param idx MapPoint在KeyFrame中的索引
 */
void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    // 记录下能观测到该MapPoint的KF和该MapPoint在KF中的索引
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
        nObs+=2; // 双目或者grbd
    else
        nObs++; // 单目
}


// 删除某个关键帧对当前地图点的观测
void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        //查找这个要删除的观测,根据单目和双目类型的不同从其中删除当前地图点的被观测次数
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);

            // 如果该keyFrame是参考帧，该Frame被删除后重新指定RefFrame
            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            // 当观测到该点的相机数目少于2时，丢弃该点
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        // 告知可以观测到该MapPoint的Frame，该MapPoint已被删除
        SetBadFlag();
}

//获取对当前点的观测详情(目测是用于优化? TODO)
map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

//只是获取当前地图点的被观测次数
int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

// 告知可以观测到该MapPoint的Frame，该MapPoint已被删除
void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;// 把mObservations转存到obs，obs和mObservations里存的是指针，赋值过程为浅拷贝
        mObservations.clear();// 把mObservations指向的内存释放，obs作为局部变量之后自动删除
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);// 告诉可以观测到该MapPoint的KeyFrame，该MapPoint被删了
    }

    mpMap->EraseMapPoint(this);// 擦除该MapPoint申请的内存
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

// 在形成闭环的时候，会更新 KeyFrame 与 MapPoint 之间的关系
void MapPoint::Replace(MapPoint* pMP)
{
    //自己换自己,也就是不换,就直接跳过了
    if(pMP->mnId==this->mnId)
        return;

    //要替换当前地图点,有两个工作:
    // 1. 将当前地图点的观测数据等其他数据都"叠加"到新的地图点上
    // 2. 将观测到当前地图点的关键帧的信息进行更新

    //- 准备工作

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;// 这一段和SetBadFlag函数相同
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        //清除当前地图点的原有观测
        mObservations.clear();
        //当前的地图点被删除了
        mbBad=true;
        //暂存当前地图点的可视次数和被找到的次数
        nvisible = mnVisible;
        nfound = mnFound;
        //指明当前地图点已经被指定的地图点替换了
        mpReplaced = pMP;
    }

    // 所有能观测到该MapPoint的keyframe都要替换
    //- 将观测到当前地图的的关键帧的信息进行更新
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        //其中又有两种情况:
        //- 一种情况是, 这个关键帧中没有对"要替换本地图点的地图点"的观测

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);// 让KeyFrame用pMP替换掉原来的MapPoint
            pMP->AddObservation(pKF,mit->second);// 让MapPoint替换掉对应的KeyFrame
        }
        else
        {

            //- 另外一种情况是,这个关键帧对当前的地图点和"要替换本地图点的地图点"都具有观测
            // 产生冲突，即pKF中有两个特征点a,b（这两个特征点的描述子是近似相同的），这两个特征点对应两个 MapPoint 为this,pMP
            // 然而在fuse的过程中pMP的观测更多，需要替换this，因此保留b与pMP的联系，去掉a与this的联系
            //说白了,既然是让对方的那个地图点来代替当前的地图点,就是说明对方更好,所以删除这个关键帧对当前帧的观测
            pKF->EraseMapPointMatch(mit->second);
        }
    }

    //- 将当前地图点的观测数据等其他数据都"叠加"到新的地图点上
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    //描述子更新
    pMP->ComputeDistinctiveDescriptors();

    //告知地图,删掉我
    mpMap->EraseMapPoint(this);
}

// 没有经过 MapPointCulling 检测的MapPoints, 认为是坏掉的点
bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

/**
 * @brief Increase Visible
 *
 * Visible表示：
 * 1. 该MapPoint在某些帧的视野范围内，通过Frame::isInFrustum()函数判断
 * 2. 该MapPoint被这些帧观测到，但并不一定能和这些帧的特征点匹配上
 *    例如：有一个MapPoint（记为M），在某一帧F的视野范围内，
 *    但并不表明该点M可以和F这一帧的某个特征点能匹配上
 * TODO  所以说，found 就是表示匹配上了嘛？
 */
void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

/**
 * @brief Increase Found
 *
 * 能找到该点的帧数+n，n默认为1
 * @see Tracking::TrackLocalMap()
 */
void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

// 计算被找到的比例
float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

/**
 * @brief 计算具有代表的描述子
 *
 * 由于一个MapPoint会被许多相机观测到，因此在插入关键帧后，需要判断是否更新当前点的最适合的描述子 \n
 * 先获得当前点的所有描述子，然后计算描述子之间的两两距离，最好的描述子与其他描述子应该具有最小的距离中值
 * @see III - C3.3
 */
void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    //获取所有观测
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    // 遍历观测到3d点的所有关键帧，获得orb描述子，并插入到vDescriptors中
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    // 获得这些描述子两两之间的距离
    const size_t N = vDescriptors.size();
	
    //float Distances[N][N];
	std::vector<std::vector<float> > Distances;
	Distances.resize(N, vector<float>(N, 0));
	for (size_t i = 0; i<N; i++)
    {
        //和自己的距离当然是0
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        // 第i个描述子到其它所有所有描述子之间的距离
        //vector<int> vDists(Distances[i],Distances[i]+N);
		vector<int> vDists(Distances[i].begin(), Distances[i].end());
		sort(vDists.begin(), vDists.end());

        // 获得中值
        int median = vDists[0.5*(N-1)];
        
        // 寻找最小的中值
        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        
        // 最好的描述子，该描述子相对于其他描述子有最小的距离中值
        // 简化来讲，中值代表了这个描述子到其它描述子的平均距离
        // 最好的描述子就是和其它描述子的平均距离最小
        mDescriptor = vDescriptors[BestIdx].clone();       
    }
}

//获取当前地图点的描述子
cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

//获取当前地图点在某个关键帧的观测中，对应的特征点的ID
int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

/**
 * @brief check MapPoint is in keyframe
 * @param  pKF KeyFrame
 * @return     true if in pKF
 */
bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

/**
 * @brief 更新平均观测方向以及观测距离范围
 *
 * 由于一个MapPoint会被许多相机观测到，因此在插入关键帧后，需要更新相应变量
 * 创建新的关键帧的时候会调用
 * 
 * @see III - C2.2 c2.4
 */
void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;

        observations=mObservations; // 获得观测到该3d点的所有关键帧
        pRefKF=mpRefKF;             // 观测到该点的参考关键帧
        Pos = mWorldPos.clone();    // 3d点在世界坐标系中的位置
    }

    if(observations.empty())
        return;

    //初始值为0向量用于累加;但是放心每次累加的变量都是经过归一化之后的
    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali); // 对所有关键帧对该点的观测方向归一化为单位向量进行求和
        n++;
    } 

    cv::Mat PC = Pos - pRefKF->GetCameraCenter(); // 参考关键帧相机指向3D点的向量（在世界坐标系下的表示）
    const float dist = cv::norm(PC); // 该点到参考关键帧相机的距离
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels; // 金字塔层数

    {
        unique_lock<mutex> lock3(mMutexPos);
        // 另见PredictScale函数前的注释
        mfMaxDistance = dist*levelScaleFactor;                           // 观测到该点的距离下限
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1]; // 观测到该点的距离上限
        mNormalVector = normal/n;                                        // 获得平均的观测方向
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

// 下图中横线的大小表示不同图层图像上的一个像素表示的真实物理空间中的大小
//              ____
// Nearer      /____\     level:n-1 --> dmin
//            /______\                       d/dmin = 1.2^(n-1-m)
//           /________\   level:m   --> d
//          /__________\                     dmax/d = 1.2^m
// Farther /____________\ level:0   --> dmax
//
//           log(dmax/d)
// m = ceil(------------)
//            log(1.2)
// 这里吴博师兄的PPT中貌似有讲
// 这个函数的作用:
// 在进行投影匹配的时候会给定特征点的搜索范围,考虑到处于不同尺度(也就是距离相机远近,位于图像金字塔中不同图层)的特征点受到相机旋转的影响不同,
// 因此会希望距离相机近的点的搜索范围更大一点,距离相机更远的点的搜索范围更小一点,所以要在这里,根据点到关键帧/帧的距离来估计它在当前的关键帧/帧中,
// 会大概处于哪个尺度
int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        // mfMaxDistance = ref_dist*levelScaleFactor 为参考帧考虑上尺度后的距离
        // ratio = mfMaxDistance/currentDist = ref_dist/cur_dist
        ratio = mfMaxDistance/currentDist;
    }

    // 同时取log线性化
    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}



} //namespace ORB_SLAM
