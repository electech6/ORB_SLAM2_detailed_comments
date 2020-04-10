/**
 * @file KeyFrame.h
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


#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>

namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

/**
 * @brief 关键帧类
 * @detials 关键帧，和普通的Frame不一样，但是可以由Frame来构造; 许多数据会被三个线程同时访问，所以用锁的地方很普遍
 * 
 */
class KeyFrame
{
public:

    /**
     * @brief 构造函数
     * @param[in] F         父类普通帧的对象
     * @param[in] pMap      所属的地图指针
     * @param[in] pKFDB     使用的词袋模型的指针
     */
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    // 这里的set,get需要用到锁

    /**
     * @brief 设置当前关键帧的位姿
     * @param[in] Tcw 位姿
     */
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();                  ///< 获取位姿
    cv::Mat GetPoseInverse();           ///< 获取位姿的逆
    cv::Mat GetCameraCenter();          ///< 获取(左目)相机的中心
    cv::Mat GetStereoCenter();          ///< 获取双目相机的中心,这个只有在可视化的时候才会用到
    cv::Mat GetRotation();              ///< 获取姿态
    cv::Mat GetTranslation();           ///< 获取位置

    /**
     * @brief Bag of Words Representation
     * @detials 计算mBowVec，并且将描述子分散在第4层上，即mFeatVec记录了属于第i个node的ni个描述子
     * @see ProcessNewKeyFrame()
     */
    void ComputeBoW();

    // ====================== Covisibility graph functions ============================

    /**
     * @brief 为关键帧之间添加连接
     * @details 更新了mConnectedKeyFrameWeights
     * @param pKF    关键帧
     * @param weight 权重，该关键帧与pKF共同观测到的3d点数量
     */
    void AddConnection(KeyFrame* pKF, const int &weight);
    /**
     * @brief 删除当前关键帧和指定关键帧之间的共视关系
     * @param[in] pKF 要删除的共视关系
     */
    void EraseConnection(KeyFrame* pKF);
    /** @brief 更新图的连接  */
    void UpdateConnections();
    /**
     * @brief 按照权重对连接的关键帧进行排序
     * @detials 更新后的变量存储在mvpOrderedConnectedKeyFrames和mvOrderedWeights中
     */
    void UpdateBestCovisibles();
    /**
     * @brief 得到与该关键帧连接的关键帧(没有排序的)
     * @return 连接的关键帧
     */
    std::set<KeyFrame *> GetConnectedKeyFrames();
    /**
     * @brief 得到与该关键帧连接的关键帧(已按权值排序)
     * @return 连接的关键帧
     */
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    /**
     * @brief 得到与该关键帧连接的前N个关键帧(已按权值排序)
     * NOTICE 如果连接的关键帧少于N，则返回所有连接的关键帧,所以说返回的关键帧的数目其实不一定是N个
     * @param N 前N个
     * @return 连接的关键帧
     */
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    /**
     * @brief 得到与该关键帧连接的权重大于等于w的关键帧
     * @param w 权重
     * @return 连接的关键帧
     */
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    /**
     * @brief 得到该关键帧与pKF的权重
     * @param  pKF 关键帧
     * @return     权重
     */
    int GetWeight(KeyFrame* pKF);

    // ========================= Spanning tree functions =======================
    /**
     * @brief 添加子关键帧（即和子关键帧具有最大共视关系的关键帧就是当前关键帧）
     * @param[in] pKF 子关键帧句柄
     */
    void AddChild(KeyFrame* pKF);
    /**
     * @brief 删除某个子关键帧
     * @param[in] pKF 子关键帧句柄
     */
    void EraseChild(KeyFrame* pKF);
    /**
     * @brief 改变当前关键帧的父关键帧
     * @param[in] pKF 父关键帧句柄
     */
    void ChangeParent(KeyFrame* pKF);
    /**
     * @brief 获取获取当前关键帧的子关键帧 
     * @return std::set<KeyFrame*>  子关键帧集合
     */
    std::set<KeyFrame*> GetChilds();
    /**
     * @brief 获取当前关键帧的父关键帧
     * @return KeyFrame* 父关键帧句柄
     */
    KeyFrame* GetParent();
    /**
     * @brief 判断某个关键帧是否是当前关键帧的子关键帧
     * @param[in] pKF 关键帧句柄
     * @return true 
     * @return false 
     */
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    /**
     * @brief 给当前关键帧添加回环边，回环边连接了形成闭环关系的关键帧
     * @param[in] pKF  和当前关键帧形成闭环关系的关键帧
     */
    void AddLoopEdge(KeyFrame* pKF);
    /**
     * @brief 获取和当前关键帧形成闭环关系的关键帧
     * @return std::set<KeyFrame*> 结果
     */
    std::set<KeyFrame*> GetLoopEdges();

    // ====================== MapPoint observation functions ==================================
    /**
     * @brief Add MapPoint to KeyFrame
     * @param pMP MapPoint
     * @param idx MapPoint在KeyFrame中的索引
     */
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    /**
     * @brief 由于其他的原因,导致当前关键帧观测到的某个地图点被删除(bad==true)了,这里是"通知"当前关键帧这个地图点已经被删除了
     * @param[in] idx 被删除的地图点索引
     */
    void EraseMapPointMatch(const size_t &idx);
    /**
     * @brief 由于其他的原因,导致当前关键帧观测到的某个地图点被删除(bad==true)了,这里是"通知"当前关键帧这个地图点已经被删除了
     * @param[in] pMP 被删除的地图点指针
     */
    void EraseMapPointMatch(MapPoint* pMP);
    /**
     * @brief 地图点的替换
     * @param[in] idx 要替换掉的地图点的索引
     * @param[in] pMP 新地图点的指针
     */
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    /**
     * @brief 获取当前帧中的所有地图点
     * @return std::set<MapPoint*> 所有的地图点
     */
    std::set<MapPoint*> GetMapPoints();
    /**
     * @brief Get MapPoint Matches 获取该关键帧的MapPoints
     */
    std::vector<MapPoint*> GetMapPointMatches();
    /**
     * @brief 关键帧中，大于等于minObs的MapPoints的数量
     * @details minObs就是一个阈值，大于minObs就表示该MapPoint是一个高质量的MapPoint \n
     * 一个高质量的MapPoint会被多个KeyFrame观测到.
     * @param  minObs 最小观测
     */
    int TrackedMapPoints(const int &minObs);
    /**
     * @brief 获取获取当前关键帧的具体的某个地图点
     * @param[in] idx id
     * @return MapPoint* 地图点句柄 
     */
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    /**
     * @brief 获取某个特征点的邻域中的特征点id
     * @param[in] x 特征点坐标
     * @param[in] y 特征点坐标
     * @param[in] r 邻域大小(半径)
     * @return std::vector<size_t> 在这个邻域内找到的特征点索引的集合
     */
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    /**
     * @brief Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
     * @param  i 第i个keypoint
     * @return   3D点（相对于世界坐标系）
     */
    cv::Mat UnprojectStereo(int i);

    // Image
    /**
     * @brief 判断某个点是否在当前关键帧的图像中
     * @param[in] x 点的坐标
     * @param[in] y 点的坐标
     * @return true 
     * @return false 
     */
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    /** @brief 设置当前关键帧不要在优化的过程中被删除  */
    void SetNotErase();
    /** @brief 准备删除当前的这个关键帧,表示不进行回环检测过程;由回环检测线程调用 */
    void SetErase();

    // Set/check bad flag
    /** @brief 真正地执行删除关键帧的操作 */
    void SetBadFlag();
    /** @brief 返回当前关键帧是否已经完蛋了 */
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    /**
     * @brief 评估当前关键帧场景深度，q=2表示中值
     * @param q q=2
     * @return Median Depth
     */
    float ComputeSceneMedianDepth(const int q);

    /// 比较两个int型权重的大小的比较函数
    static bool weightComp( int a, int b)
	{
        return a>b;
    }

    // ? 暂时没有使用到
    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2)
	{
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    /// nNextID名字改为nLastID更合适，表示上一个KeyFrame的ID号
    static long unsigned int nNextId;
    /// 在nNextID的基础上加1就得到了mnID，为当前KeyFrame的ID号
    long unsigned int mnId;
    /// 每个KeyFrame基本属性是它是一个Frame，KeyFrame初始化的时候需要Frame，
    /// mnFrameId记录了该KeyFrame是由哪个Frame初始化的
    const long unsigned int mnFrameId;

    /// 时间戳
    const double mTimeStamp;

    // ? 既然这么多和Frame重复的地方,为什么不选择使用继承呢?
    // Grid (to speed up feature matching)
    // 和Frame类中的定义相同
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame; 
    long unsigned int mnFuseTargetForKF;        ///< 标记在局部建图线程中,和哪个关键帧进行融合的操作

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;           ///< 在局部建图线程调用局部优化的时候使用,表示当前关键帧能够观测到某个地图点电视却不属于局部关键帧,so这个关键帧
                                                ///< 只是提供约束信息但是却不会去优化这个关键帧;为了避免重复添加,这里要记录触发优化的关键帧的id

    // Variables used by the keyframe database 下面的这些变量都是临时的,由外部调用暂时存放一些数据
    /// 标记了当前关键帧是id为mnLoopQuery的回环检测的候选关键帧
    long unsigned int mnLoopQuery;
    /// 当前关键帧和这个形成回环的候选关键帧中,具有相同word的个数
    int mnLoopWords;
    /// 和那个形成回环的关键帧的词袋匹配程度的评分
    float mLoopScore;
    // 和上面的变量作用差不多，不过这个变量是用来存储在辅助进行重定位的时候，要进行重定位的那个帧的id
    long unsigned int mnRelocQuery;
    /// 和那个要进行重定位的帧,所具有相同的单词的个数
    int mnRelocWords;
    /// 还有和那个帧的词袋的相似程度的评分
    float mRelocScore;

    // Variables used by loop closing
    // 经过全局BA优化后的相机的位姿
    cv::Mat mTcwGBA;
    // 进行全局BA优化之前的当前关键帧的位姿. 之所以要记录这个是因为在全局优化之后还要根据该关键帧在优化之前的位姿来更新地图点,which地图点的参考关键帧就是该关键帧
    cv::Mat mTcwBefGBA;
    // 记录是由于哪个"当前关键帧"触发的全局BA,用来防止重复写入的事情发生(浪费时间)
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    /// Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    // 和Frame类中的定义相同
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec; ///< Vector of words to represent images 当前图像的词袋模型表示
    DBoW2::FeatureVector mFeatVec; ///< Vector of nodes with indexes of local features //?

    /// Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;// 尺度因子，scale^n，scale=1.2，n为层数
    const std::vector<float> mvLevelSigma2;// 尺度因子的平方
    const std::vector<float> mvInvLevelSigma2;

    /// Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe. ---- 但是大哥..protected也不是这样设计使用的啊
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;    ///< 当前相机的位姿
    cv::Mat Twc;    ///< 当前相机位姿的逆
    cv::Mat Ow;     ///< 相机光心(左目)在世界坐标系下的坐标,这里和普通帧中的定义是一样的

    cv::Mat Cw; ///< Stereo middel point. Only for visualization

    /// MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    /// 词袋对象,目测是封装了很多操作啊
    ORBVocabulary* mpORBvocabulary;

    /// Grid over the image to speed up feature matching ,其实应该说是二维的,第三维的 vector中保存的是这个网格内的特征点的索引
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    // Covisibility Graph
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;              ///< 与该关键帧连接的关键帧与权重
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;            ///< 排序后的关键帧,和下面的这个变量相对应
    std::vector<int> mvOrderedWeights;                              ///< 排序后的权重(从大到小)

    // ===================== Spanning Tree and Loop Edges ========================
    // std::set是集合，相比vector，进行插入数据这样的操作时会自动排序
    bool mbFirstConnection;                     ///< 是否是第一次生成树
    KeyFrame* mpParent;                         ///< 当前关键帧的父关键帧
    std::set<KeyFrame*> mspChildrens;           ///< 存储当前关键帧的子关键帧
    std::set<KeyFrame*> mspLoopEdges;           ///< 和当前关键帧形成回环关系的关键帧

    // Bad flags
    bool mbNotErase;            ///< 当前关键帧已经和其他的关键帧形成了回环关系，因此在各种优化的过程中不应该被删除
    bool mbToBeErased;          ///<
    bool mbBad;                 ///< 

    float mHalfBaseline; ///< 对于双目相机来说,双目相机基线长度的一半. Only for visualization

    Map* mpMap;

    /// 在对位姿进行操作时相关的互斥锁
    std::mutex mMutexPose;
    /// 在操作当前关键帧和其他关键帧的公式关系的时候使用到的互斥锁
    std::mutex mMutexConnections;
    /// 在操作和特征点有关的变量的时候的互斥锁
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
