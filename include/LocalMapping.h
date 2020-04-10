/**
 * @file LocalMapping.h
 * @author guoqing (1337841346@qq.com)
 * @brief 局部建图线程
 * @version 0.1
 * @date 2019-04-29
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



#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

/** @brief 局部建图线程类 */
class LocalMapping
{
public:

    /**
     * @brief 构造函数
     * @param[in] pMap          局部地图的句柄？ //?
     * @param[in] bMonocular    当前系统是否是单目输入
     */
    LocalMapping(Map* pMap, const float bMonocular);

    /**
     * @brief 设置回环检测线程句柄
     * @param[in] pLoopCloser 回环检测线程句柄
     */
    void SetLoopCloser(LoopClosing* pLoopCloser);

    /**
     * @brief 设置追踪线程句柄
     * @param[in] pTracker 追踪线程句柄
     */
    void SetTracker(Tracking* pTracker);

    // Main function
    /** @brief 线程主函数 */
    void Run();

    /**
     * @brief 插入关键帧,由外部线程调用
     * @details 将关键帧插入到地图中，以便将来进行局部地图优化 \n
     * NOTICE 这里仅仅是将关键帧插入到列表中进行等待
     * @param pKF KeyFrame
     */
    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    /** @brief 外部线程调用,请求停止当前线程的工作 */
    void RequestStop();
    /** @brief 请求当前线程复位,由外部线程调用,堵塞的 */
    void RequestReset();
    /**
     * @brief 检查是否要把当前的局部建图线程停止,如果当前线程没有那么检查请求标志,如果请求标志被置位那么就设置为停止工作.由run函数调用
     * @return true 
     * @return false 
     */
    bool Stop();
    /** @brief 释放当前还在缓冲区中的关键帧指针  */
    void Release();
    /** @brief 检查mbStopped是否被置位了 */
    bool isStopped();
    /** @brief 是否有终止当前线程的请求 */
    bool stopRequested();
    /** @brief 查看当前是否允许接受关键帧 */
    bool AcceptKeyFrames();
    /**
     * @brief 设置"允许接受关键帧"的状态标志
     * @param[in] flag 是或者否
     */
    void SetAcceptKeyFrames(bool flag);
    /** @brief 设置 mbnotStop标志的状态 */
    bool SetNotStop(bool flag);

    /** @brief 外部线程调用,终止BA */
    void InterruptBA();

    /** @brief 请求终止当前线程 */
    void RequestFinish();
    /** @brief 当前线程的run函数是否已经终止 */
    bool isFinished();
    //查看队列中等待插入的关键帧数目
    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    /**
     * @brief 查看列表中是否有等待被插入的关键帧
     * @return 如果存在，返回true
     */
    bool CheckNewKeyFrames();
    /**
     * @brief 处理列表中的关键帧
     * 
     * - 计算Bow，加速三角化新的MapPoints
     * - 关联当前关键帧至MapPoints，并更新MapPoints的平均观测方向和观测距离范围
     * - 插入关键帧，更新Covisibility图和Essential图
     * @see VI-A keyframe insertion
     */
    void ProcessNewKeyFrame();
    /** @brief 相机运动过程中和共视程度比较高的关键帧通过三角化恢复出一些MapPoints */
    void CreateNewMapPoints();

    /**
     * @brief 剔除ProcessNewKeyFrame和CreateNewMapPoints函数中引入的质量不好的MapPoints
     * @see VI-B recent map points culling
     */
    void MapPointCulling();
    /** @brief 检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints */
    void SearchInNeighbors();

    /**
     * @brief 关键帧剔除
     * @detials 在Covisibility Graph中的关键帧，其90%以上的MapPoints能被其他关键帧（至少3个）观测到，则认为该关键帧为冗余关键帧。
     * @see VI-E Local Keyframe Culling
     */
    void KeyFrameCulling();

    /**
     * 根据两关键帧的姿态计算两个关键帧之间的基本矩阵
     * @param  pKF1 关键帧1
     * @param  pKF2 关键帧2
     * @return      基本矩阵
     */
    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);
    /**
     * @brief 计算三维向量v的反对称矩阵
     * @param[in] v     三维向量
     * @return cv::Mat  反对称矩阵
     */
    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    /// 当前系统输入数单目还是双目RGB-D的标志
    bool mbMonocular;

    /** @brief 检查当前是否有复位线程的请求 */
    void ResetIfRequested();
    /// 当前系统是否收到了请求复位的信号
    bool mbResetRequested;
    /// 和复位信号有关的互斥量
    std::mutex mMutexReset;

    /** @brief 检查是否已经有外部线程请求终止当前线程 */
    bool CheckFinish();
    /** @brief 设置当前线程已经真正地结束了,由本线程run函数调用 */
    void SetFinish();
    /// 当前线程是否收到了请求终止的信号
    bool mbFinishRequested;
    /// 当前线程的主函数是否已经终止
    bool mbFinished;
    // 和"线程真正结束"有关的互斥锁
    std::mutex mMutexFinish;

    // 指向局部地图的句柄
    Map* mpMap;

    // 回环检测线程句柄
    LoopClosing* mpLoopCloser;
    // 追踪线程句柄
    Tracking* mpTracker;

    // Tracking线程向LocalMapping中插入关键帧是先插入到该队列中
    std::list<KeyFrame*> mlNewKeyFrames; ///< 等待处理的关键帧列表
    /// 当前正在处理的关键帧
    KeyFrame* mpCurrentKeyFrame;

    /// 存储当前关键帧生成的地图点,也是等待检查的地图点列表
    std::list<MapPoint*> mlpRecentAddedMapPoints;

    /// 操作关键帧列表时使用的互斥量 
    std::mutex mMutexNewKFs;

    /// 终止BA的标志
    bool mbAbortBA;

    /// 当前线程是否已经真正地终止了
    bool mbStopped;
    /// 终止当前线程的请求
    bool mbStopRequested;
    /// 标志这当前线程还不能够停止工作,优先级比那个"mbStopRequested"要高.只有这个和mbStopRequested都满足要求的时候,线程才会进行一系列的终止操作
    bool mbNotStop;
    /// 和终止线程相关的互斥锁
    std::mutex mMutexStop;

    /// 当前局部建图线程是否允许关键帧输入
    bool mbAcceptKeyFrames;
    /// 和操作上面这个变量有关的互斥量
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H

