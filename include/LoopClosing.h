/**
 * @file LoopClosing.h
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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
//? 目前并不知道是用来做什么的
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;

/// 回环检测线程
class LoopClosing
{
public:
    /// 自定义数据类型, ConsistentGroup.first对应每个“连续组”中的关键帧，ConsistentGroup.second为每个“连续组”的序号
    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    /// 存储关键帧对象和位姿的键值对,这里是map的完整构造函数
    typedef map<KeyFrame*,                  //键
                g2o::Sim3,                  //值
                std::less<KeyFrame*>,       //排序算法
                Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > // 指定分配器,和内存空间开辟有关. 为了能够使用Eigen库中的SSE和AVX指令集加速,需要将传统STL容器中的数据进行对齐处理
                > KeyFrameAndPose;

public:

    /**
     * @brief 构造函数
     * @param[in] pMap          地图指针
     * @param[in] pDB           词袋数据库
     * @param[in] pVoc          词典
     * @param[in] bFixScale     表示sim3中的尺度是否要计算,对于双目和RGBD情况尺度是固定的,s=1,bFixScale=true;而单目下尺度是不确定的,此时bFixScale=false,sim
     * 3中的s需要被计算
     */
    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);
    /** @brief 设置追踪线程的句柄
     *  @param[in] pTracker 追踪线程的句柄  */
    void SetTracker(Tracking* pTracker);
    /** @brief 设置局部建图线程的句柄
     * @param[in] pLocalMapper   */
    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    /** @brief 回环检测线程主函数 */
    void Run();

    /** @brief 将某个关键帧加入到回环检测的过程中,由局部建图线程调用
     *  @param[in] pKF   */
    void InsertKeyFrame(KeyFrame *pKF);

    /** @brief 由外部线程调用,请求复位当前线程.在回环检测复位完成之前,该函数将一直保持堵塞状态 */
    void RequestReset();

    // This function will run in a separate thread
    /**
     * @brief 全局BA线程,这个函数是这个线程的主函数
     * @param[in] nLoopKF 看名字是闭环关键帧,但是实际上给的是当前关键帧的ID
     */
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);

    // 在回环纠正的时候调用,查看当前是否已经有一个全局优化的线程在进行
    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    /** @brief 由外部线程调用,请求终止当前线程 */
    void RequestFinish();

    /** @brief 由外部线程调用,判断当前回环检测线程是否已经正确终止了  */
    bool isFinished();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    /** @brief 查看列表中是否有等待被插入的关键帧
     *  @return true 如果有
     *  @return false 没有  */
    bool CheckNewKeyFrames();

    /** @brief 检测回环,如果有的话就返回真 */
    bool DetectLoop();

    /**
     * @brief 计算当前帧与闭环帧的Sim3变换等
     * @details \n
     * 1. 通过Bow加速描述子的匹配，利用RANSAC粗略地计算出当前帧与闭环帧的Sim3（当前帧---闭环帧）          \n
     * 2. 根据估计的Sim3，对3D点进行投影找到更多匹配，通过优化的方法计算更精确的Sim3（当前帧---闭环帧）     \n
     * 3. 将闭环帧以及闭环帧相连的关键帧的MapPoints与当前帧的点进行匹配（当前帧---闭环帧+相连关键帧）      \n
     * \n
     * 注意以上匹配的结果均都存在成员变量mvpCurrentMatchedPoints中，实际的更新步骤见CorrectLoop()步骤3：Start Loop Fusion \n
     * 对于双目或者是RGBD输入的情况,计算得到的尺度=1
     */
    bool ComputeSim3();

    /**
     * @brief 通过将闭环时相连关键帧的MapPoints投影到这些关键帧中，进行MapPoints检查与替换
     * @param[in] CorrectedPosesMap 关联的当前帧组中的关键帧和相应的纠正后的位姿
     */
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    /**
     * @brief 闭环纠正
     * @detials \n
     * 1. 通过求解的Sim3以及相对姿态关系，调整与当前帧相连的关键帧位姿以及这些关键帧观测到的MapPoints的位置（相连关键帧---当前帧） \n
     * 2. 将闭环帧以及闭环帧相连的关键帧的MapPoints和与当前帧相连的关键帧的点进行匹配（相连关键帧+当前帧---闭环帧+相连关键帧）     \n
     * 3. 通过MapPoints的匹配关系更新这些帧之间的连接关系，即更新covisibility graph                                      \n
     * 4. 对Essential Graph（Pose Graph）进行优化，MapPoints的位置则根据优化后的位姿做相对应的调整                         \n
     * 5. 创建线程进行全局Bundle Adjustment
     */
    void CorrectLoop();
    /** @brief  当前线程调用,检查是否有外部线程请求复位当前线程,如果有的话就复位回环检测线程 */
    void ResetIfRequested();
    /// 是否有复位当前线程的请求
    bool mbResetRequested;
    /// 和复位当前线程相关的互斥量
    std::mutex mMutexReset;

    /** @brief 当前线程调用,查看是否有外部线程请求当前线程  */
    bool CheckFinish();
    /** @brief 有当前线程调用,执行完成该函数之后线程主函数退出,线程销毁 */
    void SetFinish();
    /// 是否有终止当前线程的请求
    bool mbFinishRequested;
    /// 当前线程是否已经停止工作
    bool mbFinished;
    /// 和当前线程终止状态操作有关的互斥量
    std::mutex mMutexFinish;

    /// (全局)地图的指针
    Map* mpMap;
    /// 追踪线程句柄
    Tracking* mpTracker;

    /// 关键帧数据库
    KeyFrameDatabase* mpKeyFrameDB;
    /// 词袋模型中的大字典
    ORBVocabulary* mpORBVocabulary;
    /// 局部建图线程句柄
    LocalMapping *mpLocalMapper;

    /// 一个队列, 其中存储了参与到回环检测的关键帧 (当然这些关键帧也有可能因为各种原因被设置成为bad,这样虽然这个关键帧还是存储在这里但是实际上已经不再实质性地参与到回环检测的过程中去了)
    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    /// 操作参与到回环检测队列中的关键帧时,使用的互斥量
    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    /// 连续性阈值,构造函数中将其设置成为了3
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    /// 当前关键帧,其实称之为"当前正在处理的关键帧"更加合适
    KeyFrame* mpCurrentKF;
    // 最终检测出来的,和当前关键帧形成闭环的闭环关键帧
    KeyFrame* mpMatchedKF;
    /// 上一次执行的时候产生的连续组s
    std::vector<ConsistentGroup> mvConsistentGroups;
    /// 从上面的关键帧中进行筛选之后得到的具有足够的"连续性"的关键帧 -- 这个其实也是相当于更高层级的、更加优质的闭环候选帧
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    /// 和当前关键帧相连的关键帧形成的"当前关键帧组"
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    /// 下面的变量中存储的地图点在"当前关键帧"中成功地找到了匹配点的地图点的集合
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    /// 闭环关键帧上的所有相连关键帧的地图点
    std::vector<MapPoint*> mvpLoopMapPoints;
    // 下面的变量的cv::Mat格式版本
    cv::Mat mScw;
    // 当得到了当前关键帧的闭环关键帧以后,计算出来的从世界坐标系到当前帧的sim3变换
    g2o::Sim3 mg2oScw;

    /// 上一次闭环帧的id
    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    /// 全局BA线程是否在进行
    bool mbRunningGBA;
    /// 全局BA线程在收到停止请求之后是否停止的比标志 //? 可是直接使用上面变量的逆不就可以表示了吗? //? 表示全局BA工作是否正常结束?
    bool mbFinishedGBA;
    /// 由当前线程调用,请求停止当前正在进行的全局BA
    bool mbStopGBA;
    /// 在对和全局线程标志量有关的操作的时候使用的互斥量
    std::mutex mMutexGBA;
    /// 全局BA线程句柄
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    /// 如果是在双目或者是RGBD输入的情况下,就要固定尺度,这个变量就是是否要固定尺度的标志
    bool mbFixScale;

    /// 已经进行了的全局BA次数(包含中途被打断的)
    bool mnFullBAIdx;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
