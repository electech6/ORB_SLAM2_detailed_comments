/**
 * @file FrameDrawer.h
 * @author guoqing (1337841346@qq.com)
 * @brief 帧绘制器的定义
 * @version 0.1
 * @date 2019-02-19
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


#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

// 类型支持
#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

// 线程锁
#include <mutex>

namespace ORB_SLAM2
{

// 前视声明
class Tracking;
class Viewer;

class FrameDrawer
{
public:
    /**
     * @brief 构造函数
     * @param[in] pMap  地图对象句柄
     */
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.
    /**
     * @brief 将跟踪线程的数据拷贝到绘图线程（图像、特征点、地图、跟踪状态）, 由其他线程调用
     * @param[in] pTracker 追踪线程
     */
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    /**
     * @brief 绘制最近处理过的帧,这个将会在可视化查看器的窗口中被创建
     * @return cv::Mat 返回绘制完成的图像,可以直接进行显示
     */
    cv::Mat DrawFrame();

protected:

    /**
     * @brief 绘制底部的信息栏
     * @param[in]  im           原始图像
     * @param[in]  nState       当前系统的工作状态(初始化? 跟踪正常? 跟踪丢失?)
     * @param[out] imText       叠加后的图像
     */
    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn -- 说白了就是缓存的等待绘制的各种数据
    cv::Mat                     mIm;            ///< 当前输入的图像(灰度化之后)
    int                         N;              ///< 当前帧中特征点的数目
    vector<cv::KeyPoint>        mvCurrentKeys;  ///< 当前帧中的特征点
    vector<bool>                mvbMap;         ///< 当前帧中成功跟踪的特征点存在对应的地图点的点, 在SLAM过程中所有的点都满足
    vector<bool>                mvbVO;          ///< 当前帧中成功跟踪的特征点没有对应的地图点, 说明是在纯定位模式下新匹配成功的特征点
    bool                        mbOnlyTracking; ///< 当前处于定位模式还是处于SLAM模式
    
    int                         mnTracked;      ///< Map 类点追踪的个数
    int                         mnTrackedVO;    ///< VO  类点追踪的个数
    vector<cv::KeyPoint>        mvIniKeys;      ///< 初始化过程中参考帧提取得到的特征点, 仅用于初始化过程
    vector<int>                 mvIniMatches;   ///< 初始化过程中当前帧和参考帧特征点的匹配关系, 仅用于初始化过程
    int                         mState;         ///< 当前SLAM系统的工作状态

    Map*                        mpMap;          ///< 地图对象句柄

    std::mutex                  mMutex;         ///< 线程锁, 主要解决 Tracking 线程和 Viewer 线程同时访问的问题
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
