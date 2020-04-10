/**
 * @file Viewer.h
 * @author guoqing (1337841346@qq.com)
 * @brief 可视化查看器的声明
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


#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class Viewer
{
public:
    /**
     * @brief 构造函数
     * 
     * @param[in] pSystem           系统实例
     * @param[in] pFrameDrawer      帧绘制器
     * @param[in] pMapDrawer        地图绘制器
     * @param[in] pTracking         追踪线程
     * @param[in] strSettingPath    设置文件的路径
     */
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath);

    /**
     * @brief 进程的主函数, NOTICE 注意到这里提到了它是根据相机图像的更新帧率来绘制图像的
     * @detials Main thread function. Draw points, keyframes, the current camera pose and the last processed
     *  frame. Drawing is refreshed according to the camera fps. We use Pangolin.
     */
    void Run();

    /** @brief 请求停止当前进程 */
    void RequestFinish();
    
    /** @brief 请求当前可视化进程暂停更新图像数据 */
    void RequestStop();
    /** @brief 当前是否有停止当前进程的请求 */
    bool isFinished();
    /** @brief 判断当前进程是否已经停止 */
    bool isStopped();
    /** @brief 释放变量，避免互斥关系 */
    void Release();

private:

    /**
     * @brief 停止当前查看器的更新
     * 
     * @return true     成功停止
     * @return false    失败,一般是因为查看器进程已经销毁或者是正在销毁
     */
    bool Stop();

    ///系统对象指针
    System* mpSystem;
    ///帧绘制器
    FrameDrawer* mpFrameDrawer;
    ///地图绘制器
    MapDrawer* mpMapDrawer;
    ///追踪线程句柄
    Tracking* mpTracker;

    // 1/fps in ms
    ///每一帧图像持续的时间
    double mT;
    ///图像的尺寸
    float mImageWidth, mImageHeight;
    ///显示窗口的的查看视角,最后一个是相机的焦距
    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    /**
     * @brief 检查当前查看器进程是否已经终止
     * 
     * @return true 
     * @return false 
     */
    bool CheckFinish();
    /**
     * @brief 设置当前线程终止
     * 
     */
    void SetFinish();

    ///请求结束当前线程的标志
    bool mbFinishRequested;
    //当前线程是否已经终止
    bool mbFinished;
    ///线程锁对象,用于锁住和finsh,终止当前查看器进程相关的变量
    //? 但是我现在还是不明白,它是怎么知道我的这个线程锁对象和我的这个线程产生绑定关系的
    std::mutex mMutexFinish;

    ///当前进程是否停止
    bool mbStopped;
    ///是否头停止请求
    bool mbStopRequested;
    ///用于锁住stop,停止更新变量相关的互斥量
    std::mutex mMutexStop;

};

}


#endif // VIEWER_H
	

