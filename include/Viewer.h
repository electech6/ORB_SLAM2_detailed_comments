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

#include "FrameDrawer.h"        // 帧绘制器, 利用 OpenCV 的 GUI 实现, 其实就是显示图像
#include "MapDrawer.h"          // 地图绘制器, 利用 Pangolin 和 OpenGL 的 API 实现
#include "Tracking.h"           // 追踪线程
#include "System.h"             // 系统整体定义

#include <mutex>                // C++ STL 线程锁支持

namespace ORB_SLAM2
{

// 前视声明
class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

/** @brief 用于创建窗口, 进行用户交互和显示内容的管理 */
class Viewer
{
public:
    /**
     * @brief 构造函数
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
    /** @brief 表示当前 Viewer 已经停止 */
    // TODO
    void Release();

private:

    /**
     * @brief 停止当前查看器的更新
     * @return true     成功停止
     * @return false    失败,一般是因为查看器进程已经销毁或者是正在销毁
     */
    bool Stop();

    System*         mpSystem;           ///< 系统管理对象句柄
    FrameDrawer*    mpFrameDrawer;      ///< 帧绘制器句柄
    MapDrawer*      mpMapDrawer;        ///< 地图绘制器句柄
    Tracking*       mpTracker;          ///< 追踪线程句柄

    // 1/fps in ms
    double          mT;                 /// ? 每一帧图像持续的时间
    float           mImageWidth;        ///< 显示图像的宽度
    float           mImageHeight;       ///< 显示图像的高度

    float           mViewpointX;        ///< 三维视图中相机的查看点坐标X
    float           mViewpointY;        ///< 三维视图中相机的查看点坐标Y
    float           mViewpointZ;        ///< 三维视图中相机的查看点坐标Z
    float           mViewpointF;        ///< 三维视图中相机的焦距

    /**
     * @brief 检查当前查看器进程是否已经终止
     * @return true 
     * @return false 
     */
    bool CheckFinish();

    /** @brief 设置当前线程终止 */
    void SetFinish();

    bool mbFinishRequested;             ///< 请求结束当前线程的标志
    bool mbFinished;                    ///< 当前线程是否已经终止
    std::mutex mMutexFinish;            ///< 线程锁对象,用于锁住和finsh,终止当前查看器进程相关的变量
    
    bool mbStopped;                     ///< 当前进程是否停止绘图循环(注意不是退出)
    bool mbStopRequested;               ///< 是否请求终止绘制循环的变量, 由外部循环控制
    std::mutex mMutexStop;              ///< 控制上述变量的线程锁
};

}


#endif // VIEWER_H
	

