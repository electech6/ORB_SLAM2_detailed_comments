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

/*
这个头文件定义了ORB-SLAM2的主线程（或者称之为系统）结构，其他的各个模块都是由这里开始被调用的。
*/


#ifndef SYSTEM_H
#define SYSTEM_H

//一些公用库的支持，字符串操作，多线程操作，以及opencv库等
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

//下面则是本ORB-SLAM2系统中的其他模块
#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

namespace ORB_SLAM2
{

//要用到的其他类的前视声明
class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

//本类的定义
class System
{
public:
    // Input sensor
    //这个枚举类型用于 表示本系统所使用的传感器类型
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    //构造函数，用来初始化整个系统。
    System(const string &strVocFile,            //指定ORB字典文件的路径
           const string &strSettingsFile,       //指定配置文件的路径
           const eSensor sensor,                //指定所使用的传感器类型
           const bool bUseViewer = true);       //指定是否使用可视化界面 TODO 

    //下面是针对三种不同类型的传感器所设计的三种运动追踪接口。彩色图像为CV_8UC3类型，并且都将会被转换成为灰度图像。
    //追踪接口返回估计的相机位姿，如果追踪失败则返回NULL

    // Proccess the given stereo frame. Images must be synchronized and rectified（校正）.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    // NOTE 注意这里英文注释的说法，双目图像有同步和校准的概念。
    cv::Mat TrackStereo(const cv::Mat &imLeft,          //左目图像
                        const cv::Mat &imRight,         //右目图像
                        const double &timestamp);       //时间戳

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    // NOTE 而在这里对RGBD图像的说法则是“配准”
    cv::Mat TrackRGBD(const cv::Mat &im,                //彩色图像
                      const cv::Mat &depthmap,          //深度图像
                      const double &timestamp);         //时间戳

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im,           //图像
                           const double &timestamp);    //时间戳

    // This stops local mapping thread (map building) and performs only camera tracking.
    //使能定位模式，此时仅有运动追踪部分在工作，局部建图功能则不工作
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    //反之同上
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    // 获取从上次调用本函数后是否发生了比较大的地图变化
    bool MapChanged();

    // Reset the system (clear map)
    // 复位 系统
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    //关闭系统，这将会关闭所有线程并且丢失曾经的各种数据
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // NOTE Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    // 以TUM格式保存相机的运动轨迹，这个函数将会在Shutdown函数中被首先调用
    void SaveTrajectoryTUM(const string &filename);         //指定文件名

    // Save keyframe poses in the TUM RGB-D dataset format.
    // NOTE This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    // 以TUM格式保存关键帧位姿。  TODO 是不是这也意味着，我可以使用g2o_viewer这样的软件去查看并且进行优化实验？
    void SaveKeyFrameTrajectoryTUM(const string &filename);     //指定文件名

    // Save camera trajectory in the KITTI dataset format.
    // NOTE Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    // 以KITTI格式保存相机的运行轨迹
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // 在这里可以实现自己的地图保存和加载函数
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    //获取最近的运动追踪状态、地图点追踪状态、特征点追踪状态（）
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

private:

    //注意变量命名方式，类的变量有前缀m，如果这个变量是指针类型还要多加个前缀p，
    //如果是进程那么加个前缀t

    // Input sensor
    // 传感器类型
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    // 一个指针指向ORB字典
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    // 关键帧数据库的指针，这个数据库用于重定位和回环检测
    KeyFrameDatabase* mpKeyFrameDatabase;

    //指向地图（数据库）的指针
    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    // 追踪器，除了进行运动追踪外还要负责创建关键帧、创建新地图点和进行重定位的工作。详细信息还得看相关文件
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    //局部建图器。局部BA由它进行。
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    // 回环检测器，它会执行位姿图优化并且开一个新的线程进行全局BA
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    // 查看器，可视化 界面
    Viewer* mpViewer;

    //帧绘制器
    FrameDrawer* mpFrameDrawer;
    //地图绘制器
    MapDrawer* mpMapDrawer;


    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    //系统除了在主进程中进行运动追踪工作外，会创建局部建图线程、回环检测线程和查看器线程。
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    //复位标志，注意这里目前还不清楚为什么要定义为std::mutex类型 TODO 
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    //模式改变标志
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    // 追踪状态标志，注意前三个的类型和上面的函数类型相互对应
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
