/**
 * @file Viewer.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 查看器的实现
 * @version 2.0
 * @date 2020-06-12 update 
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

#include "Viewer.h"                 // Viewer 的定义
#include <pangolin/pangolin.h>      // pangolin 支持

#include <mutex>                    // C++ STL 线程锁支持

namespace ORB_SLAM2
{

//查看器的构造函数
Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
{
    //从文件中读取相机的帧频
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps < 1)
        fps=30;
    //计算出每一帧所持续的时间
    mT = 1e3 / fps;

    //从配置文件中获取图像的长宽参数
    mImageWidth  = fSettings["Camera.width" ];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth < 1 || mImageHeight < 1)
    {   
        //默认值
        mImageWidth = 640;
        mImageHeight = 480;
    }

    //读取三维视图中, 虚拟观察相机的视角
    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

// pangolin库的文档：http://docs.ros.org/fuerte/api/pangolin_wrapper/html/namespacepangolin.html, 不过感觉帮助不大. 这算是 Pangolin库的一个缺点吧
// 查看器的主循环由主线程(其实就是运行Tracker的线程)调用
void Viewer::Run()
{
    /* ===================================  NOTICE by Guoqing  ==================================
    *  注意提到的 "估计的相机位姿" 和 "虚拟观测相机位姿", 前者是 ORB-SLAM2 得到的结果, 后者是为了绘制三维场景
    *  设置的相机模型;
    *  我的理解是 Pangolin 是对 OpenGL 的封装, 本质上还是调用 OpenGL 工作, 而 OpenGL 基于状态机工作, 所以
    *  下面的代码出现的先后顺序十分重要;
    *  如果你对提到的计算机图形学名词术语赶到疑惑, 可以直接跳过去. 实际上对可视化部分的理解程度好坏并不影响你对
    *  ORB-SLAM2 算法本身的理解程度, 可视化部分只是个锦上添花的内容;
    *  如果你要在 ORB-SLAM2 基础上进行二次开发, 则可能需要了解一些可视化部分的执行逻辑, 有些东西特别是三维模型
    *  还是要画出来看的, 这对你调试程序会有一定帮助.
    *  ========================================================================================== */

    // 这个变量配合SetFinish函数用于指示该函数是否执行完毕
    mbFinished = false;

    // 创建窗口
    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",  // 窗口标题
                                  1024,                     // 窗口宽度
                                  768);                     // 窗口高度

    // 3D Mouse handler requires depth testing to be enabled (原始注释)
    // OpenGL命令, 启动深度测试; 启用后三维场景中的物体将按照在虚拟观察相机的深度进行遮挡\混合处理, 否则只
    // 会根据绘制的先后顺序进行遮挡\混合处理
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need (原始注释)
    // OpenGL 命令, 启用混合. 所谓混合就是半透明物体叠加的效果
    glEnable(GL_BLEND);
    // 选择混合选项, 就是你理解的半透明物体绘制方式: 底色 * (1 - 物体透明度) + 物体颜色 * 物体与透明度. 详情参考 OpenGL 文档
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 设置显示交互按钮的 panel, 名称/id 为 "menu"; 范围使用百分比表示: 最上; 最下; 最左, 左侧175个像素处
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    // 在 panel 上添加按钮/复选框: 是否视角跟随估计的相机位姿
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",  // <所在 panel id>.<显示的名称>
                                         true,                  // 初始是否选中
                                         true);                 // true = 复选框, false = 按钮
    // 是否显示地图点                                         
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    // 是否绘制关键帧
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    // 是否绘制本征图/本质图
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    // 是否工作于定位模式
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    // 是否复位整个 ORB-SLAM2 系统
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    // 定义虚拟观察相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
    // 其中 zNear zFar 表示只有在虚拟观察相机深度 zNear zFar 之间的点才会被绘制. 参考 OpenGL 的透视投影方式, pangolin 对此进行了进一步封装
    // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
    //                观测目标位置：(0, 0, 0)
    //                观测的方位向量：(0.0,-1.0, 0.0)
    
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    // 添加并设置视口(viewport).
    // 所谓视口, 就是将上一步创建的虚拟观测相机参数, 把空间中的物体投影之后得到的图像, 缩放到视口大小, 进行显示
    // 可以参考 OpenGL 的图形渲染管线
    pangolin::View& d_cam = pangolin::CreateDisplay()
            // 设置三维场景视口范围
            // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
            // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
            // 最后一个参数（-1024.0f/768.0f）为显示长宽比
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            // 关联到对应的虚拟观察相机
            .SetHandler(new pangolin::Handler3D(s_cam));

    // NOTE 除了上面两点, Pangolin 还会使用内置的着色器程序设置 OpenGZ 的渲染管线; 但是我没有仔细研读这部分代
    // 码, 而且根据 ORB-SLAM2 的绘制方式, 使用的还是"历史悠久"的 OpenGL 立即模式, 所以可能使用的是固定渲染管线
    // 有了解这方面的前辈欢迎提 issue 或 push request 完善这个部分的注释

    // 创建一个欧式变换矩阵, 保存当前绘制时, 估计的相机位姿
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    // 创建当前帧图像查看器, 其实就是事先声明一个 OpenCV 的图形窗口,备用
    cv::namedWindow("ORB-SLAM2: Current Frame");

    // 三维场景是否跟随相机视角
    bool bFollow = true;
    // 当前是否处于定位模式
    bool bLocalizationMode = false;

    // Viewer 线程主循环, 更新绘制的内容
    while(1)
    {
        // 清除缓冲区中的当前可写的颜色缓冲 和 深度缓冲. 两者搭配工作才能够使 OpenGL 绘制正确图像, 这里相当于清除了上一帧的绘制结果
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Step 1 得到最新的相机位姿
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        // Step 2 根据相机的位姿调整视角
        // menuFollowCamera 为按钮的状态, bFollow 为当前虚拟观测相机真实的状态
        if(menuFollowCamera && bFollow)
        {
            // 使虚拟相机跟随估计的相机位姿 
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            // 需要重新设置虚拟相机的投影矩阵
            s_cam.SetModelViewMatrix(
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));   
            // 然后虚拟相机跟随估计的相机的位姿
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            // 不跟踪
            bFollow = false;
        }

        // 更新定位模式或者是SLAM模式
        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        // Step 3 绘制三维视图部分, 基本的绘图工作交给 MapDrawer 完成
        // 激活控制对象
        d_cam.Activate(s_cam);
        // 三维场景区域背景设置为白色，glClearColor(red, green, blue, alpha），数值范围(0, 1)
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        // 绘制当前相机位置
        mpMapDrawer->DrawCurrentCamera(Twc);
        // 绘制关键帧和本征图(表达关键帧之间的共视关系)
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        // 绘制地图点
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        // Step 4 需要由 Pangolin 负责的内容完成, 剩下的内容交给 Pangolin 执行
        pangolin::FinishFrame();

        // Step 5 绘制当前帧图像和特征点提取匹配结果, 通过 OpenCV 来实现
        // 使用 FrameDrawer 直接绘制结果图像
        cv::Mat im = mpFrameDrawer->DrawFrame()
        // 显示;
        cv::imshow("ORB-SLAM2: Current Frame", im);
        // 必须要有这个, 一方面使得 OpenCV 有时间进行画面内容的更新, 另一方面可以控制整个绘制主循环的速率, 
        // 避免对计算机资源的无意义消耗
        cv::waitKey(mT);

        // Step 6 响应其他请求
        // 复位请求
        if(menuReset)
        {
            // 将所有的GUI控件恢复初始状态
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();

            // 当前 Viewer 相关变量也恢复到初始状态
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;

            // 通知系统复位
            mpSystem->Reset();
            // 按钮本身状态复位
            menuReset = false;
        }

        // 查看外部线程有无让 Viewer 线程停止更新的请求, 比如整个系统复位前, 请求 Viewer 暂停更新
        if(Stop())
        {
            // 检查何时不再让 Viewer 的绘制循环结束
            while(isStopped())
            {
				//usleep(3000);
				std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
        }

        // 查看外部线程是否有让 Viewer 线程终止的请求
        if(CheckFinish())
            break;
    }

    // 终止查看器,主要是设置状态,执行完成退出这个函数后,查看器进程就已经被销毁了
    SetFinish();
}

// 外部函数调用, 请求当前进程终止
void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

// 检查是否有终止当前进程的请求
bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

// 设置变量:当前进程已经终止
void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

// 判断当前进程是否已经终止
bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

// 外部线程调用, 发送停止绘图循环的请求
void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

// 查看当前查看器是否已经停止绘制循环
bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

// 查看是否有外部进程要求 Viewer 停止绘制
bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }
    return false;
}

// 和停止 Viewer 更新对应, 外部进程调用该函数, 使得 Viewer 可以继续进行更新
void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
