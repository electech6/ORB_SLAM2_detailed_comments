/**
 * @file Viewer.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 查看器的实现
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




#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

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
    if(fps<1)
        fps=30;
    //计算出每一帧所持续的时间
    mT = 1e3/fps;

    //从配置文件中获取图像的长宽参数
    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {   
        //默认值
        mImageWidth = 640;
        mImageHeight = 480;
    }

    //读取视角
    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

// pangolin库的文档：http://docs.ros.org/fuerte/api/pangolin_wrapper/html/namespacepangolin.html
//查看器的主进程看来是外部函数所调用的
void Viewer::Run()
{
    //这个变量配合SetFinish函数用于指示该函数是否执行完毕
    mbFinished = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    // 启动深度测试，OpenGL只绘制最前面的一层，绘制时检查当前像素前面是否有别的像素，如果别的像素挡住了它，那它就不会绘制
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    // 在OpenGL中使用颜色混合
    glEnable(GL_BLEND);
    // 选择混合选项
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 新建按钮和选择框，第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    // 定义相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
    // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
    //                观测目标位置：(0, 0, 0)
    //                观测的方位向量：(0.0,-1.0, 0.0)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    // 定义显示面板大小，orbslam中有左右两个面板，昨天显示一些按钮，右边显示图形
    // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
    // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
    // 最后一个参数（-1024.0f/768.0f）为显示长宽比
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    //创建一个欧式变换矩阵,存储当前的相机位姿
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    //创建当前帧图像查看器,谢晓佳在泡泡机器人的第35课中讲过这个;需要先声明窗口,再创建;否则就容易出现窗口无法刷新的情况
    cv::namedWindow("ORB-SLAM2: Current Frame");

    //ui设置
    bool bFollow = true;
    bool bLocalizationMode = false;

    //更新绘制的内容
    while(1)
    {
        // 清除缓冲区中的当前可写的颜色缓冲 和 深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // step1：得到最新的相机位姿
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        // step2：根据相机的位姿调整视角
        // menuFollowCamera为按钮的状态，bFollow为真实的状态
        if(menuFollowCamera && bFollow)
        {
            //当之前也在跟踪相机时
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            //当之前没有在跟踪相机时
            s_cam.SetModelViewMatrix(
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));   //? 不知道这个视角设置的具体作用和
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            //之前跟踪相机,但是现在菜单命令不要跟踪相机时
            bFollow = false;
        }

        //更新定位模式或者是SLAM模式
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

        d_cam.Activate(s_cam);
        // step 3：绘制地图和图像(3D部分)
        // 设置为白色，glClearColor(red, green, blue, alpha），数值范围(0, 1)
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        //绘制当前相机
        mpMapDrawer->DrawCurrentCamera(Twc);
        //绘制关键帧和共视图
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        //绘制地图点
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        pangolin::FinishFrame();

        // step 4:绘制当前帧图像和特征点提取匹配结果
        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::imshow("ORB-SLAM2: Current Frame",im);
        //NOTICE 注意对于我所遇到的问题,ORB-SLAM2是这样子来处理的
        cv::waitKey(mT);

        // step 5 相应其他请求
        //复位按钮
        if(menuReset)
        {
            //将所有的GUI控件恢复初始状态
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            //相关变量也恢复到初始状态
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            //告知系统复位
            mpSystem->Reset();
            //按钮本身状态复位
            menuReset = false;
        }

        //如果有停止更新的请求
        if(Stop())
        {
            //就不再绘图了,并且在这里每隔三秒检查一下是否结束
            while(isStopped())
            {
				//usleep(3000);
				std::this_thread::sleep_for(std::chrono::milliseconds(3));

            }
        }

        //满足的时候退出这个线程循环,这里应该是查看终止请求
        if(CheckFinish())
            break;
    }

    //终止查看器,主要是设置状态,执行完成退出这个函数后,查看器进程就已经被销毁了
    SetFinish();
}

//外部函数调用,用来请求当前进程结束
void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

//检查是否有结束当前进程的请求
bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

//设置变量:当前进程已经结束
void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

//判断当前进程是否已经结束
bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

//请求当前查看器停止更新
void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

//查看当前查看器是否已经停止更新
bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

//当前查看器停止更新
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

//释放查看器进程,因为如果停止查看器的话,查看器进程会处于死循环状态.这个就是为了释放那个标志
void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
