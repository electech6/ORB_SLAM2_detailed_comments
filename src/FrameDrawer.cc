/**
 * @file FrameDrawer.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 帧绘制器
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

// 类型支持
#include "FrameDrawer.h"
#include "Tracking.h"

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// 线程锁支持
#include<mutex>

namespace ORB_SLAM2
{
//构造函数
FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    // 一开始的时候, SLAM系统还没有准备好呢
    mState = Tracking::SYSTEM_NOT_READY;
    // 画布, 用于可视化展示显示当前帧的图像和特征点跟踪等结果
    // 这里虽然设置大小为 640x480, 但实际上在 Tracking 调用 FrameDrawer::Update() 时会自动更新画布的大小和通道个数
    // 算是 ORB-SLAM 写得不是很好的地方吧
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

// 准备需要显示的信息，包括图像、特征点、地图、跟踪状态等
cv::Mat FrameDrawer::DrawFrame()
{
    // Step 1 创建关键变量的副本, 主要是规避多线程操作的影响
    cv::Mat                 im;                 // 缓存的当前帧图像
    vector<cv::KeyPoint>    vIniKeys;           // Initialization: KeyPoints in reference frame
    vector<int>             vMatches;           // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint>    vCurrentKeys;       // KeyPoints in current frame
    vector<bool>            vbVO, vbMap;        // Tracked MapPoints in current frame
    int                     state;              // Tracking state

    // Copy variables within scoped mutex
    // 将成员变量赋值给局部变量（包括图像、状态、其它的提示）
    // NOTICE 加互斥锁，避免与FrameDrawer::Update函数中图像拷贝发生冲突
    {
        unique_lock<mutex> lock(mMutex);
        state = mState;
        if(mState == Tracking::SYSTEM_NOT_READY)
            mState = Tracking::NO_IMAGES_YET;

        // 这里使用copyTo进行深拷贝是因为后面会把单通道灰度图像转为3通道图像
        mIm.copyTo(im);

        // 如果缓存的 Tracking 线程数据没有完成初始化
        if(mState == Tracking::NOT_INITIALIZED)
        {
            // 获取当前帧和参考帧的特征点, 并且得到他们的匹配关系
            vCurrentKeys = mvCurrentKeys;
            vIniKeys     = mvIniKeys;
            vMatches     = mvIniMatches;
        }
        // 如果缓存时 Tracking 线程在正常追踪
        else if(mState == Tracking::OK)
        {
            // 获取当前帧特征点数据和类别(Map or VO)标签吧
            vCurrentKeys = mvCurrentKeys;
            vbVO         = mvbVO;
            vbMap        = mvbMap;
        }
        // 如果缓存数据时 Tracking 线程就已经跟丢了
        else if(mState==Tracking::LOST)
        {
            // 跟丢的时候就之获得当前帧的特征点就可以了
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    // 图像通道调整, 以为特征点等数据需要以彩色的形式绘制
    if(im.channels() < 3) //this should be always true
        cvtColor(im, im, CV_GRAY2BGR);

    // Draw
    // Step 2：绘制特征点
    // 初始化时，当前帧的特征坐标与初始帧的特征点坐标连成线，形成轨迹
    if(state == Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i = 0; i < vMatches.size(); i++)
        {
            // 绘制参考帧特征点到当前帧帧特征点的连线来表达匹配关系, 这就是当初看到的初始化过程中图像中显示的绿线
            if(vMatches[i] >= 0)
            {
                // 调用 OpenCV 函数绘制线
                cv::line(im,                            // 画布
                         vIniKeys[i].pt,                // 起点
                         vCurrentKeys[vMatches[i]].pt,  // 终点
                         cv::Scalar(0,255,0));          // 颜色
            }
        }
    }
    // 如果 Tracking 是正常运行
    else if(state==Tracking::OK) //TRACKING
    {
        // 当前帧追踪到的特征点计数,
        // map 类点
        mnTracked   = 0;
        // vo 类点
        mnTrackedVO = 0;

        // Draw keypoints
        // 方框半径
        const float r = 5;
        const int   n = vCurrentKeys.size();
        for(int i = 0; i < n; i++)
        {
            // 如果这个点在视觉里程计中有(应该是追踪成功了的意思吧),在局部地图中也有
            if(vbVO[i] || vbMap[i])
            {
                // 在特征点附近选择两个点作为正方形的角点
                cv::Point2f pt1,pt2;
                pt1.x = vCurrentKeys[i].pt.x - r;
                pt1.y = vCurrentKeys[i].pt.y - r;
                pt2.x = vCurrentKeys[i].pt.x + r;
                pt2.y = vCurrentKeys[i].pt.y + r;

                // This is a match to a MapPoint in the map
                // 如果这个点正确跟踪,存在并会更新对应的地图点, 在画布 im 中用绿色标注
                if(vbMap[i])
                {
                    // 通道顺序为bgr，地图中MapPoints用绿色圆点表示，并用绿色小方框圈住
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                // 如果这个点正确跟踪但是却不会存在对应的地图点, 那么说明是在纯定位模式中跟踪成功的, 使用蓝色表示
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }//遍历所有的特征点
    }

    // Step 3 然后写入状态栏的信息
    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    //返回生成的图像
    return imWithInfo;
}


//绘制状态栏上的文本信息
void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    // Step 1 生成状态描述字符串
    stringstream s;
    // 当前 Tracking 状态
    if(nState == Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState == Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState == Tracking::OK)
    {
        // 当前工作模式
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";

        // 跟踪状态
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        //在视觉里程计中匹配到的
        if(mnTrackedVO > 0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState == Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState == Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    // Step 2 图像准备
    // 回想小时候学写英文字母用的四线格, 下面的 cv::getTextSize() 函数返回的高度只是第一条线到第三条线的距离, baseline 表示第三条线到第四条线的距离
    int baseline = 0;
    // 计算字符串文字所占用的图像区域的大小
    cv::Size textSize = cv::getTextSize(s.str(),                    // 字符串
                                        cv::FONT_HERSHEY_PLAIN,     // 字体
                                        1,                          // 字体缩放
                                        1,                          // 粗细
                                        &baseline);                 // 基线, 相对于最低端的文本点的,y坐标  
                                                                    
    // 扩展之前的图像, 就是高度加10像素
    imText = cv::Mat(im.rows + textSize.height+10, im.cols, im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    // 扩充区域填充黑色背景
    imText.rowRange(im.rows, imText.rows) = cv::Mat::zeros(textSize.height + 10, im.cols, im.type());

    // Step 3 绘制文字
    cv::putText(imText,                         //目标图像
                s.str(),                        //要输出的文字
                cv::Point(5,imText.rows-5),     //输出文字的起始位置
                cv::FONT_HERSHEY_PLAIN,         //字体
                1,                              //缩放
                cv::Scalar(255,255,255),        //颜色,白色
                1,                              //线宽
                8);                             //线型
}

// Tracking 线程调用, 更新 FrameDrawer 中缓存的相关数据, 用于绘制当前帧的结果
void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    // 更新当前 Tracking 线程处理的图像(灰度图)
    pTracker->mImGray.copyTo(mIm);
    // 更新 Tracking 线程跟踪到的特征点
    mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
    // 更新 Tracking 线程跟踪到的特征点个数
    N = mvCurrentKeys.size();

    // 先认为这些追踪到的特征点都不是 VO 和 Map类型
    // 什么是VO类型, 什么是 Map 类型? 看下面的程序
    mvbVO  = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    // mbOnlyTracking等于false表示有地图更新的正常SLAM过程,等于true表示用户手动选择定位模式, 无地图更新的VO过程
    mbOnlyTracking = pTracker->mbOnlyTracking;

    // 下面要看 Tracking 线程的状态
    if(pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED)
    {
        // 如果 Tracking 线程还没有完成初始化操作, 那么更新初始特征点和匹配信息
        mvIniKeys    = pTracker->mInitialFrame.mvKeys;
        mvIniMatches = pTracker->mvIniMatches;
    }
    // 如果 Tracking 现在是正常跟踪状态
    else if(pTracker->mLastProcessedState == Tracking::OK)
    {
        // 遍历当前跟踪到的每个地图点
        for(int i = 0; i < N; i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            // 如果跟踪到了地图点
            // 为什么会有这个判断? 因为下标 i 是当前帧特征点序列的下标, 但并不是当前帧的每一个特征点都能够跟踪成功
            if(pMP)
            {
                // 如果在后面的位姿估计过程中这个也不是外点 (严谨来讲应该说"这对匹配点不是outlier")
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    // 该地图点被多帧观测到，为地图点
                    if(pMP->Observations() > 0)
                        mvbMap[i]=true;
                    else
                        //否则表示这个特征点是在当前帧中第一次提取得到的点. 这个情况只会发生在纯定位模式中, SLAM模式中不会出现这个
                        mvbVO[i]=true;
                }
            }
        }
    }

    //更新追踪线程的跟踪状态
    mState = static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
