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


#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{
//构造函数
FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    // 初始化图像显示画布
    // 包括：图像、特征点连线形成的轨迹（初始化时）、框（跟踪时的MapPoint）、圈（跟踪时的特征点）
    // ！！！固定画布大小为640*480
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

// 准备需要显示的信息，包括图像、特征点、地图、跟踪状态
cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    // step 1：将成员变量赋值给局部变量（包括图像、状态、其它的提示）
    //NOTICE 加互斥锁，避免与FrameDrawer::Update函数中图像拷贝发生冲突
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        //NOTICE 这里使用copyTo进行深拷贝是因为后面会把单通道灰度图像转为3通道图像
        mIm.copyTo(im);

        //没有初始化的时候
        if(mState==Tracking::NOT_INITIALIZED)
        {
            //获取当前帧\参考帧的特征点,并且得到他们的匹配关系
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            //当系统处于运动追踪状态时
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            //跟丢的时候就之获得当前帧的特征点就可以了
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    // step 2：绘制初始化轨迹连线，绘制特征点边框（特征点用小框圈住）
    // step 2.1：初始化时，当前帧的特征坐标与初始帧的特征点坐标连成线，形成轨迹
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            //绘制当前帧特征点到下一帧特征点的连线,其实就是匹配关系
            //NOTICE 就是当初看到的初始化过程中图像中显示的绿线
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        //当前帧追踪到的特征点计数
        mnTracked=0;
        mnTrackedVO=0;

        // Draw keypoints
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            //如果这个点在视觉里程计中有(应该是追踪成功了的意思吧),在局部地图中也有
            if(vbVO[i] || vbMap[i])
            {
                //在特征点附近正方形选择四个点
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                // step2.2：正常跟踪时，在画布im中标注特征点
                if(vbMap[i])
                {
                    // 通道顺序为bgr，地图中MapPoints用绿色圆点表示，并用绿色小方框圈住
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                //BUG 但是不知道为什么，我在实际运行中时没有发现有蓝色的点存在啊
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    // 通道顺序为bgr， NOTICE 仅当前帧能观测到的MapPoints用蓝色圆点表示，并用蓝色小方框圈住
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }//遍历所有的特征点
    }

    //然后写入状态栏的信息
    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    //返回生成的图像
    return imWithInfo;
}


//绘制状态栏上的文本信息
void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        //在视觉里程计中匹配到的
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    //计算字符串文字所占用的图像区域的大小
    cv::Size textSize = cv::getTextSize(
        s.str(),                    //字符串
        cv::FONT_HERSHEY_PLAIN,     //字体
        1,                          //字体缩放
        1,                          //粗细
        &baseline);                 //基线,相对于最低端的文本点的,y坐标  //? 不是太明白
    //扩展图像
    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    //扩充区域填充黑色背景
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    //并且绘制文字
    cv::putText(
        imText,                         //目标图像
        s.str(),                        //要输出的文字
        cv::Point(5,imText.rows-5),     //输出文字的起始位置
        cv::FONT_HERSHEY_PLAIN,         //字体
        1,                              //缩放
        cv::Scalar(255,255,255),        //颜色,白色
        1,                              //线宽
        8);                             //线型

}

/**
 * @brief 将跟踪线程的数据拷贝到绘图线程（图像、特征点、地图、跟踪状态）
 * 
 * @param[in] pTracker 跟踪线程指针
 */
void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    //拷贝跟踪线程的图像
    pTracker->mImGray.copyTo(mIm);
    //拷贝跟踪线程的特征点
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);

    mbOnlyTracking = pTracker->mbOnlyTracking;

    //如果上一帧的时候,追踪器没有进行初始化
    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        //那么就要获取初始化帧的特征点和匹配信息
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    //如果上一帧是在正常跟踪
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        //获取当前帧地图点的信息
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    //该mappoints可以被多帧观测到，则为有效的地图点
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                    //否则表示这个特征点是在当前帧中第一次提取得到的点
                        mvbVO[i]=true;
                }
            }
        }
    }
    //更新追踪线程的跟踪状态
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
