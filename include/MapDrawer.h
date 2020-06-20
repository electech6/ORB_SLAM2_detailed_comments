/**
 * @file MapDrawer.h
 * @author guoqing (1337841346@qq.com)
 * @brief 绘制地图点
 * @version 0.1
 * @date 2019-02-19 update 2020-06-20
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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

// 包含必要头文件
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
// Pangolin 支持
#include <pangolin/pangolin.h>
、、 线程锁支持
#include<mutex>

namespace ORB_SLAM2
{

class MapDrawer
{
public:
    /**
     * @brief 构造函数
     * @param[in] pMap              地图句柄
     * @param[in] strSettingPath    配置文件的路径
     */
    MapDrawer(Map* pMap, const string &strSettingPath);
    
    //地图对象句柄
    Map* mpMap;

    /** @brief 绘制地图点 */
    void DrawMapPoints();

    /**
     * @brief 绘制关键帧
     * @param[in] bDrawKF       是否绘制关键帧
     * @param[in] bDrawGraph    是否绘制共视图
     */
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

    /**
     * @brief 绘制当前相机
     * @param[in] Twc 相机的位姿矩阵
     */
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

    /**
     * @brief 设置当前帧的相机位姿
     * @param[in] Tcw 位姿矩阵
     */
    void SetCurrentCameraPose(const cv::Mat &Tcw);

    /**
     * @brief 设置参考关键帧 -- // ? 感觉这个在可视化过程中体现不出来呢? 
     * @param[in] pKF 参考关键帧的句柄
     */
    void SetReferenceKeyFrame(KeyFrame *pKF);

    /**
     * @brief 将当前设置的相机位姿 mCameraPose 由 cv::Mat 类型转化为 pangolin::OpenGlMatrix 类型
     * @param[out] M 相机位姿矩阵
     */
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:

    //绘制这些部件的参数
    float mKeyFrameSize;                    ///< 绘制的关键帧图元大小
    float mKeyFrameLineWidth;               ///< 绘制的关键帧图元线宽
    float mGraphLineWidth;                  ///< 绘制的共视图边的线宽
    float mPointSize;                       ///< 绘制的地图点图元大小
    float mCameraSize;                      ///< 绘制的当前帧图元大小(实际上用的就是关键帧的图元)
    float mCameraLineWidth;                 ///< 绘制的当前帧图元线宽

    cv::Mat mCameraPose;                    ///< 当前帧相机所在的位姿
    std::mutex mMutexCamera;                ///< 用于避免冲突的线程互斥锁
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
