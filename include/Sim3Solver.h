/**
 * @file Sim3Solver.h
 * @author guoqing (1337841346@qq.com)
 * @brief sim3 求解
 * @version 0.1
 * @date 2019-05-07
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


#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "KeyFrame.h"



namespace ORB_SLAM2
{

/** @brief Sim3 求解器 */
class Sim3Solver
{
public:

    /**
     * @brief Sim 3 Solver 构造函数
     * @param[in] pKF1              当前关键帧
     * @param[in] pKF2              候选的闭环关键帧
     * @param[in] vpMatched12       通过词袋模型加速匹配所得到的,两帧特征点的匹配关系所得到的地图点,本质上是来自于候选闭环关键帧的地图点
     * @param[in] bFixScale         当前传感器类型的输入需不需要计算尺度。单目的时候需要，双目和RGBD的时候就不需要了
     */
    Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12, const bool bFixScale = true);

    /**
     * @brief 设置进行RANSAC时的参数
     * 
     * @param[in] probability           当前这些匹配点的置信度，也就是一次采样恰好都是内点的概率
     * @param[in] minInliers            完成RANSAC所需要的最少内点个数
     * @param[in] maxIterations         设定的最大迭代次数
     */
    void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

    /**
     * @brief 在下面的这个"进行迭代计算"函数的基础上套了一层壳,使用默认参数. 不过目前好像没有被使用到
     * @param[out] vbInliers12      内点标记,下标和构造时给出的地图点向量保持一致
     * @param[out] nInliers         内点数目
     * @return cv::Mat              计算得到的变换矩阵.如果期间计算出现了问题,那么返回的是空矩阵
     */
    cv::Mat find(std::vector<bool> &vbInliers12, int &nInliers);

    /**
     * @brief Ransac求解mvX3Dc1和mvX3Dc2之间Sim3，函数返回mvX3Dc2到mvX3Dc1的Sim3变换
     * 
     * @param[in] nIterations           设置的最大迭代次数
     * @param[in] bNoMore               为true表示穷尽迭代还没有找到好的结果，说明求解失败
     * @param[in] vbInliers             标记是否是内点
     * @param[in] nInliers              内点数目
     * @return cv::Mat                  计算得到的Sim3矩阵
     */
    cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

    /**
     * @brief 获取计算的旋转矩阵
     * @return cv::Mat RANSAC过程中计算得到的最优解的旋转矩阵
     */
    cv::Mat GetEstimatedRotation();
    /**
     * @brief 获取计算的平移向量
     * @return cv::Mat 平移向量
     */
    cv::Mat GetEstimatedTranslation();
    /**
     * @brief 获取估计的从候选帧到当前帧的变换尺度
     * @return float 尺度因子
     */
    float GetEstimatedScale();


protected:

    /**
     * @brief 给出三个点,计算它们的质心以及去质心之后的坐标
     * 
     * @param[in] P     输入的3D点
     * @param[in] Pr    去质心后的点
     * @param[in] C     质心
     */
    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

    /**
     * @brief 根据两组匹配的3D点,计算P2到P1的Sim3变换
     * @param[in] P1    匹配的3D点(三个,每个的坐标都是列向量形式,三个点组成了3x3的矩阵)(当前关键帧)
     * @param[in] P2    匹配的3D点(闭环关键帧)
     */
    void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

    /**
     * @brief 通过计算的Sim3投影，和自身投影的误差比较，进行内点检测
     * 
     */
    void CheckInliers();

    /**
     * @brief 按照给定的Sim3变换进行投影操作,得到三维点的2D投影点
     * 
     * @param[in] vP3Dw         3D点
     * @param[in & out] vP2D    投影到图像的2D点
     * @param[in] Tcw           Sim3变换
     * @param[in] K             内参
     */
    void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K);

    /**
     * @brief 计算当前关键帧中的地图点在当前关键帧图像上的投影坐标
     * 
     * @param[in] vP3Dc         相机坐标系下三维点坐标
     * @param[in] vP2D          投影的二维图像坐标
     * @param[in] K             内参矩阵
     */
    void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K);


protected:

    // KeyFrames and matches
    KeyFrame* mpKF1;                            // 当前关键帧
    KeyFrame* mpKF2;                            // 闭环关键帧

    std::vector<cv::Mat> mvX3Dc1;               // 存储匹配的,当前关键帧中的地图点在当前关键帧相机坐标系下的坐标
    std::vector<cv::Mat> mvX3Dc2;               // 存储匹配的,闭环关键帧中的地图点在闭环关键帧相机坐标系下的坐标
    std::vector<MapPoint*> mvpMapPoints1;       // 匹配的地图点的中,存储当前关键帧的地图点
    std::vector<MapPoint*> mvpMapPoints2;       // 匹配的地图点的中,存储闭环关键帧的地图点
    std::vector<MapPoint*> mvpMatches12;        // 下标是当前关键帧中特征点的id,内容是对应匹配的,闭环关键帧中的地图点
    std::vector<size_t> mvnIndices1;            // 有效的匹配关系,在 vpMatched12 (构造函数) 中的索引
    std::vector<size_t> mvSigmaSquare1;         // 这个变量好像是没有被用到
    std::vector<size_t> mvSigmaSquare2;         // 这个变量好像是没有被用到
    std::vector<size_t> mvnMaxError1;           // 当前关键帧中的某个特征点所允许的最大不确定度(和所在的金字塔图层有关)
    std::vector<size_t> mvnMaxError2;           // 闭环关键帧中的某个特征点所允许的最大不确定度(同上)

    int N;                                      // 下面的这个匹配关系去掉坏点和非法值之后,得到的可靠的匹配关系的点的数目
    int mN1;                                    // 当前关键帧和闭环关键帧之间形成匹配关系的点的数目(Bow加速得到的匹配点)

    // Current Estimation
    cv::Mat mR12i;                              // 存储某次RANSAC过程中得到的旋转
    cv::Mat mt12i;                              // 存储某次RANSAC过程中得到的平移
    float ms12i;                                // 存储某次RANSAC过程中得到的缩放系数
    cv::Mat mT12i;                              // 存储某次RANSAC过程中得到的变换矩阵
    cv::Mat mT21i;                              // 上面的逆
    std::vector<bool> mvbInliersi;              // 内点标记,下标和N,mvpMapPoints1等一致,用于记录某次迭代过程中的内点情况
    int mnInliersi;                             // 在某次迭代的过程中经过投影误差进行的inlier检测得到的内点数目

    // Current Ransac State
    int mnIterations;                           // RANSAC迭代次数(当前正在进行的)
    std::vector<bool> mvbBestInliers;           // 累计的,多次RANSAC中最好的最多的内点个数时的内点标记
    int mnBestInliers;                          // 最好的一次迭代中,得到的内点个数
    cv::Mat mBestT12;                           // 存储最好的一次迭代中得到的变换矩阵
    cv::Mat mBestRotation;                      // 存储最好的一次迭代中得到的旋转
    cv::Mat mBestTranslation;                   // 存储最好的一次迭代中得到的平移
    float mBestScale;                           // 存储最好的一次迭代中得到的缩放系数

    // Scale is fixed to 1 in the stereo/RGBD case
    bool mbFixScale;                            // 当前传感器输入的情况下,是否需要计算尺度

    // Indices for random selection
    std::vector<size_t> mvAllIndices;           // RANSAC中随机选择的时候,存储可以选择的点的id(去除那些存在问题的匹配点（外点）后重新排序)

    // Projections   ？？？不太明白这个部分
    std::vector<cv::Mat> mvP1im1;               // 当前关键帧中的地图点在当前关键帧图像上的投影坐标
    std::vector<cv::Mat> mvP2im2;               // 闭环关键帧中的地图点在闭环关键帧图像上的投影坐标 

    // RANSAC probability
    double mRansacProb;                         // 在计算RANSAC的理论迭代次数时使用到的概率,详细解释还是看函数 SetRansacParameters() 中的注释吧

    // RANSAC min inliers
    int mRansacMinInliers;                      // RANSAC 结束的理想条件: 结束RANSAC过程所需要的最少内点数

    // RANSAC max iterations
    int mRansacMaxIts;                          // RANSAC 结束的不理想条件: 最大迭代次数

    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    float mTh;                                  // 没有使用到的变量
    float mSigma2;                              // 没有使用到的变量

    // Calibration
    cv::Mat mK1;                                // 当前关键帧的内参矩阵
    cv::Mat mK2;                                // 闭环关键帧的内参矩阵

};

} //namespace ORB_SLAM

#endif // SIM3SOLVER_H
