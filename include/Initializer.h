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


/**
 * @file Initializer.h
 * @author guoqing (1337841346@qq.com)
 * @brief 单目初始化部分的声明. 双目 和RGBD输入的情况下,不会使用这个类
 * @version 0.1
 * @date 2019-01-11
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"


namespace ORB_SLAM2
{

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
/**
 * @brief 单目SLAM初始化相关，双目和RGBD不会使用这个类
 */
class Initializer
{
    typedef pair<int,int> Match;

public:

    // Fix the reference frame
    /**
     * @brief 根据参考帧构造初始化器
     * 
     * @param[in] ReferenceFrame        参考帧
     * @param[in] sigma                 测量误差
     * @param[in] iterations            RANSAC迭代次数
     */
    Initializer(const Frame &ReferenceFrame,    
                float sigma = 1.0,              
                int iterations = 200);          

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    /**
     * @brief 计算基础矩阵和单应性矩阵，选取最佳的来恢复出最开始两帧之间的相对姿态，并进行三角化得到初始地图点
     * Step 1 重新记录特征点对的匹配关系
     * Step 2 在所有匹配特征点对中随机选择8对匹配特征点为一组，用于估计H矩阵和F矩阵
     * Step 3 计算fundamental 矩阵 和homography 矩阵，为了加速分别开了线程计算 
     * Step 4 计算得分比例来判断选取哪个模型来求位姿R,t
     * 
     * @param[in] CurrentFrame          当前帧，也就是SLAM意义上的第二帧
     * @param[in] vMatches12            当前帧（2）和参考帧（1）图像中特征点的匹配关系
     *                                  vMatches12[i]解释：i表示帧1中关键点的索引值，vMatches12[i]的值为帧2的关键点索引值
     *                                  没有匹配关系的话，vMatches12[i]值为 -1
     * @param[in & out] R21                   相机从参考帧到当前帧的旋转
     * @param[in & out] t21                   相机从参考帧到当前帧的平移
     * @param[in & out] vP3D                  三角化测量之后的三维地图点
     * @param[in & out] vbTriangulated        标记三角化点是否有效，有效为true
     * @return true                     该帧可以成功初始化，返回true
     * @return false                    该帧不满足初始化条件，返回false
     */
    bool Initialize(const Frame &CurrentFrame,  
                    const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21,   
                    vector<cv::Point3f> &vP3D,    
                    vector<bool> &vbTriangulated);
	
private:
    /**
     * @brief 计算单应矩阵，假设场景为平面情况下通过前两帧求取Homography矩阵，并得到该模型的评分
     * 原理参考Multiple view geometry in computer vision  P109 算法4.4
     * Step 1 将当前帧和参考帧中的特征点坐标进行归一化
     * Step 2 选择8个归一化之后的点对进行迭代
     * Step 3 八点法计算单应矩阵矩阵
     * Step 4 利用重投影误差为当次RANSAC的结果评分
     * Step 5 更新具有最优评分的单应矩阵计算结果,并且保存所对应的特征点对的内点标记
     * 
     * @param[in & out] vbMatchesInliers          标记是否是外点
     * @param[in & out] score                     计算单应矩阵的得分
     * @param[in & out] H21                       单应矩阵结果
     */
    void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
  
    /**
     * @brief 计算基础矩阵，假设场景为非平面情况下通过前两帧求取Fundamental矩阵，得到该模型的评分
     * Step 1 将当前帧和参考帧中的特征点坐标进行归一化
     * Step 2 选择8个归一化之后的点对进行迭代
     * Step 3 八点法计算基础矩阵矩阵
     * Step 4 利用重投影误差为当次RANSAC的结果评分
     * Step 5 更新具有最优评分的基础矩阵计算结果,并且保存所对应的特征点对的内点标记
     * 
     * @param[in & out] vbMatchesInliers          标记是否是外点
     * @param[in & out] score                     计算基础矩阵得分
     * @param[in & out] F21                       基础矩阵结果
     */
    void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

    /**
     * @brief 用DLT方法求解单应矩阵H
     * 这里最少用4对点就能够求出来，不过这里为了统一还是使用了8对点求最小二乘解
     * 
     * @param[in] vP1               参考帧中归一化后的特征点
     * @param[in] vP2               当前帧中归一化后的特征点
     * @return cv::Mat              计算的单应矩阵
     */
    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    
    /**
     * @brief 从特征点匹配求fundamental matrix（normalized 8点法）
     * @details 被FindFundamental函数调用具体来算Fundamental矩阵
     * @param[in]  vP1 	归一化后的点, in reference frame
     * @param[in]  vP2 	归一化后的点, in current frame
     * @return cv::Mat  基础矩阵
     * @see        Multiple View Geometry in Computer Vision - Algorithm 11.1 p282 (中文版 p191)
     */
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

    /**
     * @brief 对给定的homography matrix打分
     * @details 被FindHomography函数调用，具体来算假设使用Homography模型的得分
     * @param[in] H21				从参考帧到当前帧的单应矩阵
     * @param[in] H12				从当前帧到参考帧的单应矩阵
     * @param[out] vbMatchesInliers	匹配好的特征点对的Inliers标记
     * @param[in] sigma				方差
     * @see
     * - Author's paper - IV. AUTOMATIC MAP INITIALIZATION （2）
     * - Multiple View Geometry in Computer Vision - symmetric transfer errors: 4.2.2 Geometric distance
     * - Multiple View Geometry in Computer Vision - model selection 4.7.1 RANSAC
     */
    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);
    
    /**
     * @brief 对给定的fundamental matrix打分
     * @detials 被FindFundamental函数调用，具体来算假设使用Fundamental模型的得分
     * @param[in] F21				从当前帧到参考帧的基础矩阵
     * @param[out] vbMatchesInliers	匹配的特征点对属于inliers的标记
     * @param[in] sigma				估计误差
     * @todo  这个估计误差是啥啊
     * @see
     * - Author's paper - IV. AUTOMATIC MAP INITIALIZATION （2）
     * - Multiple View Geometry in Computer Vision - symmetric transfer errors: 4.2.2 Geometric distance
     * - Multiple View Geometry in Computer Vision - model selection 4.7.1 RANSAC
     */

    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

    /**
     * @brief 从F恢复R t
     * @details 度量重构
     * \n 1. 由Fundamental矩阵结合相机内参K，得到Essential矩阵: \f$ E = k'^T F k \f$
     * \n 2. SVD分解得到R t
     * \n 3. 进行cheirality check, 从四个解中找出最合适的解
     * 
     * @param[in] vbMatchesInliers	匹配好的特征点对的Inliers标记
     * @param[in] F21				从参考帧到当前帧的基础矩阵
     * @param[in] K					相机的内参数矩阵
     * @param[out] R21				计算好的相机从参考帧到当前帧的旋转
     * @param[out] t21				计算好的相机从参考帧到当前帧的平移
     * @param[out] vP3D				三角化测量之后的特征点的空间坐标
     * @param[out] vbTriangulated	某个特征点是否被三角化了的标记
     * @param[in] minParallax		认为三角化测量有效的最小视差角
     * @param[in] minTriangulated	认为使用三角化测量进行数据判断的最小测量点数量
     * @return 	bool 是否解析成功
     * 
     * @see Multiple View Geometry in Computer Vision - Result 9.19 p259
     */
    bool ReconstructF(vector<bool> &vbMatchesInliers,
                      cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21,
                      cv::Mat &t21,
                      vector<cv::Point3f> &vP3D,
                      vector<bool> &vbTriangulated,
                      float minParallax,
                      int minTriangulated);

    /**
     * @brief 从H恢复R t
     * @details 分解H矩阵，并从分解后的多个解中找出合适的R，t
     * \n H矩阵分解常见有两种方法：Faugeras SVD-based decomposition 和 Zhang SVD-based decomposition
     * \n 参考文献：Motion and structure from motion in a piecewise plannar environment
     * \n 这篇参考文献和下面的代码使用了Faugeras SVD-based decomposition算法
     * @param[in] vbMatchesInliers	匹配点对的内点标记
     * @param[in] H21				从参考帧到当前帧的单应矩阵
     * @param[in] K					相机的内参数矩阵
     * @param[out] R21				计算出来的相机旋转
     * @param[out] t21				计算出来的相机平移
     * @param[out] vP3D				世界坐标系下，三角化测量特征点对之后得到的特征点的空间坐标
     * @param[out] vbTriangulated	特征点对被三角化测量的标记
     * @param[in] minParallax		在进行三角化测量时，观测正常所允许的最小视差角
     * @param[in] minTriangulated	最少被三角化的点对数（其实也是点个数）
     * @return	bool 数据解算是否成功
     * @see
     * - Faugeras et al, Motion and structure from motion in a piecewise planar environment. International Journal of Pattern Recognition and Artificial Intelligence, 1988.
     * - Deeper understanding of the homography decomposition for vision-based control
     */
    bool ReconstructH(vector<bool> &vbMatchesInliers,
                      cv::Mat &H21,
                      cv::Mat &K,
                      cv::Mat &R21,
                      cv::Mat &t21,
                      vector<cv::Point3f> &vP3D,
                      vector<bool> &vbTriangulated,
                      float minParallax,
                      int minTriangulated);

    /**
     * @brief 给定投影矩阵P1,P2和图像上的点kp1,kp2，从而恢复3D坐标
     * @details 通过三角化方法，利用反投影矩阵将特征点恢复为3D点
     * @param[in]   kp1 特征点, in reference frame
     * @param[in]   kp2 特征点, in current frame
     * @param[in]   P1  投影矩阵P1, 注意这里给的参数是投影矩阵
     * @param[in]   P2  投影矩阵P2
     * @param[out]  x3D 三维点
     * @see       Multiple View Geometry in Computer Vision - 12.2 Linear triangulation methods p312
     */
    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

  
    /**
     * @brief 归一化特征点到同一尺度，作为后续normalize DLT的输入
     *  [x' y' 1]' = T * [x y 1]' 
     *  归一化后x', y'的均值为0，sum(abs(x_i'-0))=1，sum(abs((y_i'-0))=1
     * 
     *  为什么要归一化？
     *  在相似变换之后(点在不同的坐标系下),他们的单应性矩阵是不相同的
     *  如果图像存在噪声,使得点的坐标发生了变化,那么它的单应性矩阵也会发生变化
     *  我们采取的方法是将点的坐标放到同一坐标系下,并将缩放尺度也进行统一 
     *  对同一幅图像的坐标进行相同的变换,不同图像进行不同变换
     *  缩放尺度是为了让噪声对于图像的影响在一个数量级上
     * 
     *  Step 1 计算特征点X,Y坐标的均值 
     *  Step 2 计算特征点X,Y坐标离均值的平均偏离程度
     *  Step 3 将x坐标和y坐标分别进行尺度归一化，使得x坐标和y坐标的一阶绝对矩分别为1 
     *  Step 4 计算归一化矩阵：其实就是前面做的操作用矩阵变换来表示而已
     * 
     * @param[in] vKeys                               待归一化的特征点
     * @param[in & out] vNormalizedPoints             特征点归一化后的坐标
     * @param[in & out] T                             归一化特征点的变换矩阵
     */
    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    // 

    /**
     * @brief 进行cheirality check，从而进一步找出F分解后最合适的解
     * @detials ReconstructF调用该函数进行cheirality check，从而进一步找出F分解后最合适的解
     * @param[in]   R				    待检查的相机旋转矩阵R
     * @param[in]   t				    待检查的相机旋转矩阵t
     * @param[in]   vKeys1			    参考帧特征点
     * @param[in]   vKeys2			    当前帧特征点
     * @param[in]   vMatches12		    两帧特征点的匹配关系
     * @param[in]   vbMatchesInliers    特征点对的Inliers标记
     * @param[in]   K				    相机的内参数矩阵
     * @param[out]  vP3D				三角化测量之后的特征点的空间坐标
     * @param[in]   th2				    重投影误差的阈值
     * @param[out]  vbGood			    特征点（对）中是good点的标记
     * @param[out]  parallax			计算出来的比较大的视差角（注意不是最大，这个要看后面中程序的注释）
     * @return	int 返回本组解中good点的数目
     */
    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

    /**
     * @brief 分解Essential矩阵
     * @detials F矩阵通过结合内参可以得到Essential矩阵，分解E矩阵将得到4组解 \n
     * 这4组解分别为[R1,t],[R1,-t],[R2,t],[R2,-t]
     * @param[in]   E  Essential Matrix
     * @param[out]  R1 Rotation Matrix 1
     * @param[out]  R2 Rotation Matrix 2
     * @param[out]  t  Translation，另外一个结果取它的相反数就行
     * @see Multiple View Geometry in Computer Vision - Result 9.19 p259
     */
    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);


    // Keypoints from Reference Frame (Frame 1)
    /** 存储Reference Frame中的特征点 */
    vector<cv::KeyPoint> mvKeys1; 

    // Keypoints from Current Frame (Frame 2)
    /** 存储Current Frame中的特征点 */
    vector<cv::KeyPoint> mvKeys2; 

    // Current Matches from Reference to Current
    // Reference Frame: 1, Current Frame: 2
    /** Match的数据结构是pair,mvMatches12只记录Reference到Current匹配上的特征点对  */
    vector<Match> mvMatches12;
    /** 记录Reference Frame的每个特征点在Current Frame是否有匹配的特征点 */ 
    vector<bool> mvbMatched1; 

    // Calibration
    /** 相机内参 */
    cv::Mat mK; 

    // Standard Deviation and Variance
    /** 测量误差 */
    float mSigma, mSigma2; 

    // Ransac max iterations
    /** 算Fundamental和Homography矩阵时RANSAC迭代次数  */
    int mMaxIterations; 

    // Ransac sets
    /** 二维容器，外层容器的大小为迭代次数，内层容器大小为每次迭代算H或F矩阵需要的点,实际上是八对 */
    vector<vector<size_t> > mvSets; 

};

} //namespace ORB_SLAM

#endif // INITIALIZER_H
