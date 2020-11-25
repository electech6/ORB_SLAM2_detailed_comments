       /**
 * @file PnPsolver.h
 * @author guoqing (1337841346@qq.com)
 * @brief EPnP 相机位姿求解器，貌似这里ORB-SLAM2也是使用了开源的代码
 * @version 0.1
 * @date 2019-05-08
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
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
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/




#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include "MapPoint.h"
#include "Frame.h"

namespace ORB_SLAM2
{

/** @brief 相机位姿求解器 */
class PnPsolver {
public:

  /**
   * @brief 构造函数
   * @param[in] F                   当前要求解位姿的帧
   * @param[in] vpMapPointMatches   地图点,可能来自与局部地图,也可能来自于参考关键帧
   */
  PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches);

  /** @brief 析构函数 */
  ~PnPsolver();

  /**
   * @brief 设置RANSAC迭代的参数
   * @param[in] probability       用于计算RANSAC理论迭代次数所用的概率
   * @param[in] minInliers        退出RANSAC所需要的最小内点个数, 注意这个只是给定值,最终迭代的时候不一定按照这个来
   * @param[in] maxIterations     设定的最大RANSAC迭代次数
   * @param[in] minSet            表示求解这个问题所需要的最小的样本数目,简称最小集;参与到最小内点数的确定过程中
   * @param[in] epsilon           希望得到的 内点数/总体数 的比值,参与到最小内点数的确定过程中
   * @param[in] th2               内外点判定时的距离的baseline(程序中还会根据特征点所在的图层对这个阈值进行缩放的)
   */
  void SetRansacParameters(double probability = 0.99, int minInliers = 8 , int maxIterations = 300, int minSet = 4, float epsilon = 0.4,
                           float th2 = 5.991);

  // REVIEW 目测这个函数没有被调用过
  cv::Mat find(vector<bool> &vbInliers, int &nInliers);

  /**
   * @brief 进行迭代计算
   * 
   * @param[in] nIterations   给定的迭代次数,但是程序也有可能不听这个
   * @param[out] bNoMore      已经达到了最大迭代次数的标志
   * @param[out] vbInliers    内点标记 
   * @param[out] nInliers     内点数目
   * @return cv::Mat          Tcw
   */
  cv::Mat iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers);

 private:

  /**
  * @brief 通过之前求解的位姿来进行3D-2D投影，统计内点数目
  * 
  */
  void CheckInliers();

  /** @brief 使用已经是内点的匹配点对，再进行一次EPnP过程，进行相机位姿的求精. 
   *  @return bool 返回的结果表示经过求精过程后的内点数,能否达到退出RANSAC的要求
   */
  bool Refine();

  // ============================ Functions from the original EPnP code ===========================================
  /**
   * @brief 更新原始EPnP代码中使用的"最小集",如果符合更新条件的话还是重新生成一些用于计算的数组(不安全类型的那种,所以无法直接缩小)
   * @param[in] n 最小集
   */
  void set_maximum_number_of_correspondences(const int n);
  /** @brief 清空当前已有的匹配点计数,为进行新的一次迭代作准备 */
  void reset_correspondences(void);
  /**
   * @brief EPnP部分的函数,向堆内存的数组中添加匹配点对
   * @param[in] X    3D点
   * @param[in] Y    3D点
   * @param[in] Z    3D点
   * @param[in] u    2D点
   * @param[in] v    2D点
   */
  void add_correspondence(const double X, const double Y, const double Z,
              const double u, const double v);

  /**
   * @brief 使用EPnP算法计算相机的位姿.其中匹配点的信息由类的成员函数给定 
   * @param[out] R    旋转
   * @param[out] T    平移
   * @return double   使用这对旋转和平移的时候, 匹配点对的平均重投影误差
   */
  double compute_pose(double R[3][3], double T[3]);

  /**
   * @brief 目测没有使用到的函数, 在原版的EPnP中用于计算计算值和真值之间的相对误差
   * @param[out] rot_err           计算得到的相对旋转误差 
   * @param[out] transl_err        计算得到的相对平移误差
   * @param[in]  Rtrue             旋转真值
   * @param[in]  ttrue             平移真值
   * @param[in]  Rest              旋转计算值
   * @param[in]  test              平移计算值
   * @deprecated 在ORB中没有用到
   */
  void relative_error(double & rot_err, double & transl_err,
              const double Rtrue[3][3], const double ttrue[3],
              const double Rest[3][3],  const double test[3]);

  /**
   * @brief 输出计算得到的位姿.这个函数是EPnP源码中自带的,用于调试,所以在ORB中应当算是一个被放弃了的函数(因为我们并不需要输出啊)
   * @param[in] R 旋转
   * @param[in] t 平移 
   * @deprecated 在ORB中没有用到
   */
  void print_pose(const double R[3][3], const double t[3]);
  /**
  * @brief 计算在给定位姿的时候的3D点投影误差
  * @param[in] R      给定旋转
  * @param[in] t      给定平移
  * @return double    重投影误差,是平均到每一对匹配点上的误差
  */
  double reprojection_error(const double R[3][3], const double t[3]);

  /**
   * @brief 从给定的匹配点中计算出四个控制点
   * 
   */
  void choose_control_points(void);
  
  /**
  * @brief 求解世界坐标系下四个控制点的系数alphas，在相机坐标系下系数不变
  * 
  */
  void compute_barycentric_coordinates(void);

  /**
   * @brief 根据提供的每一对点的数据来填充Moment Matrix M. 每对匹配点的数据可以填充两行
   * @param[in] M                cvMat对应,存储矩阵M
   * @param[in] row              开始填充数据的行
   * @param[in] alphas           3D点,为这个空间点在当前控制点坐标系下的表示(a1~a4)
   * @param[in] u                2D点坐标
   * @param[in] v                2D点坐标 
   */
  void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);
  /**
   * @brief 通过给出的beta和vi,计算控制点在相机坐标系下的坐标
   * @param[in] betas       beta
   * @param[in] ut          其实是vi
   */
  void compute_ccs(const double * betas, const double * ut);

  /**
  * @brief 根据相机坐标系下控制点坐标ccs 和控制点系数 alphas（通过世界坐标系下3D点计算得到），得到相机坐标系下3D点坐标 pcs
  * 过程可以参考 https://blog.csdn.net/jessecw79/article/details/82945918
  */
  void compute_pcs(void);
  /** @brief 保持所有点在相机坐标系下的深度为正,调整符号  */
  void solve_for_sign(void);

  /**
   * @brief 计算N=4时候的粗糙近似解，暴力将其他量置为0
   * 
   * @param[in]  L_6x10  矩阵L
   * @param[in]  Rho     非齐次项 \rho, 列向量
   * @param[out] betas   计算得到的beta
   */
  void find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas);

  /**
   * @brief 计算N=2时候的粗糙近似解，暴力将其他量置为0
   * 
   * @param[in]  L_6x10  矩阵L
   * @param[in]  Rho     非齐次项 \rho, 列向量
   * @param[out] betas   计算得到的beta
   */
  void find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas);

  /**
   * @brief 计算N=3时候的粗糙近似解，暴力将其他量置为0
   * 
   * @param[in]  L_6x10  矩阵L
   * @param[in]  Rho     非齐次项 \rho, 列向量
   * @param[out] betas   计算得到的beta
   */
  void find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  /**
   * @brief 使用QR分解来求解增量方程 
   * @param[in]  A   洗漱矩阵
   * @param[in]  b   非齐次项
   * @param[out] X   增量
   */
  void qr_solve(CvMat * A, CvMat * b, CvMat * X);

  /**
   * @brief 计算两个三维向量的点乘
   * @param[in] v1 向量1
   * @param[in] v2 向量2
   * @return double 计算结果
   */
  double dot(const double * v1, const double * v2);
  /**
   * @brief 计算两个三维向量所表示的空间点的欧式距离的平方
   * @param[in] p1   点1
   * @param[in] p2   点2
   * @return double  计算的距离结果
   */
  double dist2(const double * p1, const double * p2);
 
  /**
   * @brief 计算四个控制点任意两点间的距离，总共6个距离，对应论文式13中的向量\rho
   * @param[in] rho  计算结果
   */
  void compute_rho(double * rho);

  /**
   * @brief 计算矩阵L,论文式13中的L矩阵,不过这里的是按照N=4的时候计算的
   * @param[in]  ut               特征值分解之后得到的12x12特征矩阵
   * @param[out] l_6x10           计算的L矩阵结果，维度6x10 
   */
  void compute_L_6x10(const double * ut, double * l_6x10);
  /**
   * @brief 对计算出来的Beta结果进行高斯牛顿法优化,求精. 过程参考EPnP论文中式(15) 
   * @param[in]  L_6x10            L矩阵
   * @param[in]  Rho               Rho向量
   * @param[out] current_betas     优化之后的Beta
   */
  void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);
  /**
   * @brief 计算高斯牛顿法优化时,增量方程中的系数矩阵和非齐次项
   * @param[in]  l_6x10 L矩阵
   * @param[in]  rho    Rho矩向量
   * @param[in]  cb     当前次迭代得到的beta1~beta4
   * @param[out] A      计算得到的增量方程中的系数矩阵
   * @param[out] b      计算得到的增量方程中的非齐次项
   */
  void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
				    double cb[4], CvMat * A, CvMat * b);

  /**
   * @brief 根据已经得到的控制点在当前相机坐标系下的坐标来恢复出相机的位姿
   * @param[in]  ut         vi
   * @param[in]  betas      betas
   * @param[out] R          计算得到的相机旋转R
   * @param[out] t          计算得到的相机位置t
   * @return double         使用这个位姿,所得到的重投影误差
   */
  double compute_R_and_t(const double * ut, const double * betas,
			 double R[3][3], double t[3]);

  /**
   * @brief 用3D点在世界坐标系和相机坐标系下对应的坐标，用ICP求取R t
   * @param[out] R   旋转
   * @param[out] t   平移
   */
  void estimate_R_and_t(double R[3][3], double t[3]);
  /**
   * @brief 复制计算得到的位姿到另外的一组变量中
   * @param[in]  R_dst 
   * @param[in]  t_dst 
   * @param[out] R_src 
   * @param[out] t_src 
   */
  void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
		    double R_src[3][3], double t_src[3]);

  /**
   * @brief 目测没有使用到的函数, 在原版的EPnP中勇于将旋转矩阵转换成为四元数的表达形式
   * @param[in]  R          需要转换的旋转矩阵
   * @param[out] q          转换后得到的四元数表达
   * @deprecated 在ORB中没有用到
   */
  void mat_to_quat(const double R[3][3], double q[4]);


  double uc, vc, fu, fv;                                          // 相机内参

  double * pws,                                                   // 3D点在世界坐标系下在坐标
                                                                  //   组织形式: x1 y1 z1 | x2 y2 z2 | ...
         * us,                                                    // 图像坐标系下的2D点坐标
                                                                  //   组织形式: u1 v1 | u2 v2 | ...
         * alphas,                                                // 真实3D点用4个虚拟控制点表达时的系数
                                                                  //   组织形式: a11 a12 a13 a14 | a21 a22 a23 a24 | ... 每个匹配点都有自己的a1~a4
         * pcs;                                                   // 3D点在当前帧相机坐标系下的坐标 
  int maximum_number_of_correspondences;                          // 每次RANSAC计算的过程中使用的匹配点对数的最大值,其实应该和最小集的大小是完全相同的
  int number_of_correspondences;                                  // 当前次迭代中,已经采样的匹配点的个数，默认值为4

  double cws[4][3],                                               // 存储控制点在世界坐标系下的坐标，第一维表示是哪个控制点，第二维表示是哪个坐标(x,y,z)
         ccs[4][3];                                               // 存储控制点在相机坐标系下的坐标, 含义同上
  double cws_determinant;                                         // 没有被使用到的变量,但是看变量名字,应该是用于存储某个矩阵的行列式值的

  vector<MapPoint*> mvpMapPointMatches;                           // 存储构造的时候给出的地图点 

  // 2D Points
  vector<cv::Point2f> mvP2D;                                      // 存储当前帧的2D点,由特征点转换而来,只保存了坐标信息
  vector<float> mvSigma2;                                         // 和2D特征点向量下标对应的尺度和不确定性信息(从该特征点所在的金字塔图层有关)

  // 3D Points
  vector<cv::Point3f> mvP3Dw;                                     // 存储给出的地图点中有效的地图点(在世界坐标系下的坐标)

  // Index in Frame
  vector<size_t> mvKeyPointIndices;                               // 记录构造时给出的地图点对应在帧中的特征点的id,这个是"跳跃的"

  // Current Estimation
  double mRi[3][3];                                               // 在某次RANSAC迭代过程中计算得到的旋转矩阵
  double mti[3];                                                  // 在某次RANSAC迭代过程中计算得到的平移向量
  cv::Mat mTcwi;                                                  // 在程序中并没有被使用到的变量
  vector<bool> mvbInliersi;                                       // 记录每次迭代时的inlier点
  int mnInliersi;                                                 // 记录每次迭代时的inlier点的数目

  // Current Ransac State
  int mnIterations;                                               // 历史上已经进行的RANSAC迭代次数
  vector<bool> mvbBestInliers;                                    // 历史上最好一次迭代时的内点标记
  int mnBestInliers;                                              // 历史上的迭代中最多的内点数
  cv::Mat mBestTcw;                                               // 历史上最佳的一次迭代所计算出来的相机位姿

  // Refined
  cv::Mat mRefinedTcw;                                            // 求精之后得到的相机位姿
  vector<bool> mvbRefinedInliers;                                 // 求精之后的内点标记
  int mnRefinedInliers;                                           // 求精之后的内点数

  // Number of Correspondences
  int N;                                                          // 就是 mvP2D 的大小,表示给出帧中和地图点匹配的特征点的个数,也就是匹配的对数(相当于采样的总体)

  // Indices for random selection [0 .. N-1]
  vector<size_t> mvAllIndices;                                    // 记录特征点在当前求解器中的向量中存储的索引,是连续的 //存储了供RANSAC过程使用的点的下标

  // RANSAC probability
  double mRansacProb;                                             // 计算RANSAC迭代次数的理论值的时候用到的概率,和Sim3Slover中的一样

  // RANSAC min inliers
  int mRansacMinInliers;                                          // 正常退出RANSAC的时候需要达到的最最少的内点个数f

  // RANSAC max iterations
  int mRansacMaxIts;                                              // RANSAC的最大迭代次数

  // RANSAC expected inliers/total ratio
  float mRansacEpsilon;                                           // RANSAC中,最小内点数占全部点个数的比例

  // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
  float mRansacTh;                                                // 在程序中并没有使用到的变量

  // RANSAC Minimun Set used at each iteration
  int mRansacMinSet;                                              // 为每次RANSAC需要的特征点数，默认为4组3D-2D对应点. 参与到最少内点数的确定过程中

  // Max square error associated with scale level. Max error = th*th*sigma(level)*sigma(level)
  vector<float> mvMaxError;                                       // 存储不同图层上的特征点在进行内点验证的时候,使用的不同的距离阈值

};

} //namespace ORB_SLAM

#endif //PNPSOLVER_H
