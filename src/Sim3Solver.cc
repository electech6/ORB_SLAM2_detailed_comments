/**
 * @file Sim3Solver.cc
 * @author guoqing (1337841346@qq.com)
 * @brief sim3 求解器
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


#include "Sim3Solver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace ORB_SLAM2
{

 /**
 * @brief Sim 3 Solver 构造函数
 * @param[in] pKF1              当前关键帧
 * @param[in] pKF2              候选的闭环关键帧
 * @param[in] vpMatched12       通过词袋模型加速匹配所得到的,两帧特征点的匹配关系所得到的地图点,本质上是来自于候选闭环关键帧的地图点
 * @param[in] bFixScale         当前传感器类型的输入需不需要计算尺度。单目的时候需要，双目和RGBD的时候就不需要了
 */
Sim3Solver::Sim3Solver(KeyFrame *pKF1, KeyFrame *pKF2, const vector<MapPoint *> &vpMatched12, const bool bFixScale):
    mnIterations(0), mnBestInliers(0), mbFixScale(bFixScale)
{
    mpKF1 = pKF1;       // 当前关键帧
    mpKF2 = pKF2;       // 闭环关键帧

    // Step 1 取出当前关键帧中的所有地图点
    vector<MapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

    // 最多匹配的地图点数目
    mN1 = vpMatched12.size();

    // 预分配空间，在后面使用
    mvpMapPoints1.reserve(mN1);
    mvpMapPoints2.reserve(mN1);
    mvpMatches12 = vpMatched12;
    mvnIndices1.reserve(mN1);
    mvX3Dc1.reserve(mN1);
    mvX3Dc2.reserve(mN1);

    // 获取两个关键帧的位姿
    cv::Mat Rcw1 = pKF1->GetRotation();
    cv::Mat tcw1 = pKF1->GetTranslation();
    cv::Mat Rcw2 = pKF2->GetRotation();
    cv::Mat tcw2 = pKF2->GetTranslation();

    mvAllIndices.reserve(mN1);

    size_t idx=0;
    // Step 2 记录匹配地图点的各种信息 
    for(int i1=0; i1<mN1; i1++)
    {
        // 如果该特征点在pKF1中有匹配
        if(vpMatched12[i1])
        {
            // pMP1和pMP2是匹配的MapPoint
            MapPoint* pMP1 = vpKeyFrameMP1[i1]; //i1 是匹配地图点在KF1的索引
            MapPoint* pMP2 = vpMatched12[i1];   //vpMatched12[i1] 是在KF2中对应的匹配地图点

            if(!pMP1)
                continue;

            if(pMP1->isBad() || pMP2->isBad())
                continue;

            // indexKF1和indexKF2是匹配特征点的索引
            int indexKF1 = pMP1->GetIndexInKeyFrame(pKF1);
            int indexKF2 = pMP2->GetIndexInKeyFrame(pKF2);

            if(indexKF1<0 || indexKF2<0)
                continue;

            // kp1和kp2是匹配的图像特征点
            const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[indexKF2];

            const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];

	        // 自由度为2的卡方分布，显著性水平为0.01，对应的临界阈值为9.21
            mvnMaxError1.push_back(9.210*sigmaSquare1);
            mvnMaxError2.push_back(9.210*sigmaSquare2);

            // mvpMapPoints1和mvpMapPoints2是匹配的MapPoints容器
            mvpMapPoints1.push_back(pMP1);
            mvpMapPoints2.push_back(pMP2);
            mvnIndices1.push_back(i1);

            // 计算这对匹配地图点分别在各自相机坐标系下的坐标（用来计算SIM3）
            cv::Mat X3D1w = pMP1->GetWorldPos();
            mvX3Dc1.push_back(Rcw1*X3D1w+tcw1);

            cv::Mat X3D2w = pMP2->GetWorldPos();
            mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);

            // 所有有效三维点的索引
            mvAllIndices.push_back(idx);
            idx++;
        }
    } 

    mK1 = pKF1->mK;
    mK2 = pKF2->mK;

    // Step 3 将相机坐标系下的三维地图点分别投影到各自相机的二维图像坐标，用于后面和Sim3投影的比较，筛选内点
    FromCameraToImage(mvX3Dc1,mvP1im1,mK1);
    FromCameraToImage(mvX3Dc2,mvP2im2,mK2);

    // Step 4 设置默认的RANSAC参数,避免在调用的时候因为忘记设置导致崩溃
    SetRansacParameters();
}

/**
 * @brief 设置进行RANSAC时的参数
 * 
 * @param[in] probability           当前这些匹配点的置信度，也就是一次采样恰好都是内点的概率
 * @param[in] minInliers            完成RANSAC所需要的最少内点个数
 * @param[in] maxIterations         设定的最大迭代次数
 */
void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations)
{
    mRansacProb = probability;              // 0.99
    mRansacMinInliers = minInliers;         // 20
    mRansacMaxIts = maxIterations;          // 最大迭代次数 300

    // 匹配点的数目
    N = mvpMapPoints1.size(); // number of correspondences

    // 内点标记向量
    mvbInliersi.resize(N);

    // Adjust Parameters according to number of correspondences
    float epsilon = (float)mRansacMinInliers/N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations 
    // 计算迭代次数的理论值，也就是经过这么多次采样，其中至少有一次采样中,三对点都是内点
    // epsilon 表示了在这 N 对匹配点中,我随便抽取一对点是内点的概率; 
    // 为了计算Sim3,我们需要从这N对匹配点中取三对点;那么如果我有放回的从这些点中抽取三对点,取这三对点均为内点的概率是 p0=epsilon^3
    // 相应地,如果取三对点中至少存在一对匹配点是外点, 概率为p1=1-p0
    // 当我们进行K次采样的时候,其中每一次采样中三对点中都存在至少一对外点的概率就是p2=p1^k
    // K次采样中,至少有一次采样中三对点都是内点的概率是p=1-p2
    // 候根据 p2=p1^K 我们就可以导出 K 的公式：K=\frac{\log p2}{\log p1}=\frac{\log(1-p)}{\log(1-epsilon^3)}
    // 也就是说，我们进行K次采样,其中至少有一次采样中,三对点都是内点; 因此我们就得到了RANSAC迭代次数的理论值
    int nIterations;

    if(mRansacMinInliers==N)        
        nIterations=1; // 这种情况的时候最后计算得到的迭代次数的确就是一次
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon,3)));   

    // 外层的max保证RANSAC能够最少迭代一次;
    // 内层的min的目的是,如果理论值比给定值要小,那么我们优先选择使用较少的理论值来节省时间(其实也有极大概率得到能够达到的最好结果);
    // 如果理论值比给定值要大,那么我们也还是有限选择使用较少的给定值来节省时间
    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    // 当前正在进行的迭代次数
    mnIterations = 0;
}

/**
 * @brief Ransac求解mvX3Dc1和mvX3Dc2之间Sim3，函数返回mvX3Dc2到mvX3Dc1的Sim3变换
 * 
 * @param[in] nIterations           设置的最大迭代次数
 * @param[in] bNoMore               为true表示穷尽迭代还没有找到好的结果，说明求解失败
 * @param[in] vbInliers             标记是否是内点
 * @param[in] nInliers              内点数目
 * @return cv::Mat                  计算得到的Sim3矩阵
 */
cv::Mat Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;                        // 现在还没有达到最好的效果
    vbInliers = vector<bool>(mN1,false);    // 的确和最初传递给这个解算器的地图点向量是保持一致
    nInliers=0;                             // 存储迭代过程中得到的内点个数

    // Step 1 如果匹配点比要求的最少内点数还少，不满足Sim3 求解条件，返回空
    // mRansacMinInliers 表示RANSAC所需要的最少内点数目
    if(N<mRansacMinInliers)
    {
        bNoMore = true;  // 表示求解失败
        return cv::Mat();   
    }

    // 可以使用的点对的索引,为了避免重复使用
    vector<size_t> vAvailableIndices;

    // 随机选择的来自于这两个帧的三对匹配点
    cv::Mat P3Dc1i(3,3,CV_32F);
    cv::Mat P3Dc2i(3,3,CV_32F);

    // nCurrentIterations：     当前迭代的次数
    // nIterations：            理论迭代次数
    // mnIterations：           总迭代次数
    // mRansacMaxIts：          最大迭代次数
    int nCurrentIterations = 0;
    // Step 2 随机选择三个点，用于求解后面的Sim3
    // 条件1: 已经进行的总迭代次数还没有超过限制的最大总迭代次数
    // 条件2: 当前迭代次数还没有超过理论迭代次数
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;// 这个函数中迭代的次数
        mnIterations++;      // 总的迭代次数，默认为最大为300

        // 记录所有有效（可以采样）的候选三维点索引
        vAvailableIndices = mvAllIndices;

        // Get min set of points
        // Step 2.1 随机取三组点，取完后从候选索引中删掉
        for(short i = 0; i < 3; ++i)
        {
            // DBoW3中的随机数生成函数
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            // P3Dc1i和P3Dc2i中点的排列顺序：
            // x1 x2 x3 ...
            // y1 y2 y3 ...
            // z1 z2 z3 ...
            mvX3Dc1[idx].copyTo(P3Dc1i.col(i));
            mvX3Dc2[idx].copyTo(P3Dc2i.col(i));

            // 从"可用索引列表"中删除这个点的索引 
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // Step 2.2 根据随机取的两组匹配的3D点，计算P3Dc2i 到 P3Dc1i 的Sim3变换
        ComputeSim3(P3Dc1i,P3Dc2i);

        // Step 2.3 对计算的Sim3变换，通过投影误差进行inlier检测
        CheckInliers();

        // Step 2.4 记录并更新最多的内点数目及对应的参数
        if(mnInliersi>=mnBestInliers)
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i.clone();
            mBestRotation = mR12i.clone();
            mBestTranslation = mt12i.clone();
            mBestScale = ms12i;

            if(mnInliersi>mRansacMinInliers) // 只要计算得到一次合格的Sim变换，就直接返回
            {
                // 返回值,告知得到的内点数目
                nInliers = mnInliersi;
                for(int i=0; i<N; i++)
                    if(mvbInliersi[i])
                        // 标记为内点
                        vbInliers[mvnIndices1[i]] = true;
                return mBestT12;
            } // 如果当前次迭代已经合格了,直接返回
        } // 更新最多的内点数目
    } // 迭代循环

    // Step 3 如果已经达到了最大迭代次数了还没得到满足条件的Sim3，说明失败了，放弃，返回
    if(mnIterations>=mRansacMaxIts)
        bNoMore=true;

    return cv::Mat();   // no more的时候返回的是一个空矩阵
}

// 在"进行迭代计算"函数 iterate 的基础上套了一层壳,使用默认参数. 不过目前好像没有被使用到
cv::Mat Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers12,nInliers);
}

 
/**
 * @brief 给出三个点,计算它们的质心以及去质心之后的坐标
 * 
 * @param[in] P     输入的3D点
 * @param[in] Pr    去质心后的点
 * @param[in] C     质心
 */
void Sim3Solver::ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
{
    // 矩阵P每一行求和，结果存在C。这两句也可以使用CV_REDUCE_AVG选项来实现
    cv::reduce(P,C,1,CV_REDUCE_SUM);
    C = C/P.cols;// 求平均

    for(int i=0; i<P.cols; i++)
    {
        Pr.col(i)=P.col(i)-C;//减去质心
    }
}


/**
 * @brief 根据两组匹配的3D点,计算P2到P1的Sim3变换
 * @param[in] P1    匹配的3D点(三个,每个的坐标都是列向量形式,三个点组成了3x3的矩阵)(当前关键帧)
 * @param[in] P2    匹配的3D点(闭环关键帧)
 */
void Sim3Solver::ComputeSim3(cv::Mat &P1, cv::Mat &P2)
{
    // Sim3计算过程参考论文:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    // Step 1: 定义3D点质心及去质心后的点
    // O1和O2分别为P1和P2矩阵中3D点的质心
    // Pr1和Pr2为减去质心后的3D点
    cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
    cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
    cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
    cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

    ComputeCentroid(P1,Pr1,O1);
    ComputeCentroid(P2,Pr2,O2);

    // Step 2: 计算论文中三维点数目n>3的 M 矩阵。这里只使用了3个点
    // Pr2 对应论文中 r_l,i'，Pr1 对应论文中 r_r,i',计算的是P2到P1的Sim3，论文中是left 到 right的Sim3
    cv::Mat M = Pr2*Pr1.t();

    // Step 3: 计算论文中的 N 矩阵

    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    cv::Mat N(4,4,P1.type());

    N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);   // Sxx+Syy+Szz
    N12 = M.at<float>(1,2)-M.at<float>(2,1);                    // Syz-Szy
    N13 = M.at<float>(2,0)-M.at<float>(0,2);                    // Szx-Sxz
    N14 = M.at<float>(0,1)-M.at<float>(1,0);                    // ...
    N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
    N23 = M.at<float>(0,1)+M.at<float>(1,0);
    N24 = M.at<float>(2,0)+M.at<float>(0,2);
    N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
    N34 = M.at<float>(1,2)+M.at<float>(2,1);
    N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

    N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                 N12, N22, N23, N24,
                                 N13, N23, N33, N34,
                                 N14, N24, N34, N44);


    // Step 4: 特征值分解求最大特征值对应的特征向量，就是我们要求的旋转四元数

    cv::Mat eval, evec;  // val vec
    // 特征值默认是从大到小排列，所以evec[0] 是最大值
    cv::eigen(N,eval,evec); 

    // N 矩阵最大特征值（第一个特征值）对应特征向量就是要求的四元数（q0 q1 q2 q3），其中q0 是实部
    // 将(q1 q2 q3)放入vec（四元数的虚部）
    cv::Mat vec(1,3,evec.type());
    (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)


    // Rotation angle. sin is the norm of the imaginary part, cos is the real part
    // 四元数虚部模长 norm(vec)=sin(theta/2), 四元数实部 evec.at<float>(0,0)=q0=cos(theta/2)
    // 这一步的ang实际是theta/2，theta 是旋转向量中旋转角度
    // ? 这里也可以用 arccos(q0)=angle/2 得到旋转角吧
    double ang=atan2(norm(vec),evec.at<float>(0,0));

    // vec/norm(vec)归一化得到归一化后的旋转向量,然后乘上角度得到包含了旋转轴和旋转角信息的旋转向量vec
    vec = 2*ang*vec/norm(vec); //Angle-axis x. quaternion angle is the half

    mR12i.create(3,3,P1.type());
    // 旋转向量（轴角）转换为旋转矩阵
    cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

    // Step 5: Rotate set 2
    // 利用刚计算出来的旋转将三维点旋转到同一个坐标系，P3对应论文里的 r_l,i', Pr1 对应论文里的r_r,i'
    cv::Mat P3 = mR12i*Pr2;

    // Step 6: 计算尺度因子 Scale
    if(!mbFixScale)
    {
        // 论文中有2个求尺度方法。一个是p632右中的位置，考虑了尺度的对称性
        // 代码里实际使用的是另一种方法，这个公式对应着论文中p632左中位置的那个
        // Pr1 对应论文里的r_r,i',P3对应论文里的 r_l,i',(经过坐标系转换的Pr2), n=3, 剩下的就和论文中都一样了
        double nom = Pr1.dot(P3);
        // 准备计算分母
        cv::Mat aux_P3(P3.size(),P3.type());
        aux_P3=P3;
        // 先得到平方
        cv::pow(P3,2,aux_P3);
        double den = 0;

        // 然后再累加
        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<float>(i,j);
            }
        }

        ms12i = nom/den;
    }
    else
        ms12i = 1.0f;

    // Step 7: 计算平移Translation
    mt12i.create(1,3,P1.type());
    // 论文中平移公式
    mt12i = O1 - ms12i*mR12i*O2;

    // Step 8: 计算双向变换矩阵，目的是在后面的检查的过程中能够进行双向的投影操作

    // Step 8.1 用尺度，旋转，平移构建变换矩阵 T12
    mT12i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sR = ms12i*mR12i;

    //         |sR t|
    // mT12i = | 0 1|
    sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
    mt12i.copyTo(mT12i.rowRange(0,3).col(3));

    // Step 8.2 T21

    mT21i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sRinv = (1.0/ms12i)*mR12i.t();
    sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
    cv::Mat tinv = -sRinv*mt12i;
    tinv.copyTo(mT21i.rowRange(0,3).col(3));
}

/**
 * @brief 通过计算的Sim3投影，和自身投影的误差比较，进行内点检测
 * 
 */
void Sim3Solver::CheckInliers()
{
    // 用计算的Sim3 对所有的地图点投影，得到图像点
    vector<cv::Mat> vP1im2, vP2im1;
    Project(mvX3Dc2,vP2im1,mT12i,mK1);// 把2系中的3D经过Sim3变换(mT12i)到1系中计算重投影坐标
    Project(mvX3Dc1,vP1im2,mT21i,mK2);// 把1系中的3D经过Sim3变换(mT21i)到2系中计算重投影坐标

    mnInliersi=0;

    // 对于两帧的每一个匹配点
    for(size_t i=0; i<mvP1im1.size(); i++)
    {
        // 当前关键帧中的地图点直接在当前关键帧图像上的投影坐标mvP1im1，mvP2im2
        // 对于这对匹配关系,在两帧上的投影点距离都要进行计算
        cv::Mat dist1 = mvP1im1[i]-vP2im1[i];
        cv::Mat dist2 = vP1im2[i]-mvP2im2[i];

        // 取距离的平方作为误差
        const float err1 = dist1.dot(dist1);
        const float err2 = dist2.dot(dist2);

        // 根据之前确定的这个最大容许误差来确定这对匹配点是否是外点
        if(err1<mvnMaxError1[i] && err2<mvnMaxError2[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
            mvbInliersi[i]=false;
    }// 遍历其中的每一对匹配点
}

// 得到计算的旋转矩阵
cv::Mat Sim3Solver::GetEstimatedRotation()
{
    return mBestRotation.clone();
}

// 得到计算的平移向量
cv::Mat Sim3Solver::GetEstimatedTranslation()
{
    return mBestTranslation.clone();
}
// 得到估计的从候选帧到当前帧的变换尺度
float Sim3Solver::GetEstimatedScale()
{
    return mBestScale;
}

/**
 * @brief 按照给定的Sim3变换进行投影操作,得到三维点的2D投影点
 * 
 * @param[in] vP3Dw         3D点
 * @param[in & out] vP2D    投影到图像的2D点
 * @param[in] Tcw           Sim3变换
 * @param[in] K             内参
 */
void Sim3Solver::Project(const vector<cv::Mat> &vP3Dw, vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K)
{
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    // 对每个3D地图点进行投影操作
    for(size_t i=0, iend=vP3Dw.size(); i<iend; i++)
    {
        // 首先将对方关键帧的地图点坐标转换到这个关键帧的相机坐标系下
        cv::Mat P3Dc = Rcw*vP3Dw[i]+tcw;
        // 投影
        const float invz = 1/(P3Dc.at<float>(2));
        const float x = P3Dc.at<float>(0)*invz;
        const float y = P3Dc.at<float>(1)*invz;

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

/**
 * @brief 计算当前关键帧中的地图点在当前关键帧图像上的投影坐标
 * 
 * @param[in] vP3Dc         相机坐标系下三维点坐标
 * @param[in] vP2D          投影的二维图像坐标
 * @param[in] K             内参矩阵
 */
void Sim3Solver::FromCameraToImage(const vector<cv::Mat> &vP3Dc, vector<cv::Mat> &vP2D, cv::Mat K)
{
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dc.size());

    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)
    {
        const float invz = 1/(vP3Dc[i].at<float>(2));
        const float x = vP3Dc[i].at<float>(0)*invz;
        const float y = vP3Dc[i].at<float>(1)*invz;

        // 图像上的u,v坐标
        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

} //namespace ORB_SLAM
