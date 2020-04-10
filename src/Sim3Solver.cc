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

// 构造函数
Sim3Solver::Sim3Solver(KeyFrame *pKF1, KeyFrame *pKF2, const vector<MapPoint *> &vpMatched12, const bool bFixScale):
    mnIterations(0), mnBestInliers(0), mbFixScale(bFixScale)
{
    mpKF1 = pKF1;       // 当前关键帧
    mpKF2 = pKF2;       // 闭环关键帧

    // 当前关键帧中的所有地图点
    vector<MapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

    // 这个就认为是已经和当前关键帧中有匹配关系的,闭环关键帧的地图点吧
    mN1 = vpMatched12.size();

    // 准备工作
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
    // mN1为pKF2特征点的个数
    for(int i1=0; i1<mN1; i1++)
    {
        // 如果该特征点在pKF1中有匹配
        if(vpMatched12[i1])
        {
            // pMP1和pMP2是匹配的MapPoint
            MapPoint* pMP1 = vpKeyFrameMP1[i1];
            MapPoint* pMP2 = vpMatched12[i1];

            if(!pMP1)
                continue;

            if(pMP1->isBad() || pMP2->isBad())
                continue;

            // indexKF1和indexKF2是匹配特征点的索引
            int indexKF1 = pMP1->GetIndexInKeyFrame(pKF1);
            int indexKF2 = pMP2->GetIndexInKeyFrame(pKF2);

            if(indexKF1<0 || indexKF2<0)
                continue;

            // kp1和kp2是匹配特征点
            const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[indexKF2];

            const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];

            //? 这个数值9.210是怎么确定的
            //? 以及这个计算出来的数值最后将会用在什么地方? 内点的验证?
            mvnMaxError1.push_back(9.210*sigmaSquare1);
            mvnMaxError2.push_back(9.210*sigmaSquare2);

            // mvpMapPoints1和mvpMapPoints2是匹配的MapPoints容器
            mvpMapPoints1.push_back(pMP1);
            mvpMapPoints2.push_back(pMP2);
            mvnIndices1.push_back(i1);

            // 计算这对匹配地图点分别在各自相机坐标系下的坐标
            cv::Mat X3D1w = pMP1->GetWorldPos();
            mvX3Dc1.push_back(Rcw1*X3D1w+tcw1);

            cv::Mat X3D2w = pMP2->GetWorldPos();
            mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);

            mvAllIndices.push_back(idx);
            idx++;
        }
    } // 处理可以匹配的地图

    mK1 = pKF1->mK;
    mK2 = pKF2->mK;

    // 将两个地图点们分别投影到各自相机的图像中,得到其投影坐标
    // 我觉得进行这一步的目的,应该是在局部优化或者全局优化后,地图点的位置也好,关键帧的位姿也好都会发生变化,为了能够利用上之前优化之后的关键帧位姿和地图点位置,这里就
    // 进行了这样的一个操作
    FromCameraToImage(mvX3Dc1,mvP1im1,mK1);
    FromCameraToImage(mvX3Dc2,mvP2im2,mK2);

    // 设置默认的RANSAC参数,避免在调用的时候因为忘记设置导致崩溃
    SetRansacParameters();
}

// 设置进行RANSAC时的参数
void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations)
{
    mRansacProb = probability;              // 在当前这些点的匹配关系中,一次采样下面的这么多点(和内点个数相同)时,这些点恰好都是内点的概率
    mRansacMinInliers = minInliers;         // 退出RANSAC所需要的最少内点个数;为了方便下面程序中的理解,这里也可以看做是,我们认为的,这堆匹配关系中包含的真正的内点个数
    mRansacMaxIts = maxIterations;          // 最大迭代次数

    // 可靠的匹配点的数目
    N = mvpMapPoints1.size(); // number of correspondences

    // 内点标记
    mvbInliersi.resize(N);

    // Adjust Parameters according to number of correspondences
    float epsilon = (float)mRansacMinInliers/N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations 

    /**
     * \n 这里其实就是计算进行RANSAC时,迭代次数的理论值
     * \n 为了方便描述这里的变量意义还是和程序中保持一致(虽然会使得数学公式看上去很臃肿).
     * \n epsilon 表示了在这 N 对匹配点中,我随便抽取一对点是内点的概率; 
     * \n 为了计算Sim3,我们需要从这N对匹配点中取三对点;那么如果我从这些点中抽取三对点,不严谨地我们认为三对点是可以重复的,那么取这三对点均为内点的概率是 p0=epsilon^3
     * \n (上面说不严谨是因为实际上我们是要取若干对不同的点来计算sim3,这几对匹配点肯定不能够是相同的;但是毕竟我们用于计算sim3的点的对数不是非常多,相比N来说特别小.所以可以
     *    不严谨地认为,前面抽取的点不影响后面抽取点的为内点的概率)
     * \n 相应地,如果取三对点中至少存在一对匹配点是外点, 概率为p1=1-p0
     * \n 而我们RANSAC的过程中不会说只取一次,只做一次这种采样;当我们进行K次采样的时候,其中每一次采样中三对点中都存在至少一对外点的概率就是p2=p1^k
     * \n so我们可以比较轻松得出,K次采样中,至少有一次采样中三对点都是内点的概率是p=1-p2 (不要被这里频繁出现的"至少"搞晕了)
     * \n 这个时候根据 p2=p1^K 我们就可以导出 K 的公式:
     * \n \f$ K=\frac{\log p2}{\log p1}=\frac{\log(1-p)}{\log(1-epsilon^3)} \f$ 
     * \n 这意味着一般来讲,我们进行K次采样,其中至少有一次采样中,三对点都是内点; 因此我们就得到了RANSAC迭代次数的理论值
     * \n 但是这里程序中有个问题:p我们是不知道的! 然后程序中选择直接从外部给定,就是mRansacProb ... 所以我觉得虽然有理论支持,但是这里的理论值还是不准确的
     */

    // 迭代次数的理论值
    int nIterations;

    if(mRansacMinInliers==N)        // 这种情况的时候最后计算得到的迭代次数的确就是一次
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon,3)));   // 计算理论值. 保险的话向上取整.

    /**
     * \n 最后的决策过程说明:
     * \n 外层的max保证RANSAC能够最少迭代一次;
     * \n 内存的min的目的是,如果理论值比给定值要小,那么我们优先选择使用较少的理论值来节省时间(其实也有极大概率得到能够达到的最好结果);
     * \n 如果理论值比给定值要大,那么我们也还是有限选择使用较少的给定值来节省时间;这种情况下是考虑到可能我们的上层应用不需要一个特别优秀的结果,差不多就可以了,但是
     *    希望能够尽快把这个差不多的结果给计算出来,so就可以使用较少的给定值,来限制RANSAC计算的时间
     * \n 当然后者这种情况,更多的时候是给的时候就是瞎给的,根本就没有什么目的性,然后这里碰巧就选择了那个比较小的值 
     */
    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    // 当前正在进行的迭代次数
    mnIterations = 0;
}

// Ransac求解mvX3Dc1和mvX3Dc2之间Sim3，函数返回mvX3Dc2到mvX3Dc1的Sim3变换
cv::Mat Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;                        // 现在还没有达到最好的效果
    vbInliers = vector<bool>(mN1,false);    // 的确和最初传递给这个解算器的地图点向量是保持一致
    nInliers=0;                             // 存储迭代过程中得到的内点个数

    // 如果经过"处理"后的点的数目已经比那个要求的最小点的数据少了,那么就说明...我们已经没有更好的选择了
    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();   // 表示求解失败
    }

    // 可以使用的点对的索引,为了避免重复使用
    vector<size_t> vAvailableIndices;

    // 来自于这两个帧的三对匹配点
    cv::Mat P3Dc1i(3,3,CV_32F);
    cv::Mat P3Dc2i(3,3,CV_32F);

    // 这个函数中迭代的次数
    int nCurrentIterations = 0;
    // 条件1: 还没有超过限制的最大迭代次数
    // 条件2: nCurrentIterations  还没有超过给定的迭代次数
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;// 这个函数中迭代的次数
        mnIterations++;      // 总的迭代次数，默认为最大为300

        vAvailableIndices = mvAllIndices;

        // Get min set of points
        // STEP 1：任意取三组点算Sim矩阵
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

            // 从"可用索引列表"中删除这个点
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // STEP 2：根据两组匹配的3D点，计算之间的Sim3变换
        ComputeSim3(P3Dc1i,P3Dc2i);

        // STEP 3：通过投影误差进行inlier检测
        CheckInliers();

        // 更新最多的内点数目
        if(mnInliersi>=mnBestInliers)
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i.clone();
            mBestRotation = mR12i.clone();
            mBestTranslation = mt12i.clone();
            mBestScale = ms12i;

            if(mnInliersi>mRansacMinInliers)// 只要计算得到一次合格的Sim变换，就直接返回
            {
                // 返回值,告知得到的内点数目
                nInliers = mnInliersi;
                for(int i=0; i<N; i++)
                    if(mvbInliersi[i])
                        // 用这种方式将"处理后的地图点向量坐标"转换成为"处理前的地图点向量坐标"
                        vbInliers[mvnIndices1[i]] = true;
                return mBestT12;
            } // 如果当前次迭代已经合格了,直接返回
        } // 更新最多的内点数目
    } // 迭代循环

    // 如果已经达到了设计的最大迭代次数,就no more 了
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

// 给出三个点,计算它们的质心以及去质心之后的坐标
void Sim3Solver::ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
{
    // 这两句可以使用CV_REDUCE_AVG选项来搞定
    cv::reduce(P,C,1,CV_REDUCE_SUM);// 矩阵P每一行求和
    C = C/P.cols;// 求平均

    for(int i=0; i<P.cols; i++)
    {
        Pr.col(i)=P.col(i)-C;//减去质心
    }
}

// 根据两组匹配的3D点,计算之间的Sim3变换
// 三对匹配点,每个点的坐标都是列向量形式,三个点组成了3x3的矩阵,三对点组成了两个3x3矩阵P1,P2
void Sim3Solver::ComputeSim3(cv::Mat &P1, cv::Mat &P2)
{
    // ！！！！！！！这段代码一定要看这篇论文！！！！！！！！！！！
    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    // Step 1: Centroid and relative coordinates

    cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
    cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
    cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
    cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

    // O1和O2分别为P1和P2矩阵中3D点的质心
    // Pr1和Pr2为减去质心后的3D点
    ComputeCentroid(P1,Pr1,O1);
    ComputeCentroid(P2,Pr2,O2);

    // Step 2: Compute M matrix
    // 这里是按照"大于三组匹配点"中的M矩阵来计算的;形式和论文中略有不同,但是本质上是一样的
    cv::Mat M = Pr2*Pr1.t();

    // Step 3: Compute N matrix

    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    cv::Mat N(4,4,P1.type());

    N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
    N12 = M.at<float>(1,2)-M.at<float>(2,1);
    N13 = M.at<float>(2,0)-M.at<float>(0,2);
    N14 = M.at<float>(0,1)-M.at<float>(1,0);
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


    // Step 4: Eigenvector of the highest eigenvalue

    cv::Mat eval, evec;  // val vec

    cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

    /**
     * \n 补充一下这里计算四元数的时候用到的一些技巧:
     * \n 对于四元数 p=(p0,p1i,p2j,p3k), 其中的三个虚部和空间中的三个轴相对应:
     * \n \f$ p=\cos(\theta/2)+\mathbf{n}\sin(\theta/2) \f$
     * \n 可以非常方便地写成旋转向量的表达方式,上式中的旋转向量就是n;一般地旋转向量模的大小表示了旋转的角度theta的大小,但是这里的n只能够表示旋转的方向
     * \n so我们只需要得到了旋转向量即可恢复出欧拉角.其中 
     * \n \f$ \mathbf{n}\sin(\theta/2)
     * 
     */

    // N矩阵最大特征值（第一个特征值）对应特征向量就是要求的四元数（q0 q1 q2 q3） (第0行,其中q0就是我们日常说的w)
    // 将(q1 q2 q3)放入vec行向量，vec就是四元数旋转轴乘以sin(angle/2)
    cv::Mat vec(1,3,evec.type());
    (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

    // Rotation angle. sin is the norm of the imaginary part, cos is the real part
    // 这里是这样来的:将四元数转换成为复数的形式,计算虚部(模长)和实部的夹角
    // 这里的 norm(vec)=sin(theta/2), evec.at<float>(0,0)=q0=cos(theta/2)
    // ? 为什么不 arccos(w)=angle/2呢???? 
    double ang=atan2(norm(vec),evec.at<float>(0,0));

    // 虚部表示旋转向量,归一化得到归一化后的旋转向量,然后乘上角度得到包含了旋转轴和旋转角信息的旋转向量.
    vec = 2*ang*vec/norm(vec); //Angle-axis x. quaternion angle is the half

    mR12i.create(3,3,P1.type());

    cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

    // Step 5: Rotate set 2
    // 要放到同一个坐标系下进行计算
    cv::Mat P3 = mR12i*Pr2;

    // Step 6: Scale

    if(!mbFixScale)
    {
        // 论文中还有一个求尺度的公式，p632右中的位置，那个公式不用考虑旋转
        // 这个公式对应着论文中p632左中位置的那个,r对应着Pr1,l对应着P3(经过坐标系转换的Pr2),剩下的就和论文中都一样了
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

    // Step 7: Translation

    mt12i.create(1,3,P1.type());
    // 论文中公式
    mt12i = O1 - ms12i*mR12i*O2;

    // Step 8: Transformation
    // 计算双向的位姿变换,目的是在下面的检查的过程中能够进行双向的投影操作

    // Step 8.1 T12
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

// 通过投影误差进行内点检测
void Sim3Solver::CheckInliers()
{
    vector<cv::Mat> vP1im2, vP2im1;
    Project(mvX3Dc2,vP2im1,mT12i,mK1);// 把2系中的3D经过Sim3变换(mT12i)到1系中计算重投影坐标
    Project(mvX3Dc1,vP1im2,mT21i,mK2);// 把1系中的3D经过Sim3变换(mT21i)到2系中计算重投影坐标

    mnInliersi=0;

    // 对于两帧的每一个匹配点
    for(size_t i=0; i<mvP1im1.size(); i++)
    {
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

// 按照给定的Sim3变换进行投影操作,得到三维点的2D投影点
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

// 计算当前关键帧中的地图点在当前关键帧图像上的投影坐标
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

        // 非齐次坐标
        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

} //namespace ORB_SLAM
