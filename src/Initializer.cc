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
 * @file Initializer.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 单目初始化的实现文件
 * @version 0.1
 * @date 2019-01-11
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "Initializer.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include "Optimizer.h"          //不太明白为什么这个头文件的包含在vscode中会报错
#include "ORBmatcher.h"

//这里使用到了多线程的加速技术
#include<thread>

namespace ORB_SLAM2
{



/**
 * @brief 根据参考帧构造初始化器
 * 
 * @param[in] ReferenceFrame        参考帧
 * @param[in] sigma                 测量误差
 * @param[in] iterations            RANSAC迭代次数
 */
Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
{
	//从参考帧中获取相机的内参数矩阵
    mK = ReferenceFrame.mK.clone();
	
	// 从参考帧中获取去畸变后的特征点
    mvKeys1 = ReferenceFrame.mvKeysUn;
	
	//获取估计误差
    mSigma = sigma;
    mSigma2 = sigma*sigma;

	//最大迭代次数
    mMaxIterations = iterations;
}


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
bool Initializer::Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                             vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
{
    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2

    //获取当前帧的去畸变之后的特征点
    mvKeys2 = CurrentFrame.mvKeysUn;

    // mvMatches12记录匹配上的特征点对，记录的是帧2在帧1的匹配索引
    mvMatches12.clear();
	// 预分配空间，大小和关键点数目一致mvKeys2.size()
    mvMatches12.reserve(mvKeys2.size());

    // 记录参考帧1中的每个特征点是否有匹配的特征点
    // 这个成员变量后面没有用到，后面只关心匹配上的特征点 	
    mvbMatched1.resize(mvKeys1.size());

    // Step 1 重新记录特征点对的匹配关系存储在mvMatches12，是否有匹配存储在mvbMatched1
    // 将vMatches12（有冗余） 转化为 mvMatches12（只记录了匹配关系）
    for(size_t i=0, iend=vMatches12.size();i<iend; i++)
    {
		//vMatches12[i]解释：i表示帧1中关键点的索引值，vMatches12[i]的值为帧2的关键点索引值
        //没有匹配关系的话，vMatches12[i]值为 -1
        if(vMatches12[i]>=0)
        {
			//mvMatches12 中只记录有匹配关系的特征点对的索引值
            //i表示帧1中关键点的索引值，vMatches12[i]的值为帧2的关键点索引值
            mvMatches12.push_back(make_pair(i,vMatches12[i]));
			//标记参考帧1中的这个特征点有匹配关系
            mvbMatched1[i]=true;
        }
        else
			//标记参考帧1中的这个特征点没有匹配关系
            mvbMatched1[i]=false;
    }

    // 有匹配的特征点的对数
    const int N = mvMatches12.size();
    // Indices for minimum set selection
    // 新建一个容器vAllIndices存储特征点索引，并预分配空间
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);

	//在RANSAC的某次迭代中，还可以被抽取来作为数据样本的特征点对的索引，所以这里起的名字叫做可用的索引
    vector<size_t> vAvailableIndices;
	//初始化所有特征点对的索引，索引值0到N-1
    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    // Step 2 在所有匹配特征点对中随机选择8对匹配特征点为一组，用于估计H矩阵和F矩阵
    // 共选择 mMaxIterations (默认200) 组
    //mvSets用于保存每次迭代时所使用的向量
    mvSets = vector< vector<size_t> >(mMaxIterations,		//最大的RANSAC迭代次数
									  vector<size_t>(8,0));	//这个则是第二维元素的初始值，也就是第一维。这里其实也是一个第一维的构造函数，第一维vector有8项，每项的初始值为0.

	//用于进行随机数据样本采样，设置随机数种子
    DUtils::Random::SeedRandOnce(0);

	//开始每一次的迭代 
    for(int it=0; it<mMaxIterations; it++)
    {
		//迭代开始的时候，所有的点都是可用的
        vAvailableIndices = vAllIndices;

        // Select a minimum set
		//选择最小的数据样本集，使用八点法求，所以这里就循环了八次
        for(size_t j=0; j<8; j++)
        {
            // 随机产生一对点的id,范围从0到N-1
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            // idx表示哪一个索引对应的特征点对被选中
            int idx = vAvailableIndices[randi];
			
			//将本次迭代这个选中的第j个特征点对的索引添加到mvSets中
            mvSets[it][j] = idx;

            // 由于这对点在本次迭代中已经被使用了,所以我们为了避免再次抽到这个点,就在"点的可选列表"中,
            // 将这个点原来所在的位置用vector最后一个元素的信息覆盖,并且删除尾部的元素
            // 这样就相当于将这个点的信息从"点的可用列表"中直接删除了
            vAvailableIndices[randi] = vAvailableIndices.back();
			vAvailableIndices.pop_back();
        }//依次提取出8个特征点对
    }//迭代mMaxIterations次，选取各自迭代时需要用到的最小数据集

    

    // Launch threads to compute in parallel a fundamental matrix and a homography
    // Step 3 计算fundamental 矩阵 和homography 矩阵，为了加速分别开了线程计算 
 
    //这两个变量用于标记在H和F的计算中哪些特征点对被认为是Inlier
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
	//计算出来的单应矩阵和基础矩阵的RANSAC评分，这里其实是采用重投影误差来计算的
    float SH, SF; //score for H and F
    //这两个是经过RANSAC算法后计算出来的单应矩阵和基础矩阵
    cv::Mat H, F; 

    // 构造线程来计算H矩阵及其得分
    // thread方法比较特殊，在传递引用的时候，外层需要用ref来进行引用传递，否则就是浅拷贝
    thread threadH(&Initializer::FindHomography,	//该线程的主函数
				   this,							//由于主函数为类的成员函数，所以第一个参数就应该是当前对象的this指针
				   ref(vbMatchesInliersH), 			//输出，特征点对的Inlier标记
				   ref(SH), 						//输出，计算的单应矩阵的RANSAC评分
				   ref(H));							//输出，计算的单应矩阵结果
    // 计算fundamental matrix并打分，参数定义和H是一样的，这里不再赘述
    thread threadF(&Initializer::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));
    // Wait until both threads have finished
	//等待两个计算线程结束
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    // Step 4 计算得分比例来判断选取哪个模型来求位姿R,t
	//通过这个规则来判断谁的评分占比更多一些，注意不是简单的比较绝对评分大小，而是看评分的占比
    float RH = SH/(SH+SF);			//RH=Ratio of Homography

    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    // 注意这里更倾向于用H矩阵恢复位姿。如果单应矩阵的评分占比达到了0.4以上,则从单应矩阵恢复运动,否则从基础矩阵恢复运动
    if(RH>0.40)
		//更偏向于平面，此时从单应矩阵恢复，函数ReconstructH返回bool型结果
        return ReconstructH(vbMatchesInliersH,	//输入，匹配成功的特征点对Inliers标记
							H,					//输入，前面RANSAC计算后的单应矩阵
							mK,					//输入，相机的内参数矩阵
							R21,t21,			//输出，计算出来的相机从参考帧1到当前帧2所发生的旋转和位移变换
							vP3D,				//特征点对经过三角测量之后的空间坐标，也就是地图点
							vbTriangulated,		//特征点对是否成功三角化的标记
							1.0,				//这个对应的形参为minParallax，即认为某对特征点的三角化测量中，认为其测量有效时
												//需要满足的最小视差角（如果视差角过小则会引起非常大的观测误差）,单位是角度
							50);				//为了进行运动恢复，所需要的最少的三角化测量成功的点个数
    else //if(pF_HF>0.6)
        // 更偏向于非平面，从基础矩阵恢复
        return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,1.0,50);

	//一般地程序不应该执行到这里，如果执行到这里说明程序跑飞了
    return false;
}

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
void Initializer::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
{
    // Number of putative matches
	//匹配的特征点对总数
    const int N = mvMatches12.size();

    // Normalize coordinates
    // Step 1 将当前帧和参考帧中的特征点坐标进行归一化，主要是平移和尺度变换
    // 具体来说,就是将mvKeys1和mvKey2归一化到均值为0，一阶绝对矩为1，归一化矩阵分别为T1、T2
    // 这里所谓的一阶绝对矩其实就是随机变量到取值的中心的绝对值的平均值
    // 归一化矩阵就是把上述归一化的操作用矩阵来表示。这样特征点坐标乘归一化矩阵可以得到归一化后的坐标
   

	//归一化后的参考帧1和当前帧2中的特征点坐标
    vector<cv::Point2f> vPn1, vPn2;
	// 记录各自的归一化矩阵
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);

	//这里求的逆在后面的代码中要用到，辅助进行原始尺度的恢复
    cv::Mat T2inv = T2.inv();

    // Best Results variables
	// 记录最佳评分
    score = 0.0;
	// 取得历史最佳评分时,特征点对的inliers标记
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
	//某次迭代中，参考帧的特征点坐标
    vector<cv::Point2f> vPn1i(8);
	//某次迭代中，当前帧的特征点坐标
    vector<cv::Point2f> vPn2i(8);
	//以及计算出来的单应矩阵、及其逆矩阵
    cv::Mat H21i, H12i;

    // 每次RANSAC记录Inliers与得分
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
	//下面进行每次的RANSAC迭代
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
		// Step 2 选择8个归一化之后的点对进行迭代
        for(size_t j=0; j<8; j++)
        {
			//从mvSets中获取当前次迭代的某个特征点对的索引信息
            int idx = mvSets[it][j];

            // vPn1i和vPn2i为匹配的特征点对的归一化后的坐标
			// 首先根据这个特征点对的索引信息分别找到两个特征点在各自图像特征点向量中的索引，然后读取其归一化之后的特征点坐标
            vPn1i[j] = vPn1[mvMatches12[idx].first];    //first存储在参考帧1中的特征点索引
            vPn2i[j] = vPn2[mvMatches12[idx].second];   //second存储在参考帧1中的特征点索引
        }//读取8对特征点的归一化之后的坐标

		// Step 3 八点法计算单应矩阵
        // 利用生成的8个归一化特征点对, 调用函数 Initializer::ComputeH21() 使用八点法计算单应矩阵  
        // 关于为什么计算之前要对特征点进行归一化，后面又恢复这个矩阵的尺度？
        // 可以在《计算机视觉中的多视图几何》这本书中P193页中找到答案
        // 书中这里说,8点算法成功的关键是在构造解的方称之前应对输入的数据认真进行适当的归一化
   
        cv::Mat Hn = ComputeH21(vPn1i,vPn2i);
        
        // 单应矩阵原理：X2=H21*X1，其中X1,X2 为归一化后的特征点    
        // 特征点归一化：vPn1 = T1 * mvKeys1, vPn2 = T2 * mvKeys2  得到:T2 * mvKeys2 =  Hn * T1 * mvKeys1   
        // 进一步得到:mvKeys2  = T2.inv * Hn * T1 * mvKeys1
        H21i = T2inv*Hn*T1;
		//然后计算逆
        H12i = H21i.inv();

        // Step 4 利用重投影误差为当次RANSAC的结果评分
        currentScore = CheckHomography(H21i, H12i, 			//输入，单应矩阵的计算结果
									   vbCurrentInliers, 	//输出，特征点对的Inliers标记
									   mSigma);				//TODO  测量误差，在Initializer类对象构造的时候，由外部给定的

    
        // Step 5 更新具有最优评分的单应矩阵计算结果,并且保存所对应的特征点对的内点标记
        if(currentScore>score)
        {
			//如果当前的结果得分更高，那么就更新最优计算结果
            H21 = H21i.clone();
			//保存匹配好的特征点对的Inliers标记
            vbMatchesInliers = vbCurrentInliers;
			//更新历史最优评分
            score = currentScore;
        }
    }
}

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
void Initializer::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
{
    // 计算基础矩阵,其过程和上面的计算单应矩阵的过程十分相似.

    // Number of putative matches
	// 匹配的特征点对总数
    const int N = vbMatchesInliers.size();

    // Normalize coordinates
    // Step 1 将当前帧和参考帧中的特征点坐标进行归一化，主要是平移和尺度变换
    // 具体来说,就是将mvKeys1和mvKey2归一化到均值为0，一阶绝对矩为1，归一化矩阵分别为T1、T2
    // 这里所谓的一阶绝对矩其实就是随机变量到取值的中心的绝对值的平均值
    // 归一化矩阵就是把上述归一化的操作用矩阵来表示。这样特征点坐标乘归一化矩阵可以得到归一化后的坐标

    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
	// ! 注意这里取的是归一化矩阵T2的转置,因为基础矩阵的定义和单应矩阵不同，两者去归一化的计算也不相同
    cv::Mat T2t = T2.t();

    // Best Results variables
	//最优结果
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
	// 某次迭代中，参考帧的特征点坐标
    vector<cv::Point2f> vPn1i(8);
    // 某次迭代中，当前帧的特征点坐标
    vector<cv::Point2f> vPn2i(8);
    // 某次迭代中，计算的基础矩阵
    cv::Mat F21i;

    // 每次RANSAC记录的Inliers与得分
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    // 下面进行每次的RANSAC迭代
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        // Step 2 选择8个归一化之后的点对进行迭代
        for(int j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            // vPn1i和vPn2i为匹配的特征点对的归一化后的坐标
			// 首先根据这个特征点对的索引信息分别找到两个特征点在各自图像特征点向量中的索引，然后读取其归一化之后的特征点坐标
            vPn1i[j] = vPn1[mvMatches12[idx].first];        //first存储在参考帧1中的特征点索引
            vPn2i[j] = vPn2[mvMatches12[idx].second];       //second存储在参考帧1中的特征点索引
        }

        // Step 3 八点法计算基础矩阵
        cv::Mat Fn = ComputeF21(vPn1i,vPn2i);
        F21i = T2t*Fn*T1;

        // Step 4 利用重投影误差为当次RANSAC的结果评分
        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

		// Step 5 更新具有最优评分的基础矩阵计算结果,并且保存所对应的特征点对的内点标记
        if(currentScore>score)
        {
            //如果当前的结果得分更高，那么就更新最优计算结果
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}



// |x'|     | h1 h2 h3 ||x|
// |y'| = a | h4 h5 h6 ||y|  简写: x' = a H x, a为一个尺度因子
// |1 |     | h7 h8 h9 ||1|
// 使用DLT(direct linear tranform)求解该模型
// x' = a H x 
// ---> (x') 叉乘 (H x)  = 0  (因为方向相同) (取前两行就可以推导出下面的了)
// ---> Ah = 0 
// A = | 0  0  0 -x -y -1 xy' yy' y'|  h = | h1 h2 h3 h4 h5 h6 h7 h8 h9 |
//     |-x -y -1  0  0  0 xx' yx' x'|
// 通过SVD求解Ah = 0，A^T*A最小特征值对应的特征向量即为解
// 其实也就是右奇异值矩阵的最后一列

//从特征点匹配求homography（normalized DLT），其实这里最少用四对点就能够求出来，不过这里为了和基础矩阵统一还是使用了8对点求最小二乘解
/** @see Initializer::ComputeF21() */
cv::Mat Initializer::ComputeH21(
    const vector<cv::Point2f> &vP1, //归一化后的点, in reference frame
    const vector<cv::Point2f> &vP2) //归一化后的点, in current frame
{
    /** 该函数使用了较多的数学知识, 介绍如下. 
     * \n 这里使用了 DLT 直接分解方法来求单应矩阵. 有两种方法都可以推导得到单应矩阵的计算形式,先介绍第一种.
     * \n 单应矩阵的定义可列写如下:
     * \n \f$  \begin{bmatrix} x'\\y'\\1 \end{bmatrix} = a \begin{bmatrix} h_1&h_2&h_3\\h_4&h_5&h_6\\h_7&h_8&h_9 \end{bmatrix}
     * \begin{bmatrix} x\\y\\1 \end{bmatrix} \f$
     * \n 其中\f$ a \f$ 是一个缩放系数. 上式可以写成矩阵形式:
     * \n \f$ \mathbf{x}'=a\mathbf{H}\mathbf{x}  \f$
     * \n 可以发现 \f$ \mathbf{x}' \f$和\f$ \mathbf{H}\mathbf{x} \f$的方向相同,所以他们的叉乘为0:
     * \n \f$ \mathbf{x}' \times \mathbf{H}\mathbf{x}  \f$
     * \n 使用 DLT 直接分解法求解. 上式可展开为:
     * \n \f$ \begin{bmatrix} x'\\y'\\1 \end{bmatrix} \times \begin{bmatrix} h_1&h_2&h_3\\h_4&h_5&h_6\\h_7&h_8&h_9 \end{bmatrix}
     * \begin{bmatrix} x\\y\\1 \end{bmatrix} =0 \f$
     * \n \f$ \begin{bmatrix} 0&-1&y'\\1&0&-x'\\-y'&x'&0 \end{bmatrix} \cdot 
     * \begin{bmatrix} xh_1+yh_2+h_3\\xh_4+yh_5+h_6\\xh_7+yh_8+h_9 \end{bmatrix} = 0 \f$
     * \n 对于前两行,展开有:
     * \n \f$  \begin{cases} -xh_4-yh_5-h_6+xy'h_7+yy'h_8+y'h_9=0\\ xh_1+yh_2+h_3-xx'h_7-x'yh_8-x'h_9=0 \end{cases} \f$
     * \n 写成矩阵形式,即
     * \n \f$  \begin{bmatrix} 0&0&0&-x&-y&-1&xy'&yy'&y'\\ x&y&1&0&0&0&-xx'&-x'y&-x' \end{bmatrix} \cdot 
     * \begin{bmatrix} h_1\\h_2\\h_3\\h_4\\h_5\\h_6\\h_7\\h_8\\h_9 \end{bmatrix} =0 \f$
     * \n 即得到了一个齐次方程组的形式,写成矩阵形式为:
     * \n \f$ \mathbf{A}\cdot\mathbf{h}=0  \f$
     * \n 这个形式就可以在后面的过程中使用奇异值分解的方法来求解.
     */

    /** 另外一种求法:
     * \n 同样地, 对于单应矩阵的定义, 缩放系数 \f$ a \neq 0 \f$ 如果不加的话,那么一般地单应矩阵中的 \f$ h_9 \neq =1\f$ 
     * \n \f$ \begin{bmatrix} x'\\y'\\1 \end{bmatrix} = 
     * \begin{bmatrix} h_1&h_2&h_3\\h_4&h_5&h_6\\h_7&h_8&h_9 \end{bmatrix}  \begin{bmatrix} x\\y\\1 \end{bmatrix} \f$
     * \n 前两行除第三行:
     * \n \f$  x'=(h_1x+h_2y+h_3)/(h_7x+h_8y+h_9) \f$
     * \n \f$  y'=(h_4x+h_5y+h_6)/(h_7x+h_8y+h_9) \f$
     * \n 然后整理,进行移项,有
     * \n \f$  \begin{cases} xh_1+yh_2+h_3-x'(xh_7+yh_8+h_9)=0\\ xh_4+yh_5+h_6-y'(xh_7+y_h8+h_9)=0 \end{cases} \f$
     * \n 然后就可以导出和上面相同的矩阵形式. 
     */
    /** @see 视觉SLAM十四讲P147  */

    /** 最后,为了求解这个齐次方程组,利用一个现成的结论: 只需要对A进行奇异值分解, 其最小奇异值所对应的右奇异向量就是这个方程组的近似解. */ 

    /** 计算步骤如下: <ul> */
	//获取参与计算的特征点的数目
    const int N = vP1.size();

    /** <li> 构造用于计算的矩阵 A </li> */
    cv::Mat A(2*N,				//行，注意每一个点的数据对应两行
			  9,				//列
			  CV_32F); // 2N*9	//数据类型

	//开始构造矩阵A，对于每一个点
    for(int i=0; i<N; i++)
    {
		//获取特征点对的像素坐标
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

		//生成这个点的第一行
        A.at<float>(2*i,0) = 0.0;
        A.at<float>(2*i,1) = 0.0;
        A.at<float>(2*i,2) = 0.0;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

		//生成这个点的第二行
        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1;
        A.at<float>(2*i+1,3) = 0.0;
        A.at<float>(2*i+1,4) = 0.0;
        A.at<float>(2*i+1,5) = 0.0;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;

    }//对于每一个点构造矩阵A

    //保存计算结果，vt中的t表示是转置
    cv::Mat u,w,vt;

	//
    /** <li> 使用opencv提供的进行奇异值分解的函数 cv::SVDecomp() 求解矩阵A的奇异值分解  </li> */
    cv::SVDecomp(A,							//输入，待进行奇异值分解的矩阵
				 w,							//输出，奇异值矩阵
				 u,							//输出，矩阵U
				 vt,						//输出，矩阵V^T
				 cv::SVD::MODIFY_A | 		//输入，MODIFY_A是指允许计算函数可以修改待分解的矩阵，官方文档上说这样可以加快计算速度、节省内存
				     cv::SVD::FULL_UV);		//FULL_UV=把U和VT补充成单位正交方阵

	/** <li> 然后返回最小奇异值所对应的右奇异向量 </li> */
    //注意前面说的是右奇异值矩阵的最后一列，但是在这里因为是vt，转置后了，所以是行；由于A有9列数据，故最后一列的下标为8
    return vt.row(8).reshape(0, 			//转换后的通道数，这里设置为0表示是与前面相同
							 3); 			//转换后的行数
							// v的最后一列
    /** </ul> */
}//计算单应矩阵H（函数）

// x'Fx = 0 整理可得：Af = 0
// A = | x'x x'y x' y'x y'y y' x y 1 |, f = | f1 f2 f3 f4 f5 f6 f7 f8 f9 |
// 通过SVD求解Af = 0，A'A最小特征值对应的特征向量即为解
//其实也是运用DLT解法，由于点是三维的因此使用DLT法构造出来的A矩阵和前面的都有些不一样；但是其他的操作和上面计算单应矩阵
//都是基本相似的

//从特征点匹配求fundamental matrix（normalized 8点法）
/** @see Initializer::ComputeH21() */
cv::Mat Initializer::ComputeF21(
    const vector<cv::Point2f> &vP1, //归一化后的点, in reference frame
    const vector<cv::Point2f> &vP2) //归一化后的点, in current frame
{
    /** 计算基础矩阵. 其定义是:
     * \f$ \mathbf{x}^{\text{T}}\mathbf{F}\mathbf{x}=0  \f$
     * \n 所以和 Initializer::ComputeH21() 类似的思路, 使用 DLT 直接线性分解得到:
     * \n \f$  \begin{bmatrix} x'\\y'\\1 \end{bmatrix} =
     * \begin{bmatrix} f_1&f_2&f3\\ f_4&f_5&f6\\ f_7&f_8&f9\\ \end{bmatrix} \cdot
     * \begin{bmatrix} x\\y\\1 \end{bmatrix} =0  \f$ 
     * \n 展开计算:
     * \n \f$  \begin{bmatrix} x'f_1+y'f_4+f_7&x'f_2+y'f_5+f_8&x'f_3+y'f_6+f_9 \end{bmatrix}
     * \begin{bmatrix} x\\y\\1 \end{bmatrix} = 0 \f$
     * \n \f$  xx'f_1+xy'f_4+xf_7+x'yf_2+yy'f_5+yf_8+x'f_3+y'f6+f_9=0 \f$
     * \n \f$  \begin{bmatrix} xx'&x'y&x'&xy'&yy'&y'&x&y&1 \end{bmatrix} 
     * \begin{bmatrix} f_1\\f_2\\f_3\\f_4\\f_5\\f_6\\f_7\\f_8\\f_9 \end{bmatrix} =0 \f$
     * \n \f$ \mathbf{A}\cdot\mathbf{f}=0 \f$
     * \n 于是又得到了这种形式,可以和计算单应矩阵一样的SVD方法来求解了.
     * \n 计算过程如下: <ul>
     */ 


	//获取参与计算的特征点对数
    const int N = vP1.size();

	//初始化A矩阵
    cv::Mat A(N,9,CV_32F); // N*9

    /** <li> 对于每对特征点，生成A矩阵的内容 </li> */
    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    //存储奇异值分解结果的变量
    cv::Mat u,w,vt;

	
    /** <li> 进行第一次奇异值分解,求解出基础矩阵, 使用 cv::SVDecomp() 函数 </li> */
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
	//转换成基础矩阵的形式
    cv::Mat Fpre = vt.row(8).reshape(0, 3); // v的最后一列

    //NOTICE 注意基础矩阵的定义中，由于乘了一个t^，这个矩阵因为是从三维向量拓展出来的因此它的秩为2
    //所以正确的基础矩阵的秩应当小于等于2，这里就是进行的这个处理
    /** <li> 但是由于基础矩阵的秩为2,而我们不敢保证计算得到的这个结果的秩为2,所以需要通过第二次奇异值分解,来强制使其秩为2 </li> 
     * \n 在基础矩阵的定义中;
     * \n \f$  \mathbf{x}_1^{\text{T}}\mathbf{F}\mathbf{x}_2^{\text{T}}=
     * \mathbf{x}_1^{\text{T}}\mathbf{K}_1^{\text{-T}}[\mathbf{T}]_{\times}\mathbf{R}\mathbf{K}_2^{-1}\mathbf{x}_2^{\text{T}}=0  \f$
     * \n 其中 \f$ \mathbf{x}_1 \f$ 和 \f$ \mathbf{x}_2 \f$ 为两帧图像上的匹配点, \f$ \mathbf{K}_1 \f$ 和 \f$ \mathbf{K}_1 \f$ 为两帧图像的相机内参,
     * \f$ \mathbf{R} \f$ 和 \f$ \mathbf{T} \f$ 是从图像1到 图像2的旋转矩阵和平移矩阵,\f$ [\cdot]_{\times} \f$表示进行矩阵的反对称操作. 
     * 有基础矩阵的另外一种等效定义形式为;
     * \n \f$  \mathbf{F}=\mathbf{K}_1^{\text{-T}}[\mathbf{T}]_{\times}\mathbf{R}\mathbf{K}_2^{-1} \f$
     * \n 注意到: <ul>
     * <li> \f$  \mathbf{R} \f$ 的秩为3 </li>
     * <li> \f$ \mathbf{K}_1  \f$ 的秩为3 </li>
     * <li> \f$  \mathbf{K}_2 \f$ 的秩为3 </li> </ul>
     * 而 \f$ [\mathbf{T}]_{\times} \f$ 的秩为2. 所以最后基础矩阵为上面所有矩阵相乘的结果,其秩为2.
     * 操作过程如下: <ul> 
    */
    /** <li> 对初步得来的基础矩阵继续进行一次奇异值分解 </li> */
    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
	/** <li> 强制将第三个奇异值设置为0 </li> */
    w.at<float>(2)=0; // 秩2约束，将第3个奇异值设为0
    /** <li> 重新组合好满足秩约束的基础矩阵，作为最终计算结果返回  </li> */
    return  u*cv::Mat::diag(w)*vt;
    /** </ul> */
    /** </ul> */
}//计算基础矩阵

//对给定的homography matrix打分,需要使用到卡方检验的知识
float Initializer::CheckHomography(
    const cv::Mat &H21,                 //从参考帧到当前帧的单应矩阵
    const cv::Mat &H12,                 //从当前帧到参考帧的单应矩阵
    vector<bool> &vbMatchesInliers,     //匹配好的特征点对的Inliers标记
    float sigma)                        //估计误差
{

    // 说明：在已值n维观测数据误差服从N(0，sigma）的高斯分布时
    // 其误差加权最小二乘结果为  sum_error = SUM(e(i)^T * Q^(-1) * e(i))
    // 其中：e(i) = [e_x,e_y,...]^T, Q维观测数据协方差矩阵，即sigma * sigma组成的协方差矩阵
    // 误差加权最小二次结果越小，说明观测数据精度越高
    // 那么，score = SUM((th - e(i)^T * Q^(-1) * e(i)))的分数就越高
    // 算法目标： 检查单应变换矩阵
    // 检查方式：通过H矩阵，进行参考帧和当前帧之间的双向投影，并计算起加权最小二乘投影误差

    // 算法流程
    // input: 单应性矩阵 H21, H12, 匹配点集 mvKeys1
    //    do:
    //        for p1(i), p2(i) in mvKeys:
    //           error_i1 = ||p2(i) - H21 * p1(i)||2
    //           error_i2 = ||p1(i) - H12 * p2(i)||2
    //           
    //           w1 = 1 / sigma / sigma
    //           w2 = 1 / sigma / sigma
    // 
    //           if error1 < th
    //              score +=   th - error_i1 * w1
    //           if error2 < th
    //              score +=   th - error_i2 * w2
    // 
    //           if error_1i > th or error_2i > th
    //              p1(i), p2(i) are inner points
    //              vbMatchesInliers(i) = true
    //           else 
    //              p1(i), p2(i) are outliers
    //              vbMatchesInliers(i) = false
    //           end
    //        end
    //   output: score, inliers

	// 特点匹配个数
    const int N = mvMatches12.size();

	// 获取从参考帧到当前帧的单应矩阵的各个元素
    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);

	// 获取从当前帧到参考帧的单应矩阵的各个元素
    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);

	// 给特征点对的Inliers标记预分配空间
    vbMatchesInliers.resize(N);

	// 初始化score值
    float score = 0;

    // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
	// 自由度为2的卡方分布，误差项有95%的概率不符合正态分布时的阈值
    const float th = 5.991;

    //信息矩阵，方差平方的倒数
    const float invSigmaSquare = 1.0/(sigma * sigma);

    // 遍历N对特征匹配点
    // 通过H矩阵，进行参考帧和当前帧之间的双向投影，并计算起加权最小二乘投影误差
    // H21 表示从img1 到 img2的变换矩阵
    // H12 表示从img2 到 img1的变换矩阵 
    for(int i = 0; i < N; i++)
    {
		// 一开始都默认为Inlier
        bool bIn = true;

		// 提取参考帧和当前帧之间的特征匹配点对
        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];
        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // 计算 img2 到 img1 的重投影误差
        // x2in1 = H12*x2
        // 将图像2中的特征点单应到图像1中
        // |u1|   |h11inv h12inv h13inv||u2|   |u2in1|
        // |v1| = |h21inv h22inv h23inv||v2| = |v2in1| * w2in1inv
        // |1 |   |h31inv h32inv h33inv||1 |   |  1  |
		// 计算投影归一化坐标
        const float w2in1inv = 1.0/(h31inv * u2 + h32inv * v2 + h33inv);
        const float u2in1 = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
        const float v2in1 = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;
   
        // 计算加权最小二乘误差 = ||p2(i) - H21 * p1(i)||2 = (p2.x * p2.x + p1.x + p1.x) * w
        const float squareDist1 = (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);
        const float chiSquare1 = squareDist1 * invSigmaSquare;

        // 离群点阈值分割，或者累加得分
        if(chiSquare1>th)
            bIn = false;
        else
            score += th - chiSquare1;

        // 计算从img1 到 img2 的投影变换误差
        // x1in2 = H12*x2
        // 将图像2中的特征点单应到图像1中
        // |u2|   |h11 h12 h13||u1|   |u1in2|
        // |v2| = |h21 h22 h23||v1| = |v1in2| * w1in2inv
        // |1 |   |h31 h32 h33||1 |   |  1  |
		// 计算投影归一化坐标
        const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
        const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

        // 计算加权最小二乘误差 
        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);
        const float chiSquare2 = squareDist2*invSigmaSquare;

        // 离群点阈值分割，或者累加得分
        if(chiSquare2>th)
            bIn = false;
        else
            score += th - chiSquare2;

        // 如果从img2 到 img1 和 从img1 到img2的重投影误差均满足要求，则说明是Inlier point
        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }
    return score;
}


float Initializer::CheckFundamental(
    const cv::Mat &F21,             //当前帧和参考帧之间的基础矩阵
    vector<bool> &vbMatchesInliers, //匹配的特征点对属于inliers的标记
    float sigma)                    //方差
{

    // 说明：在已值n维观测数据误差服从N(0，sigma）的高斯分布时
    // 其误差加权最小二乘结果为  sum_error = SUM(e(i)^T * Q^(-1) * e(i))
    // 其中：e(i) = [e_x,e_y,...]^T, Q维观测数据协方差矩阵，即sigma * sigma组成的协方差矩阵
    // 误差加权最小二次结果越小，说明观测数据精度越高
    // 那么，score = SUM((th - e(i)^T * Q^(-1) * e(i)))的分数就越高
    // 算法目标：检查基础矩阵
    // 检查方式：利用对极几何原理 p2^T * F * p1 = 0
    // 假设：三维空间中的点 P 在 img1 和 img2 两图像上的投影分别为 p1 和 p2（两个为同名点）
    //   则：p2 一定存在于极线 l 上，即 p2l = 0. 而l = Fx1 = (a, b, c)^T
    //      所以，这里的误差项 e 为 x2 到 极线 l 的距离，如果在直线上，则 e = 0
    //      根据点到直线的距离公式：d = (ax + by + c) / sqrt(a * a + b * b)
    //      所以，e =  (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)

    // 算法流程
    // input: 基础矩阵 F 左右视图匹配点集 mvKeys1
    //    do:
    //        for p1(i), p2(i) in mvKeys:
    //           l2 = F * p1(i)
    //           l1 = p2(i) * F
    //           error_i1 = dist_point_to_line(x2,l2)
    //           error_i2 = dist_point_to_line(x1,l1)
    //           
    //           w1 = 1 / sigma / sigma
    //           w2 = 1 / sigma / sigma
    // 
    //           if error1 < th
    //              score +=   thScore - error_i1 * w1
    //           if error2 < th
    //              score +=   thScore - error_i2 * w2
    // 
    //           if error_1i > th or error_2i > th
    //              p1(i), p2(i) are inner points
    //              vbMatchesInliers(i) = true
    //           else 
    //              p1(i), p2(i) are outliers
    //              vbMatchesInliers(i) = false
    //           end
    //        end
    //   output: score, inliers

	// 获取匹配的特征点对的总对数
    const int N = mvMatches12.size();

	// 然后提取基础矩阵中的元素数据
    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

	// 预分配空间
    vbMatchesInliers.resize(N);

	// 设置评分初始值（因为后面需要进行这个数值的累计）
    float score = 0;

    // 基于卡方检验计算出的阈值
	// 自由度为1的卡方分布，当平方和有95%的概率不符合正态分布时的阈值
    const float th = 3.841;

    // 此处是为了和checkHomography 保持同一个评分制度，比如满分都是100分
    const float thScore = 5.991;

	// 信息矩阵，或 协方差矩阵的逆矩阵
    const float invSigmaSquare = 1.0/(sigma*sigma);


    // 计算img1 和 img2 在估计 F 时的score值
    for(int i=0; i<N; i++)
    {
		//默认为这对特征点是Inliers
        bool bIn = true;

	    // 提取参考帧和当前帧之间的特征匹配点对
        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

		// 提取出特征点的坐标
        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        /** <li> 计算重投影误差.但是注意由于基础矩阵的定义形式: </li>
         * \n  \f$  \mathbf{x}'\mathbf{F}\mathbf{x}=0 \f$
         * \n 这导致它不能够像单应矩阵那个样子经过求逆直接得到点,这里是可以得到一条线. 
         * 记 \f$ [u_1,v_1,1]^{\text{T}}  \f$ 为在参考帧上的点, \f$  [u_2,v_2,1]^{\text{T}} \f$ 为匹配的在当前帧上的特征点, 那么从当前帧(2)
         * 到参考帧(1)的重投影产生的直线 \f$ \mathbf{l}_2  \f$ 可以按照下式计算出来:
         * \n \f$ \mathbf{l}_2= \begin{bmatrix} a_2\\b_2\\c_2 \end{bmatrix} =
         *    \begin{bmatrix} f_1&f_2&f_3\\ f_4&f_5&f_6\\ f_7&f_8&f_9 \end{bmatrix}^{-1}
         *    \begin{bmatrix} u_2\\v_2\\1 \end{bmatrix}  \f$
         * \n 其中 \f$  [a_2,b_2,1]^{\text{T}} \f$ 是直线 \f$ \mathbf{l}_2  \f$ 的参数. 
         * 在理想状态下, 点 \f$ [u_1,v_1,1]^{\text{T}} \f$ 应该完全在直线 \f$ \mathbf{l}_2 \f$ 上.
         * 这里的重投影误差定义的就是,原真实特征点 \f$ [u_1,v_1,1]^{\text{T}} \f$ 到根据基础矩阵反投影得到的直线 \f$ \mathbf{l}_2 \f$的距离:
         * \n \f$ \\Delta_{1\leftarrow2} ^2= \frac{(a_2u_1+b_2v_1+c_2)^2}{(a_2^2+b_2^2)} \f$
         * \n  同样地,也要和计算单应矩阵的评分一样,计算归一化误差:
         * \n \f$  e_{1\leftarrow2}^2=\frac{ \Delta_{1\leftarrow2} ^2}{\sigma^2} \f$
         */

        // 计算 img1 上的点在 img2 上投影得到的极线 l2 = F21 * p1 = (a2,b2,c2)
		const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;
    
        // 计算误差 e = (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)
        const float num2 = a2*u2+b2*v2+c2;
        const float squareDist1 = num2*num2/(a2*a2+b2*b2);
        // 带权重误差
        const float chiSquare1 = squareDist1*invSigmaSquare;
		
        // 阈值分割 大于就说明这个点是Outlier 
        if(chiSquare1>th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // 计算img2上的点在 img1 上投影得到的极线 l1= p2 * F21 = (a1,b1,c1)
        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        // 计算误差 e = (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)
        const float num1 = a1*u1+b1*v1+c1;
        const float squareDist2 = num1*num1/(a1*a1+b1*b1);

        // 带权重误差
        const float chiSquare2 = squareDist2*invSigmaSquare;

        // 阈值分割 大于就说明这个点是Outlier 
        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;
        
        // 保存结果
        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }
    //  返回评分
    return score;
}


//注意下文中的符号“'”表示矩阵的转置
//                          |0 -1  0|
// E = U Sigma V'   let W = |1  0  0|
//                          |0  0  1|
// 得到4个解 E = [R|t]
// R1 = UWV' R2 = UW'V' t1 = U3 t2 = -U3

//从F恢复R t
bool Initializer::ReconstructF(
    vector<bool> &vbMatchesInliers, //匹配好的特征点对的Inliers标记
    cv::Mat &F21,                   //从参考帧到当前帧的基础矩阵
    cv::Mat &K,                     //相机的内参数矩阵
    cv::Mat &R21,                   //计算好的相机从参考帧到当前帧的旋转
    cv::Mat &t21,                   //计算好的相机从参考帧到当前帧的平移
    vector<cv::Point3f> &vP3D,      //三角化测量之后的特征点的空间坐标
    vector<bool> &vbTriangulated,   //某个特征点是否被三角化了的标记
    float minParallax,              //认为三角化测量有效的最小视差角
    int minTriangulated)            //认为使用三角化测量进行数据判断的最小测量点数量
{
	/** 这里计算基础矩阵的分解:(下面公式的推导来自于吴博师兄的PPT,在工程文件夹下面)
	 * \n \f$ \mathbf{E}=\mathbf{K}^{\text{T}}\mathbf{F}\mathbf{K}  \f$
     * \n 其中 \f$ \mathbf{K} \f$ 是相机的内参数矩阵. 对于基础矩阵 \f$ \mathbf{B} \f$ ,我们可以对其进行SVD分解:
     * \n \f$  \mathbf{E}=\mathbf{U}\mathbf{\Sigma}\mathbf{V}^{\text{T}} \f$
     * \n 令:
     * \n \f$ \mathbf{W}=\mathbf{R}_z(\frac{\pi}{2})= \begin{bmatrix} 0&-1&0\\ 1&0&0\\ 0&0&1 \end{bmatrix}  \f$
     * \n 那么我们可以直接利用结论:
     * \n \f$ \mathbf{E}=[\mathbf{R}|\mathbf{T}]= \begin{cases}
     * \mathbf{R}_1=\mathbf{U}\mathbf{W}\mathbf{V}^{\text{T}}\\ \mathbf{R}_2=\mathbf{U}\mathbf{W}^{\text{T}}\mathbf{V}^{\text{T}}\\
     * \mathbf{T}_1=\mathbf{U}_3\\ \mathbf{T}_2=-\mathbf{U}_3 \end{cases}  \f$
     * \n 一共组成了四种情况. 这个程序的基本操作方法是,统计这四个模型中3D点在摄像头前方投影误差小于给定阈值的3D点个数和每个模型下较大的视察叫;如果其中一个模型的
     * 视差角大于阈值,并且满足条件的3D点明显大于其他模型,那么这个模型就是最优的选择.
     * \n 详细的操作步骤如下: <ul>
	 */ 
    
    //  统计有效匹配点个数，并用 N 表示
    //  vbMatchesInliers 中存储匹配点对是否是有效
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i]) N++;

    // 根据基础矩阵和相机的内参数矩阵计算本质矩阵
    cv::Mat E21 = K.t()*F21*K;

    // 定义本质矩阵分解结果，形成四组解,分别是：
    // (R1, t) (R1, -t) (R2, t) (R2, -t)
    cv::Mat R1, R2, t;

    // 调用解析函数 Initializer::DecomposeE()，从本质矩阵求解两个R解和两个t解，
    // 不过由于两个t解互为相反数，因此这里先只获取一个
    // 虽然这个函数对t有归一化，但并没有决定单目整个SLAM过程的尺度. 
    // 因为 CreateInitialMapMonocular 函数对3D点深度会缩放，然后反过来对 t 有改变.
    DecomposeE(E21,R1,R2,t);  
    cv::Mat t1=t;
    cv::Mat t2=-t;

    // Reconstruct with the 4 hyphoteses and check
    // 从上面求解的4种R和T的组合中，选出最佳组合
    // 原理：若某一组合使恢复得到的3D点位于相机正前方的数量最多，那么该组合就是最佳组合
    // 实现：根据计算的解组合成为四种情况,并依次调用 Initializer::CheckRT() 进行检查,得到可以进行三角化测量的点的数目
	// 定义四组解分别在对同一匹配点集进行三角化测量之后的特征点空间坐标
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;

	// 定义四组解分别对同一匹配点集的有效三角化结果，True or False
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;

	// 定义四种解对应的比较大的特征点对视差角
    float parallax1,parallax2, parallax3, parallax4;

	// 使用同一组匹配点检查四组解，并范围当前解重建的3D点在摄像头前方且投影误差小于阈值的个数，记为有效3D点个数
    int nGood1 = CheckRT(R1,t1,							//当前组解
						 mvKeys1,mvKeys2,				//参考帧和当前帧中的特征点
						 mvMatches12, vbMatchesInliers,	//特征点的匹配关系和Inliers标记
						 K, 							//相机的内参数矩阵
						 vP3D1,							//存储三角化以后特征点的空间坐标
						 4.0*mSigma2,					//三角化测量过程中允许的最大重投影误差
						 vbTriangulated1,				//参考帧中被成功进行三角化测量的特征点的标记
						 parallax1);					//认为某对特征点三角化测量有效的最小视差角
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

    // 选取最大可三角化测量的点的数目maxGood
    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

	// 释放变量，并在后面赋值为最佳R和T
    R21 = cv::Mat();
    t21 = cv::Mat();

    // 确定最小的可以三角化的点数为 0.9倍的内点数. 
    // 如果给定的数目笔这个还大,就用大的.
    int nMinGood = max(static_cast<int>(0.9*N), minTriangulated);

	// 统计四组解中能重建有效3D坐标的解个数
    // 此处的有效是指：当前解能重建的有效3D点个数 > 0.7 * maxGood
    int nsimilar = 0;
    if(nGood1>0.7*maxGood)
        nsimilar++;
    if(nGood2>0.7*maxGood)
        nsimilar++;
    if(nGood3>0.7*maxGood)
        nsimilar++;
    if(nGood4>0.7*maxGood)
        nsimilar++;

    // 四个结果中如果没有明显的最优结果或者没有足够数量的三角化点，则返回失败
    // 结果筛选
    // 条件1: 如果四组解能够重建的最多3D点个数仍然小于所要求的3D点个数（mMinGood），则Pass
    // 条件2: 如果存在两组或两组以上的解能有效重建>0.7*maxGood的3D，则Pass，因为存在两个解
    if(maxGood<nMinGood || nsimilar>1)	
    {
        return false;
    }


    // 选择最佳解
    // 条件1: 有效重建最多的3D点，即maxGood == nGoodx，也即是位于相机前方的3D点个数最多
    // 条件2: 3D点重建时的视差角 parallax 必须大于最小视差角 minParallax，理由是角度越大3D点精度越高

    //看看最好的good点是在哪种解的条件下发生的
    if(maxGood==nGood1)
    {
		//如果该种解下的parallax大于函数参数中给定的最小值
        if(parallax1>minParallax)
        {
            // 存储3D坐标
            vP3D = vP3D1;

			// 获取特征点向量的三角化测量标记
            vbTriangulated = vbTriangulated1;

			// 存储相机姿态
            R1.copyTo(R21);
            t1.copyTo(t21);
			
            // 结束
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2>minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood3)
    {
        if(parallax3>minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood4)
    {
        if(parallax4>minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }

    // 如果有最优解但是不满足对应的parallax>minParallax，或者是其他的原因导致的无法求出相机R，t，那么返回false表示求解失败
    return false;
}


// H矩阵分解常见有两种方法：Faugeras SVD-based decomposition 和 Zhang SVD-based decomposition
// 参考文献：Motion and structure from motion in a piecewise plannar environment
// 这篇参考文献和下面的代码使用了Faugeras SVD-based decomposition算法
// 从H恢复R t
/** @see Motion and structure from motion in a piecewise planar environment. International Journal of Pattern Recognition and Artificial Intelligence, 1988 */
bool Initializer::ReconstructH(
    vector<bool> &vbMatchesInliers, //匹配点对的内点标记
    cv::Mat &H21,                   //从参考帧到当前帧的单应矩阵
    cv::Mat &K,                     //相机的内参数矩阵
    cv::Mat &R21,                   //计算出来的相机旋转
    cv::Mat &t21,                   //计算出来的相机平移
    vector<cv::Point3f> &vP3D,      //世界坐标系下，三角化测量特征点对之后得到的特征点的空间坐标
    vector<bool> &vbTriangulated,   //特征点对被三角化测量的标记
	float minParallax,              //在进行三角化测量时，观测正常所允许的最小视差角
    int minTriangulated)            //最少被三角化的点对数（其实也是点个数）
{
    /** 这个函数从单应矩阵中恢复相机的的运动. 各种计算公式都是从上面的参考文献中导出的,原理复杂,不再详细介绍原因. 
     * 计算过程如下:  <ul>
    */

    /** <li> 统计匹配的特征点对中属于Inlier的个数 </li> */
    int N=0;
	//遍历
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
		//如果被遍历到的这个点属于Inlier
        if(vbMatchesInliers[i])
			//计数变量++
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988

    /** <li> 由于前面我们是在图像上,也就是在相机的归一化成像平面*相机内参矩阵上得到的单应矩阵,首先需要将他们转换到在空间意义下的单应矩阵.  </li> */
    cv::Mat invK = K.inv();
	//这个部分不要看PPT，对不太上，可以结合着视觉SLAM十四讲P146页中单应矩阵的推导来看 
	//其实最后还是缺少了一个因子,但是这个因子最终是反映在单目尺度的不确定性(具体地是表现在后面的平移)上,所以这里完全可以不管它

    cv::Mat A = invK*H21*K;

    /** <li> 对单应矩阵(注意不是函数传递过来的单应矩阵)进行奇异值分解 </li> */
	//存储进行奇异值分解的结果
    cv::Mat U,w,Vt,V;
	//进行奇异值分解
    cv::SVD::compute(A,						//等待被进行奇异值分解的矩阵
					 w,						//奇异值矩阵
					 U,						//奇异值分解左矩阵
					 Vt,					//奇异值分解右矩阵，注意函数返回的是转置
					 cv::SVD::FULL_UV);		//全部分解
	//得到奇异值分解的右矩阵本身
    V=Vt.t();

	//计算矩阵U的行列式的值和矩阵V^t的行列式的值的乘积
	//|V|==|Vt|
    float s = cv::determinant(U)*cv::determinant(Vt);
	
	//取得矩阵的各个奇异值
    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    /** <li> 查看是否满足奇异值分解的的降序排列,如果不符合那么就放弃解算  </li> */
    // SVD分解的正常情况是特征值降序排列
	// NOTICE 妙啊！！！用这种方式来比较浮点数的大小
    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
		//如果不满足奇异值的降序排列则说明计算错误，解算失败，返回false
        return false;
    }


    /** <li> 在ORBSLAM中没有对奇异值 d1 d2 d3的关系进行分类讨论,直接进行了计算  </li> */
    //由于d1 d2 d3的关系再怎么特殊，计算过程也还是都一样的，因此这里没有进行判断而是直接进行处理了
    //存储每一种子情况下解出来的旋转矩阵、平移向量和空间向量
    vector<cv::Mat> vR, vt, vn;
	//预分配空间
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);


    /** <li> 为了方便计算, 先计算了所有量的不带符号的值  </li> */
    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    // 法向量n'= [x1 0 x3] 对应ppt的公式17
	//上面注释中的e1 e3其实就是PPT中公式17的表示+-1的符号变量
	//x1的未加符号版
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
	//下面两个数组则是存储了e1 e3相互搭配可能会出现的四种解的情况
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    /** <li> 根据d'=±d2的情况进行分类讨论.分别计算各种情况下的 n' R' t' </li> */

    //case d'=+d2
    // 计算ppt中公式19
	//当确定了d'=d2时，PPT中公式19中sin_theta项的无符号版本
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);
	//cos_theta项
    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
	//根据e1 e3的各种不同的排列组合，可能的sin_thet项的两种解，这里因为后面计算方便就写成了这种形式
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    // 计算旋转矩阵 R‘，计算ppt中公式18
	//根据不同的e1 e3组合所得出来的四种R t的解
    //      | ctheta      0   -aux_stheta|       | aux1|
    // Rp = |    0        1       0      |  tp = |  0  |
    //      | aux_stheta  0    ctheta    |       |-aux3|

    //      | ctheta      0    aux_stheta|       | aux1|
    // Rp = |    0        1       0      |  tp = |  0  |
    //      |-aux_stheta  0    ctheta    |       | aux3|

    //      | ctheta      0    aux_stheta|       |-aux1|
    // Rp = |    0        1       0      |  tp = |  0  |
    //      |-aux_stheta  0    ctheta    |       |-aux3|

    //      | ctheta      0   -aux_stheta|       |-aux1|
    // Rp = |    0        1       0      |  tp = |  0  |
    //      | aux_stheta  0    ctheta    |       | aux3|
	// 开始遍历这四种情况中的每一种
    for(int i=0; i<4; i++)
    {
		//生成Rp，就是PPT中公式的 R‘
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];		//在这里你就明白为什么前面非得用数组表示了
        Rp.at<float>(2,0)=stheta[i];		
        Rp.at<float>(2,2)=ctheta;

		//NOTICE 这里的变量定义和PPT中的不同，看原始论文：
		//Motion and structure from motion in a piecewise planner environment
        cv::Mat R = s*U*Rp*Vt;
		//将这个“真实”的R添加到相关的向量中
        vR.push_back(R);

		//生成tp
        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];         //注意这里写的是正确的,PPT上的写法不太合适;但是由于x3符号是随机搭配的,所以最后得到的是相同的结果
        tp*=d1-d3;

        // 这里虽然对t有归一化，并没有决定单目整个SLAM过程的尺度
        // 因为CreateInitialMapMonocular函数对3D点深度会缩放，然后反过来对 t 有改变
		//恢复原始的t
        cv::Mat t = U*tp;
		//首先进行向量自归一化，然后添加到vector中
        vt.push_back(t/cv::norm(t));

		//构造法向量np
        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

		//恢复原始的法向量
        cv::Mat n = V*np;
		//看PPT 16页的图，保持平面法向量向上
        if(n.at<float>(2)<0)
            n=-n;
		//添加到vector
        vn.push_back(n);
    }//对于由e1 e3导致的每种可能的解
    
    //case d'=-d2
    // 计算ppt中 公式22，sin_theta的无符号版
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);
	//cos_theta项
    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
	//考虑到e1,e2的取值，这里的sin_theta有两种可能的解，但是为了下面的计算方便需要，这里写成了这种形式
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    //对于每种由e1 e3取值的组合而形成的四种解的情况
    for(int i=0; i<4; i++)
    {
		// 计算旋转矩阵 R‘，计算ppt中公式21
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

		//恢复出原来的R
        cv::Mat R = s*U*Rp*Vt;
		//然后添加到vector中
        vR.push_back(R);

		//构造tp
        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

		//恢复出原来的t
        cv::Mat t = U*tp;
		//归一化之后加入到vector中,要提供给上面的平移矩阵都是要进行过归一化的
        vt.push_back(t/cv::norm(t));

		//构造法向量np
        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

		//恢复出原来的法向量
        cv::Mat n = V*np;
		//保证法向量指向上方
        if(n.at<float>(2)<0)
            n=-n;
		//添加到vector中
        vn.push_back(n);
    }

	//最好的good点
    int bestGood = 0;
	//其次最好的good点
    int secondBestGood = 0;    
	//最好的解的索引，初始值为-1
    int bestSolutionIdx = -1;
	//最大的视差角
    float bestParallax = -1;
	//存储最好解对应的，对特征点对进行三角化测量的结果
    vector<cv::Point3f> bestP3D;
	//最佳解所对应的，那些可以被三角化测量的点的标记
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the WFaugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
	
    /** <li> 对于 d'=d2 和 d'=-d2 分别对应8组(R t) 分别验证每一种解的情况</li> <ul> */
    for(size_t i=0; i<8; i++)
    {
		//第i组解对应的比较大的视差角
        float parallaxi;
		//三角化测量之后的特征点的空间坐标
        vector<cv::Point3f> vP3Di;
		//特征点对是否被三角化的标记
        vector<bool> vbTriangulatedi;
	
        /** <li> 调用 Initializer::CheckRT(), 计算good点的数目 </li> */
        int nGood = CheckRT(vR[i],vt[i],					//当前组解的旋转矩阵和平移向量
							mvKeys1,mvKeys2,				//特征点
							mvMatches12,vbMatchesInliers,	//特征匹配关系以及Inlier标记
							K,								//相机的内参数矩阵
							vP3Di, 							//存储三角化测量之后的特征点空间坐标的
							4.0*mSigma2,					//三角化过程中允许的最大重投影误差
							vbTriangulatedi,				//特征点是否被成功进行三角测量的标记
							parallaxi);						// 这组解在三角化测量的时候的比较大的视差角
        
        /** <li> 更新历史最优和次优的解 </li> */
        // 保留最优的和次优的解.保存次优解的目的是看看最优解是否突出
		//如果当前组解的good点数是历史最优
        if(nGood>bestGood)
        {
			//那么之前的历史最优就变成了历史次优
            secondBestGood = bestGood;
			//更新历史最优点
            bestGood = nGood;
			//最优解的组索引为i（就是当前次遍历）
            bestSolutionIdx = i;
			//更新
            bestParallax = parallaxi;
			//更新
            bestP3D = vP3Di;
			//更新
            bestTriangulated = vbTriangulatedi;
        }
        //如果当前组的good计数小于历史最优但却大于历史次优
        else if(nGood>secondBestGood)
        {
			//说明当前组解是历史次优点，更新之
            secondBestGood = nGood;
        }
    }//分别验证每一组解的情况
    /** </ul> */


    /** <li> 最优解要满足下面的四个条件: </li> <ol>*/
    if(secondBestGood<0.75*bestGood && 		/** <li> 最优解的good点数要足够突出 </li> */
	   bestParallax>=minParallax && 		/** <li> 最优解的视角差大于规定的阈值 </li> */
	   bestGood>minTriangulated && 			/** <li> 最优解的good点数要大于规定的最小的被三角化的点数量 </li> */
	   bestGood>0.9*N)						/** <li> 最优解的good数要足够多，达到9成以上 </li> */
       /** </ol> */
    {
        /** <li> 只有上面的四个条件同时被满足时，我们才能够认为这组解是真正的最好的解、是满足要求的解 </li> */
		
		//从最佳的解的索引访问到R，t
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
		//获得最佳解时，对特征点三角化测量得到的空间坐标
        vP3D = bestP3D;
		//获取特征点的被成功进行三角化的标记
        vbTriangulated = bestTriangulated;

		//返回真，找到了最好的解
        return true;
    }
	//没有找到，返回false
    return false;
    /** </ul> */
}


//其实下面的这些推导，看PPT上讲得也许会更好
// Trianularization: 已知匹配特征点对{x x'} 和 各自相机矩阵{P P'}, 估计三维点 X
// x' = P'X  x = PX
// 它们都属于 x = aPX模型
//                         |X|
// |x|     |p1 p2  p3  p4 ||Y|     |x|    |--p0--||.|
// |y| = a |p5 p6  p7  p8 ||Z| ===>|y| = a|--p1--||X|
// |z|     |p9 p10 p11 p12||1|     |z|    |--p2--||.|
// 采用DLT的方法：x叉乘PX = 0
// |yp2 -  p1|     |0|
// |p0 -  xp2| X = |0|
// |xp1 - yp0|     |0|
// 两个点:
// |yp2   -  p1  |     |0|
// |p0    -  xp2 | X = |0| ===> AX = 0
// |y'p2' -  p1' |     |0|
// |p0'   - x'p2'|     |0|
// 变成程序中的形式：
// |xp2  - p0 |     |0|
// |yp2  - p1 | X = |0| ===> AX = 0
// |x'p2'- p0'|     |0|
// |y'p2'- p1'|     |0|
// 然后就组成了一个四元一次正定方程组，求解呗

//给定投影矩阵P1,P2和图像上的点kp1,kp2，从而恢复3D坐标 (三角化)
void Initializer::Triangulate(
    const cv::KeyPoint &kp1,    //特征点, in reference frame
    const cv::KeyPoint &kp2,    //特征点, in current frame
    const cv::Mat &P1,          //投影矩阵P1
    const cv::Mat &P2,          //投影矩阵P2
    cv::Mat &x3D)               //三维点
{
	// NOTICE
    // 在DecomposeE函数和ReconstructH函数中对t有归一化
    // 这里三角化过程中恢复的3D点深度取决于 t 的尺度，
    // 但是这里恢复的3D点并没有决定单目整个SLAM过程的尺度
    // 因为CreateInitialMapMonocular函数对3D点深度会缩放，然后反过来对 t 有改变

    /** 这里三角化过程中恢复的3D点深度取决于平移向量 t 的尺度,但是这里所恢复的3D点并没有决定整个SLAM过程的尺度. 后面将会有一个名为 CreateInitialMapMonocular()
     *  的函数对3D点的深度进行缩放, 从而对 t 产生改变. 
     * \n 求解的过程比较简单,根据相应的算法构造矩阵,然后对其进行奇异值分解,其中右奇异矩阵的最后一行就是最终的解. 
     * \n 算法还是看相关整理的文件吧.
     */

	//这个就是上面注释中的矩阵A
    cv::Mat A(4,4,CV_32F);

	//构造参数矩阵A
    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);

	//奇异值分解的结果
    cv::Mat u,w,vt;
	//对系数矩阵A进行奇异值分解
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
	//根据前面的结论，奇异值分解右矩阵的最后一行其实就是解，原理类似于前面的求最小二乘解，四个未知数四个方程正好正定
	//别忘了我们更习惯用列向量来表示一个点的空间坐标
    x3D = vt.row(3).t();
	//为了符合其次坐标的形式，使最后一维为1
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}


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
void Initializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)                           //将特征点归一化的矩阵
{
    // 归一化的是这些点在x方向和在y方向上的一阶绝对矩（随机变量的期望）。

    // Step 1 计算特征点X,Y坐标的均值 meanX, meanY
    float meanX = 0;
    float meanY = 0;

	//获取特征点的数量
    const int N = vKeys.size();

	//设置用来存储归一后特征点的向量大小，和归一化前保持一致
    vNormalizedPoints.resize(N);

	//开始遍历所有的特征点
    for(int i=0; i<N; i++)
    {
		//分别累加特征点的X、Y坐标
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    //计算X、Y坐标的均值
    meanX = meanX/N;
    meanY = meanY/N;

    // Step 2 计算特征点X,Y坐标离均值的平均偏离程度 meanDevX, meanDevY，注意不是标准差
    float meanDevX = 0;
    float meanDevY = 0;

    // 将原始特征点减去均值坐标，使x坐标和y坐标均值分别为0
    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

		//累计这些特征点偏离横纵坐标均值的程度
        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    // 求出平均到每个点上，其坐标偏离横纵坐标均值的程度；将其倒数作为一个尺度缩放因子
    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;
    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;

    // Step 3 将x坐标和y坐标分别进行尺度归一化，使得x坐标和y坐标的一阶绝对矩分别为1 
    // 这里所谓的一阶绝对矩其实就是随机变量到取值的中心的绝对值的平均值（期望）
    for(int i=0; i<N; i++)
    {
		//对，就是简单地对特征点的坐标进行进一步的缩放
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    // Step 4 计算归一化矩阵：其实就是前面做的操作用矩阵变换来表示而已
    // |sX  0  -meanx*sX|
    // |0   sY -meany*sY|
    // |0   0      1    |
    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}

/**
 * @brief 用R，t来对特征匹配点三角化，并根据三角化结果判断R,t的合法性
 * 
 * @param[in] R                                     旋转矩阵R
 * @param[in] t                                     平移矩阵t
 * @param[in] vKeys1                                参考帧特征点  
 * @param[in] vKeys2                                当前帧特征点
 * @param[in] vMatches12                            两帧特征点的匹配关系
 * @param[in] vbMatchesInliers                      特征点对内点标记
 * @param[in] K                                     相机内参矩阵
 * @param[in & out] vP3D                            三角化测量之后的特征点的空间坐标
 * @param[in] th2                                   重投影误差的阈值
 * @param[in & out] vbGood                          标记成功三角化点？
 * @param[in & out] parallax                        计算出来的比较大的视差角（注意不是最大，具体看后面代码）
 * @return int 
 */
int Initializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{   
    // 对给出的特征点对及其R t , 通过三角化检查解的有效性，也称为 cheirality check

    // Calibration parameters
	//从相机内参数矩阵获取相机的校正参数
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

	//特征点是否是good点的标记，这里的特征点指的是参考帧中的特征点
    vbGood = vector<bool>(vKeys1.size(),false);
	//重设存储空间坐标的点的大小
    vP3D.resize(vKeys1.size());

	//存储计算出来的每对特征点的视差
    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    // Step 1：计算相机的投影矩阵  
    // 投影矩阵P是一个 3x4 的矩阵，可以将空间中的一个点投影到平面上，获得其平面坐标，这里均指的是齐次坐标。
    // 对于第一个相机是 P1=K*[I|0]
 
    // 以第一个相机的光心作为世界坐标系, 定义相机的投影矩阵
    cv::Mat P1(3,4,				//矩阵的大小是3x4
			   CV_32F,			//数据类型是浮点数
			   cv::Scalar(0));	//初始的数值是0
	//将整个K矩阵拷贝到P1矩阵的左侧3x3矩阵，因为 K*I = K
    K.copyTo(P1.rowRange(0,3).colRange(0,3));
    // 第一个相机的光心设置为世界坐标系下的原点
    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    // 计算第二个相机的投影矩阵 P2=K*[R|t]
    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
	//最终结果是K*[R|t]
    P2 = K*P2;
    // 第二个相机的光心在世界坐标系下的坐标
    cv::Mat O2 = -R.t()*t;

	//在遍历开始前，先将good点计数设置为0
    int nGood=0;

	// 开始遍历所有的特征点对
    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {

		// 跳过outliers
        if(!vbMatchesInliers[i])
            continue;

        // Step 2 获取特征点对，调用Triangulate() 函数进行三角化，得到三角化测量之后的3D点坐标
        // kp1和kp2是匹配好的有效特征点
        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
		//存储三维点的的坐标
        cv::Mat p3dC1;

        // 利用三角法恢复三维点p3dC1
        Triangulate(kp1,kp2,	//特征点
					P1,P2,		//投影矩阵
					p3dC1);		//输出，三角化测量之后特征点的空间坐标		

		// Step 3 第一关：检查三角化的三维点坐标是否合法（非无穷值）
        // 只要三角测量的结果中有一个是无穷大的就说明三角化失败，跳过对当前点的处理，进行下一对特征点的遍历 
        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
			//其实这里就算是不这样写也没问题，因为默认的匹配点对就不是good点
            vbGood[vMatches12[i].first]=false;
			//继续对下一对匹配点的处理
            continue;
        }

        // Check parallax
        // Step 4 第二关：通过三维点深度值正负、两相机光心视差角大小来检查是否合法 

        //得到向量PO1
        cv::Mat normal1 = p3dC1 - O1;
		//求取模长，其实就是距离
        float dist1 = cv::norm(normal1);

		//同理构造向量PO2
        cv::Mat normal2 = p3dC1 - O2;
		//求模长
        float dist2 = cv::norm(normal2);

		//根据公式：a.*b=|a||b|cos_theta 可以推导出来下面的式子
        float cosParallax = normal1.dot(normal2)/(dist1*dist2);

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        // 如果深度值为负值，为非法三维点跳过该匹配点对
        // ?视差比较小时，重投影误差比较大。这里0.99998 对应的角度为0.36°,这里不应该是 cosParallax>0.99998 吗？
        // ?因为后面判断vbGood 点时的条件也是 cosParallax<0.99998 
        // !可能导致初始化不稳定
        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        // 讲空间点p3dC1变换到第2个相机坐标系下变为p3dC2
        cv::Mat p3dC2 = R*p3dC1+t;	
		//判断过程和上面的相同
        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Step 5 第三关：计算空间点在参考帧和当前帧上的重投影误差，如果大于阈值则舍弃
        // Check reprojection error in first image
        // 计算3D点在第一个图像上的投影误差
		//投影到参考帧图像上的点的坐标x,y
        float im1x, im1y;
		//这个使能空间点的z坐标的倒数
        float invZ1 = 1.0/p3dC1.at<float>(2);
		//投影到参考帧图像上。因为参考帧下的相机坐标系和世界坐标系重合，因此这里就直接进行投影就可以了
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

		//参考帧上的重投影误差，这个的确就是按照定义来的
        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        // 重投影误差太大，跳过淘汰
        if(squareError1>th2)
            continue;

        // Check reprojection error in second image
        // 计算3D点在第二个图像上的投影误差，计算过程和第一个图像类似
        float im2x, im2y;
        // 注意这里的p3dC2已经是第二个相机坐标系下的三维点了
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

		// 计算重投影误差
        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        // 重投影误差太大，跳过淘汰
        if(squareError2>th2)
            continue;

        // Step 6 统计经过检验的3D点个数，记录3D点视差角 
        // 如果运行到这里就说明当前遍历的这个特征点对靠谱，经过了重重检验，说明是一个合格的点，称之为good点 
        vCosParallax.push_back(cosParallax);
		//存储这个三角化测量后的3D点在世界坐标系下的坐标
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
		//good点计数++
        nGood++;

		//判断视差角，只有视差角稍稍大一丢丢的才会给打good点标记
		//? bug 我觉得这个写的位置不太对。你的good点计数都++了然后才判断，不是会让good点标志和good点计数不一样吗
        if(cosParallax<0.99998)
            vbGood[vMatches12[i].first]=true;
    }

    // Step 7 得到3D点中较大的视差角，并且转换成为角度制表示
    if(nGood>0)
    {
        // 从小到大排序
        sort(vCosParallax.begin(),vCosParallax.end());

        // !排序后并没有取最大的视差角，而是取一个较大的视差角
		// 作者的做法：如果经过检验过后的有效3D点小于50个，那么就取最后那个最大的视差角
		// 如果大于50个，就取排名第50个的视差角，足够大就可以没有必要非得要最大的 
        // ?可能是为了避免3D点太多时出现太大的视差角，那可以取个中值啊！
        size_t idx = min(50,int(vCosParallax.size()-1));
		//将这个选中的角弧度制转换为角度制
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
		//如果没有good点那么这个就直接设置为0了
        parallax=0;

	//返回good点计数
    return nGood;
}

/**
 * @brief 分解Essential矩阵
 * 
 * F矩阵通过结合内参可以得到Essential矩阵，分解E矩阵将得到4组解 \n
 * 这4组解分别为[R1,t],[R1,-t],[R2,t],[R2,-t]
 * @param E  Essential Matrix
 * @param R1 Rotation Matrix 1
 * @param R2 Rotation Matrix 2
 * @param t  Translation，另外一个结果取它的相反数就行
 * @see Multiple View Geometry in Computer Vision - Result 9.19 p259
 */
void Initializer::DecomposeE(
    const cv::Mat &E,   //Essential Matrix
    cv::Mat &R1,        //Rotation Matrix 1
    cv::Mat &R2,        //Rotation Matrix 2
    cv::Mat &t)         //Translation，另外一个结果取它的相反数就行
{
    /** 其实这里的过程，师兄的PPT上讲得都差不多了。基本步骤如下：<ul> */

    /** <li> 对本质矩阵进行奇异值分解 </li> */
	//准备存储对本质矩阵进行奇异值分解的结果
    cv::Mat u,w,vt;
	//对本质矩阵进行奇异值分解
    cv::SVD::compute(E,w,u,vt);

    /** <li> 左奇异值矩阵U的最后一列就是t，对其进行归一化 </li> 
     * 但是需要注意这个地方并没有决定尺度；尺度的决定将会在CreateInitialMapMonocular函数对3D点深度会缩放，然后反过来对 t 有改变
    */
    u.col(2).copyTo(t);
    t=t/cv::norm(t);

    /** <li> 构造一个绕Z轴旋转pi/2的旋转矩阵W，按照下式组合得到旋转矩阵R1： </li> 
     * \n R1 = u*W*vt
     * \n 计算完成后要检查一下旋转矩阵行列式的数值，使其满足行列式为1的约束
    */
    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

	//计算
    R1 = u*W*vt;
	//检查旋转矩阵行列式的数值
    if(cv::determinant(R1)<0) // 旋转矩阵有行列式为1的约束
        R1=-R1;

	/** <li> 同理将矩阵W取转置来按照相同的公式计算旋转矩阵R2： </li> 
     * \n R2 = u*W.t()*vt
     * \n 当然也要进行同样的保证行列式为1的约束的操作
    */
    R2 = u*W.t()*vt;
	//检查旋转矩阵行列式的数值
    if(cv::determinant(R2)<0)
        R2=-R2;

    /** </ul> */
}

} //namespace ORB_SLAM
