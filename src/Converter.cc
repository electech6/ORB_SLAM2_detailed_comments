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
 * @file Converter.cc
 * @author guoqing (1337841346@qq.com)
 * @brief ORB-SLAM2中一些常用的转换的实现
 * @version 0.1
 * @date 2019-01-03
 * 
 * @copyright Copyright (c) 2019
 * 
 */


#include "Converter.h"


namespace ORB_SLAM2
{

//将描述子转换为描述子向量，其实本质上是cv:Mat->std:vector
std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
	//存储转换结果的向量
    std::vector<cv::Mat> vDesc;
	//创建保留空间
    vDesc.reserve(Descriptors.rows);
	//对于每一个特征点的描述子
    for (int j=0;j<Descriptors.rows;j++)
		//从描述子这个矩阵中抽取出来存到向量中
        vDesc.push_back(Descriptors.row(j));
	
	//返回转换结果
    return vDesc;
}


//将变换矩阵转换为李代数se3：cv:Mat->g2o::SE3Quat
g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
	//首先将旋转矩阵提取出来
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

	//然后将平移向量提取出来
    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

	//构造g2o::SE3Quat类型并返回
    return g2o::SE3Quat(R,t);
}

//李代数se3转换为变换矩阵：g2o::SE3Quat->cv::Mat
cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
	///在实际操作上，首先转化成为Eigen中的矩阵形式，然后转换成为cv::Mat的矩阵形式。
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
	//然后再由Eigen::Matrix->cv::Mat
    return toCvMat(eigMat);
}

//将仿射矩阵由g2o::Sim3->cv::Mat
cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
	///首先将仿射矩阵的旋转部分转换成为Eigen下的矩阵格式
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
	///对于仿射矩阵的平移部分也是要转换成为Eigen下的矩阵格式
    Eigen::Vector3d eigt = Sim3.translation();
	///获取仿射矩阵的缩放系数
    double s = Sim3.scale();
	///然后构造cv::Mat格式下的仿射矩阵
	///@todo 感觉这里的sim3就是在se3的基础上多了一个缩放系数，但是实际上真的是这样吗？
    return toCvSE3(s*eigR,eigt);
}

//Eigen::Matrix<double,4,4> -> cv::Mat，用于变换矩阵T的中间转换
cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
	//首先定义存储计算结果的变量
    cv::Mat cvMat(4,4,CV_32F);
	//然后逐个元素赋值
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

	//返回计算结果，还是用深拷贝函数
    return cvMat.clone();
}

//Eigen::Matrix3d -> cv::Mat 
cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
	//首先定义存储计算结果的变量
    cv::Mat cvMat(3,3,CV_32F);
	//然后逐个元素进行赋值
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

	//返回深拷贝形式的转换结果
    return cvMat.clone();
}

//Eigen::Matrix<double,3,1> -> cv::Mat
cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
	//首先定义保存转换结果的变量
    cv::Mat cvMat(3,1,CV_32F);
	//还是老办法，挨个赋值
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

	//返回转换结果
    return cvMat.clone();
}

//将给定的旋转矩阵R和平移向量t转换成为 以cv::Mat格式存储的李群SE3（其实就是变换矩阵）
cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
	//首先生成用于存储转换结果的单位矩阵
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
	//将旋转矩阵复制到左上角
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    //将旋转矩阵复制到最右侧的一列
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    //返回计算结果
    return cvMat.clone();
}

// 将OpenCV中Mat类型的向量转化为Eigen中Matrix类型的变量
Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
	//首先生成用于存储转换结果的向量
    Eigen::Matrix<double,3,1> v;
	//然后通过逐个赋值的方法完成转换
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);
	//返回转换结果
    return v;
}

//cv::Point3f -> Eigen::Matrix<double,3,1>
Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
	//声明存储转换结果用的变量
    Eigen::Matrix<double,3,1> v;
	//直接赋值的方法
    v << cvPoint.x, cvPoint.y, cvPoint.z;
	//返回转换结果
    return v;
}

//cv::Mat -> Eigen::Matrix<double,3,3>
Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
	//生成用于存储转换结果的变量
    Eigen::Matrix<double,3,3> M;

	//然后也就是相当赋值转换了
    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);
	//返回转换结果
    return M;
}

//将cv::Mat类型的四元数转换成为std::vector型
std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
	//首先将cv::Mat格式的旋转矩阵转换成为Eigen::Matrix格式
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
	//然后利用这个矩阵转换成为四元数格式
	Eigen::Quaterniond q(eigMat);
	//最后声明一个这样的向量
    std::vector<float> v(4);
	//将四元数的四个元素分别保存到这个vector中
    ///@note 注意,使用std::vector存储的四元数的顺序是x y z w
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();
	//返回转换结果
    return v;
}

} //namespace ORB_SLAM
