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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

/**
 * @brief 加载图像
 * @param[in]  strPathLeft          保存左目图像文件名称的文件的路径
 * @param[in]  strPathRight         保存右目图像文件名称的文件的路径
 * @param[in]  strPathTimes         保存图像时间戳的文件的路径
 * @param[out] vstrImageLeft        左目图像序列中每张图像的文件名
 * @param[out] vstrImageRight       右目图像序列中每张图像的文件名
 * @param[out] vTimeStamps          图像序列中每张图像的时间戳(认为左目图像和右目图像的时间戳已经对齐)
 */
void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    // step 0 参数检查
    if(argc != 6)
    {
        cerr << endl << "Usage: ./stereo_euroc path_to_vocabulary path_to_settings path_to_left_folder path_to_right_folder path_to_times_file" << endl;
        return 1;
    }

    // step 1 获取图像的访问路径
    // Retrieve paths to images
    // 保存左右目图像每张图像路径和时间戳的向量
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimeStamp;
    //获得图像位置
    LoadImages(string(argv[3]), string(argv[4]), string(argv[5]), vstrImageLeft, vstrImageRight, vTimeStamp);

    //检查图像序列是否为空
    if(vstrImageLeft.empty() || vstrImageRight.empty())
    {
        cerr << "ERROR: No images in provided path." << endl;
        return 1;
    }

    // 当然如果左右目图像的数目不一致也不可以
    if(vstrImageLeft.size()!=vstrImageRight.size())
    {
        cerr << "ERROR: Different number of left and right images." << endl;
        return 1;
    }

    // step 2 从给出的配置文件中读取去畸变参数
    // Read rectification parameters
    cv::FileStorage fsSettings(
            argv[2],                    // path_to_settings
            cv::FileStorage::READ);     // 以只读方式打开
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    //相机内参
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;
    //TODO 目测是经过双目立体矫正后的投影矩阵
    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    // 修正变换矩阵,见下面的函数调用
    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    //去畸变参数
    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    //图像尺寸
    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    //参数合法性检查(不为空就行)
    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    //存储四个映射矩阵
    cv::Mat M1l,M2l,M1r,M2r;
    //根据相机内参,去畸变参数以及立体矫正后,得到的新的相机内参来计算从原始图像到处理后理想双目图像的映射矩阵
    //这个函数的定义参考:[https://blog.csdn.net/u013341645/article/details/78710740]
    // 左目
    cv::initUndistortRectifyMap(            //计算无畸变和修正转换映射
        K_l,                                //输入的相机内参矩阵 (单目标定阶段得到的相机内参矩阵)
        D_l,                                //单目标定阶段得到的相机的去畸变参数
        R_l,                                //可选的修正变换矩阵,3*3, 从 cv::stereoRectify 得来.如果这个矩阵为空矩阵,那么就将会被设置成为单位矩阵
        P_l.rowRange(0,3).colRange(0,3),    //新的相机内参矩阵
        cv::Size(cols_l,rows_l),            //在去畸变之前的图像尺寸
        CV_32F,                             //第一个输出映射的类型
        M1l,                                //第一个输出映射表
        M2l);                               //第二个输出映射
    // 右目
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    // step 3 构造SLAM系统

    // 获取图像数目
    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // 创建系统
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    // step 4 对每一张输入的图像执行追踪
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        // step 4.1 读取原始图像
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);

        //合法性检查
        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(imRight.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageRight[ni]) << endl;
            return 1;
        }

        // step 4.2 对左右目图像进行双目矫正和去畸变处理
        //参考博客 [https://blog.csdn.net/sss_369/article/details/52983123]
        // 左目
        cv::remap(              //重映射，就是把一幅图像中某位置的像素放置到另一个图片指定位置的过程。
            imLeft,             //输入图像
            imLeftRect,         //输出图像
            M1l,                //第一个映射矩阵表
            M2l,                //第二个映射矩阵
            cv::INTER_LINEAR);
        // 右目
        cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);

        double tframe = vTimeStamp[ni];

        // step 4.3 开始计时

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        // step 4.5 开始追踪
        SLAM.TrackStereo(imLeftRect,imRightRect,tframe);

        // step 4.6 追踪完成，停止计时，计算追踪时间

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        // step 4.7 等待一段时间以符合下一帧图像的时间戳
        double T=0;
        if(ni<nImages-1)
            T = vTimeStamp[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimeStamp[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // step 5 追踪过程执行完毕，退出系统
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    // step 6 统计追踪时间
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    // step 7 以TUM格式保存轨迹文件（普通帧+ 关键帧）
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    return 0;
}

//加载图像，和单目中的做法一样
void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}
