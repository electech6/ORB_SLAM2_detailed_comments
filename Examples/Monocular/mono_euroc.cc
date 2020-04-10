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
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

/**
 * @brief 获取图像文件的信息
 * @param[in]  strImagePath     图像文件存放路径
 * @param[in]  strPathTimes     时间戳文件的存放路径
 * @param[out] vstrImages       图像文件名数组
 * @param[out] vTimeStamps      时间戳数组
 */
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    // step 0 检查输入参数个数是否足够
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file" << endl;
        return 1;
    }

    // step 1 加载图像
    // Retrieve paths to images
    // 图像序列的文件名字符串序列
    vector<string> vstrImageFilenames;
    // 时间戳
    vector<double> vTimestamps;
    LoadImages(string(argv[3]),             // path_to_image_folder
               string(argv[4]),             // path_to_times_file
               vstrImageFilenames,          // 读取到的图像名称数组
               vTimestamps);                // 时间戳数组

    // 当前图像序列中的图像数目
    int nImages = vstrImageFilenames.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // step 2 加载SLAM系统
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(
        argv[1],                            // path_to_vocabulary
        argv[2],                            // path_to_settings
        ORB_SLAM2::System::MONOCULAR,       // 单目模式
        true);                              // 启用可视化查看器

    // step 3 运行前准备
    // Vector for tracking time statistics
    // 统计追踪一帧耗时 (仅Tracker线程)
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    // step 4 依次追踪序列中的每一张图像
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        // step 4.1 读根据前面获得的图像文件名读取图像,读取过程中不改变图像的格式 
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        // step 4.2 图像的合法性检查
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

        // step 4.3 开始计时
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        // step 4.4 追踪当前图像
        SLAM.TrackMonocular(im,tframe);

        // step 4.5 追踪完成,停止当前帧的图像计时, 并计算追踪耗时

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        // step 4.6 根据图像时间戳中记录的两张图像之间的时间和现在追踪当前图像所耗费的时间,继续等待指定的时间以使得下一张图像能够
        // 按照时间戳被送入到SLAM系统中进行跟踪
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // step 5 如果所有的图像都预测完了,那么终止当前的SLAM系统
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    // step 6 计算平均耗时
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
    // step 7 保存TUM格式的相机轨迹
    // 估计是单目时有尺度漂移, 而LGA GBA都只能优化关键帧使尺度漂移最小, 普通帧所产生的轨迹漂移这里无能为力, 我猜作者这样就只
    // 保存了关键帧的位姿,从而避免普通帧带有尺度漂移的位姿对最终误差计算的影响
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

// 从文件中加载图像序列中每一张图像的文件路径和时间戳
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    // 打开文件
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    // 遍历文件
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        // 只有在当前行不为空的时候执行
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            // 生成当前行所指出的RGB图像的文件名称
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            // 记录该图像的时间戳
            vTimeStamps.push_back(t/1e9);

        }
    }
}
