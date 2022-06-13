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
#include<unistd.h>
using namespace std;


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    /*
     * argv[0]  ?????未知
     * argv[1] 词典文件路径
     * argv[2] 配置文件路径
     * argv[3] 数据集的路径
    */

    // 检查输入参数是不是四个，如果不是就说明图像的路径没有设置清楚
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    // 检查图像的路径和加载图像
    vector<string> vstrImageFilenames;  // 数据集中每帧图像的文件名
    vector<double> vTimestamps;         // 数据集中每帧图像的时间戳
    string strFile = string(argv[3])+"/rgb.txt";// 读取包含数据集中图像名的 txt 文件???这是个啥???
    LoadImages(strFile, vstrImageFilenames, vTimestamps);//导入图像???

    int nImages = vstrImageFilenames.size();//读入图像的个数

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // 通过System类的构造函数初始化一个SLAM 系统
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop. 主循环逐帧处理
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file 读取每一帧图像及其时间戳
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];//读取每帧图像的时间戳

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

// COMPILEDWITHC11是什么???
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // 跟踪单目图像
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads 关闭线程，统计时间，保存轨迹
    SLAM.Shutdown();

    // Tracking time statistics 统计追踪时间
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory 保存相机轨迹
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

/**
 * @brief 导入图片
 * 
 * @param[in] strFile                   读入的文件名称
 * @param[in&out] vstrImageFilenames    彩色图片名称
 * @param[in&out] vTimestamps           记录时间戳
 */
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;//f输入流
    f.open(strFile.c_str());

    // skip first three lines
    // 前三行是注释，跳过
    string s0;//表示把从输入流f读入的字符串存放在这个字符串s0中（可以自己随便命名，str什么的都可以）；
    // 在不设置的情况下系统默认该字符为'\n'，也就是回车换行符（遇到回车停止读入）。
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;//每帧图像的时间戳
            string sRGB;// 每帧图像的文件名
            ss >> t;
            vTimestamps.push_back(t);// 获取每帧图像的时间戳
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);// 获取每帧图像的文件名
        }
    }
}
