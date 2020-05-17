t/**
 * @file Tracking.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 追踪线程
 * @version 0.1
 * @date 2019-02-21
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

//HACK 更新一下.h文件中的文档注释,主要是函数;以及当前文件中每个步骤的doxygen文档信息


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>
#include<cmath>
#include<mutex>


using namespace std;

// 程序中变量名的第一个字母如果为"m"则表示为类中的成员变量，member
// 第一个、第二个字母:
// "p"表示指针数据类型
// "n"表示int类型
// "b"表示bool类型
// "s"表示set类型
// "v"表示vector数据类型
// 'l'表示list数据类型
// "KF"表示KeyPoint数据类型   NOTICE

namespace ORB_SLAM2
{

///构造函数
Tracking::Tracking(
    System *pSys,                       //系统实例
    ORBVocabulary* pVoc,                //BOW字典
    FrameDrawer *pFrameDrawer,          //帧绘制器
    MapDrawer *pMapDrawer,              //地图点绘制器
    Map *pMap,                          //地图句柄
    KeyFrameDatabase* pKFDB,            //关键帧产生的词袋数据库
    const string &strSettingPath,       //配置文件路径
    const int sensor):                  //传感器类型
        mState(NO_IMAGES_YET),                              //当前系统还没有准备好
        mSensor(sensor),                                
        mbOnlyTracking(false),                              //处于SLAM模式
        mbVO(false),                                        //当处于纯跟踪模式的时候，这个变量表示了当前跟踪状态的好坏
        mpORBVocabulary(pVoc),          
        mpKeyFrameDB(pKFDB), 
        mpInitializer(static_cast<Initializer*>(NULL)),     //暂时给地图初始化器设置为空指针
        mpSystem(pSys), 
        mpViewer(NULL),                                     //注意可视化的查看器是可选的，因为ORB-SLAM2最后是被编译成为一个库，所以对方人拿过来用的时候也应该有权力说我不要可视化界面（何况可视化界面也要占用不少的CPU资源）
        mpFrameDrawer(pFrameDrawer),
        mpMapDrawer(pMapDrawer), 
        mpMap(pMap), 
        mnLastRelocFrameId(0)                               //恢复为0,没有进行这个过程的时候的默认值
{
    // Load camera parameters from settings file
    // Step 1 从配置文件中加载相机参数
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    //     |fx  0   cx|
    // K = |0   fy  cy|
    //     |0   0   1 |
    //构造相机内参矩阵
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    // 图像矫正系数
    // [k1 k2 p1 p2 k3]
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    //有些相机的畸变系数中会没有k3项
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    // 双目摄像头baseline * fx 50
    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    //输出
    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;

    // 1:RGB 0:BGR
    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    // Step 2 加载ORB特征点有关的参数,并新建特征点提取器

    // 每一帧提取的特征点数 1000
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    // 图像建立金字塔时的变化尺度 1.2
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    // 尺度金字塔的层数 8
    int nLevels = fSettings["ORBextractor.nLevels"];
    // 提取fast特征点的默认阈值 20
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    // 如果默认阈值提取不出足够fast特征点，则使用最小阈值 8
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    // tracking过程都会用到mpORBextractorLeft作为特征点提取器
    mpORBextractorLeft = new ORBextractor(
        nFeatures,      //参数的含义还是看上面的注释吧
        fScaleFactor,
        nLevels,
        fIniThFAST,
        fMinThFAST);

    // 如果是双目，tracking过程中还会用用到mpORBextractorRight作为右目特征点提取器
    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // 在单目初始化的时候，会用mpIniORBextractor来作为特征点提取器
    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        // 判断一个3D点远/近的阈值 mbf * 35 / fx
        //ThDepth其实就是表示基线长度的多少倍
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        // 深度相机disparity转化为depth时的因子
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

//设置局部建图器
void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

//设置回环检测器
void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

//设置可视化查看器
void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

// 输入左右目图像，可以为RGB、BGR、RGBA、GRAY
// 1、将图像转为mImGray和imGrayRight并初始化mCurrentFrame
// 2、进行tracking过程
// 输出世界坐标系到该帧相机坐标系的变换矩阵
cv::Mat Tracking::GrabImageStereo(
    const cv::Mat &imRectLeft,      //左侧图像
    const cv::Mat &imRectRight,     //右侧图像
    const double &timestamp)        //时间戳
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    // step 1 ：将RGB或RGBA图像转为灰度图像
    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    //NOTE 这里考虑得十分周全,甚至脸四通道的图像都考虑到了
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    // step 2 ：构造Frame
    mCurrentFrame = Frame(
        mImGray,                //左目图像
        imGrayRight,            //右目图像
        timestamp,              //时间戳
        mpORBextractorLeft,     //左目特征提取器
        mpORBextractorRight,    //右目特征提取器
        mpORBVocabulary,        //字典
        mK,                     //内参矩阵
        mDistCoef,              //去畸变参数
        mbf,                    //基线长度
        mThDepth);              //远点,近点的区分阈值

    // step 3 ：跟踪
    Track();

    //返回位姿
    return mCurrentFrame.mTcw.clone();
}

// 输入左目RGB或RGBA图像和深度图
// 1、将图像转为mImGray和imDepth并初始化mCurrentFrame
// 2、进行tracking过程
// 输出世界坐标系到该帧相机坐标系的变换矩阵
cv::Mat Tracking::GrabImageRGBD(
    const cv::Mat &imRGB,           //彩色图像
    const cv::Mat &imD,             //深度图像
    const double &timestamp)        //时间戳
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    // step 1：将RGB或RGBA图像转为灰度图像
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    // step 2 ：将深度相机的disparity转为Depth , 也就是转换成为真正尺度下的深度
    //这里的判断条件感觉有些尴尬
    //前者和后者满足一个就可以了
    //满足前者意味着,mDepthMapFactor 相对1来讲要足够大
    //满足后者意味着,如果深度图像不是浮点型? 才会执行
    //意思就是说,如果读取到的深度图像是浮点型,就不执行这个尺度的变换操作了呗? TODO 
    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(  //将图像转换成为另外一种数据类型,具有可选的数据大小缩放系数
            imDepth,            //输出图像
            CV_32F,             //输出图像的数据类型
            mDepthMapFactor);   //缩放系数

    // 步骤3：构造Frame
    mCurrentFrame = Frame(
        mImGray,                //灰度图像
        imDepth,                //深度图像
        timestamp,              //时间戳
        mpORBextractorLeft,     //ORB特征提取器
        mpORBVocabulary,        //词典
        mK,                     //相机内参矩阵
        mDistCoef,              //相机的去畸变参数
        mbf,                    //相机基线*相机焦距
        mThDepth);              //内外点区分深度阈值

    // 步骤4：跟踪
    Track();

    //返回当前帧的位姿
    return mCurrentFrame.mTcw.clone();
}

// 输入左目RGB或RGBA图像
// 1、将图像转为mImGray并初始化mCurrentFrame
// 2、进行tracking过程
// 输出世界坐标系到该帧相机坐标系的变换矩阵
cv::Mat Tracking::GrabImageMonocular(
    const cv::Mat &im,          //单目图像
    const double &timestamp)    //时间戳
{
    mImGray = im;

    // Step 1 ：将RGB或RGBA图像转为灰度图像
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    // Step 2 ：构造Frame
    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)// 没有成功初始化的前一个状态就是NO_IMAGES_YET
        mCurrentFrame = Frame(
            mImGray,
            timestamp,
            mpIniORBextractor,      //LNOTICE 这个使用的是初始化的时候的ORB特征点提取器
            mpORBVocabulary,
            mK,
            mDistCoef,
            mbf,
            mThDepth);
    else
        mCurrentFrame = Frame(
            mImGray,
            timestamp,
            mpORBextractorLeft,     //NOTICE 当程序正常运行的时候使用的是正常的ORB特征点提取器
            mpORBVocabulary,
            mK,
            mDistCoef,
            mbf,
            mThDepth);

    // Step 3 ：跟踪
    Track();

    //返回当前帧的位姿
    return mCurrentFrame.mTcw.clone();
}

/**
 * @brief Tracking 线程
 * 
 */
void Tracking::Track()
{
    // track包含两部分：估计运动、跟踪局部地图
    
    // mState为tracking的状态机
    // SYSTME_NOT_READY, NO_IMAGE_YET, NOT_INITIALIZED, OK, LOST
    // 如果图像复位过、或者第一次运行，则为NO_IMAGE_YET状态
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    // mLastProcessedState 存储了Tracking最新的状态，用于FrameDrawer中的绘制
    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    //其实也就只有在这里才上了一次锁
    //? 疑问:这样子不是在大部分的时间中阻止了对地图的更新吗?
    // -- 不过应该注意到ORB-SLAM的主要耗时在特征点的提取和匹配部分,这些部分具体体现在帧的构造函数中,而在那个时候地图是没有被上锁的,
    // -- 还是有足够的时间更新地图的
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    // Step 1 初始化
    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            //双目初始化(RGBD相机也当做为双目相机来看待)
            StereoInitialization();
        else
            //单目初始化
            MonocularInitialization();

        //这一帧处理完成了,更新帧绘制器中存储的最近的一份状态
        mpFrameDrawer->Update(this);

        //这个状态量在上面的初始化函数中被更新
        if(mState!=OK)
            return;
    }
    // Step 2 跟踪
    else
    {
        // System is initialized. Track Frame.

        // bOK为临时变量，用于表示每个函数是否执行成功
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        // 在viewer中有个开关menuLocalizationMode，有它控制是否ActivateLocalizationMode，并最终管控mbOnlyTracking
        // mbOnlyTracking等于false表示正常VO模式（有地图更新），mbOnlyTracking等于true表示用户手动选择定位模式
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            //SLAM模式

            // 正常初始化成功
            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                // 检查并更新上一帧被替换的MapPoints
                // 更新Fuse函数和SearchAndFuse函数替换的MapPoints
                //由于追踪线程需要使用上一帧的信息,而局部建图线程则可能会对原有的地图点进行替换.在这里进行检查
                CheckReplacedInLastFrame();

                // step 2.1：跟踪上一帧或者参考帧或者重定位

                // 运动模型是空的或刚完成重定位
                // mCurrentFrame.mnId < mnLastRelocFrameId + 2 这个判断不应该有
                // 应该只要 mVelocity 不为空，就优先选择 TrackWithMotionModel
                // mnLastRelocFrameId 上一次重定位的那一帧
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    //对于上述两个条件我的理解:
                    //第一个条件,如果运动模型为空,那么就意味着之前已经..跟丢了
                    //第二个条件,就是如果当前帧紧紧地跟着在重定位的帧的后面,那么我们可以认为,通过在当前帧和发生重定位的帧(作为参考关键帧?)的基础上,
                    //我们可以恢复得到相机的位姿

                    // 将上一帧的位姿作为当前帧的初始位姿
                    // 通过BoW的方式在参考帧中找当前帧特征点的匹配点
                    // 优化每个特征点都对应3D点重投影误差即可得到位姿
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    // 根据恒速模型设定当前帧的初始位姿
                    // 通过投影的方式在参考帧中找当前帧特征点的匹配点
                    // 优化每个特征点所对应3D点的投影误差即可得到位姿
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        // TrackReferenceKeyFrame是跟踪参考帧，不能根据固定运动速度模型预测当前帧的位姿态，通过bow加速匹配（SearchByBow）
                        // 最后通过优化得到优化后的位姿
                        //说白了就是根据恒速模型失败了.只能这样根据参考关键帧来了
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                //如果正常的初始化不成功,那么就只能重定位了
                // BOW搜索，PnP求解位姿
                bOK = Relocalization();
            }
        }
        else        //如果现在处于定位模式
        {
            
            // Localization Mode: Local Mapping is deactivated
            // 只进行跟踪tracking，局部地图不工作
 
            // step 2.1：跟踪上一帧或者参考帧或者重定位

            // tracking跟丢了, 那么就只能进行重定位了
            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else    //如果没有跟丢
            {
                // mbVO是mbOnlyTracking为true时的才有的一个变量
                // mbVO为false表示此帧匹配了很多的MapPoints，跟踪很正常，
                // mbVO为true表明此帧匹配了很少的MapPoints，少于10个，要跪的节奏 - 233333
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    // mbVO为0则表明此帧匹配了很多的3D map点，非常好

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                        // 这个地方是不是应该加上：
                        // if(!bOK)
                        //    bOK = TrackReferenceKeyFrame();
                        //? 所以原作者为什么不加呢? 
                    }
                    else
                    {
                        //如果恒速模型不被满足,那么就只能够通过参考关键帧来定位了
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    // mbVO为1，则表明此帧匹配了很少的3D map点，少于10个，要跪的节奏，既做跟踪又做定位

                    //MM=Motion Model,通过运动模型进行跟踪的结果
                    bool bOKMM = false;
                    //通过重定位方法来跟踪的结果
                    bool bOKReloc = false;
                    
                    //运动模型中构造的地图点
                    vector<MapPoint*> vpMPsMM;
                    //在追踪运动模型后发现的外点
                    vector<bool> vbOutMM;
                    //运动模型得到的位姿
                    cv::Mat TcwMM;

                    //当运动模型非空的时候,根据运动模型计算位姿
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        // 这三行没啥用？
                        //? 这三行中的内容会在上面的函数中被更新吗?
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }

                    //使用重定位的方法来得到当前帧的位姿
                    bOKReloc = Relocalization();

                    // 重定位没有成功，但是如果跟踪成功
                    if(bOKMM && !bOKReloc)
                    {
                        // 这三行没啥用？
                        //? 这里为什么说没啥用呢?
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        //如果重定位本身要跪了
                        if(mbVO)
                        {
                            // 这段代码是不是有点多余？应该放到 TrackLocalMap 函数中统一做
                            // 更新当前帧的MapPoints被观测程度
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                //如果这个特征点形成了地图点,并且也不是外点的时候
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    //增加被观测的次数
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }//增加当前可视地图点的被观测次数
                        }//如果重定位要跪了
                    }//如果重定位没有成功,但是跟踪成功了
                    else if(bOKReloc)// 只要重定位成功整个跟踪过程正常进行（定位与跟踪，更相信重定位）
                    {
                        mbVO = false;
                    }
                    //有一个成功我们就认为执行成功了
                    bOK = bOKReloc || bOKMM;
                }//上一次的结果告诉我们本帧要跪了
            }//判断是否跟丢
        }//判断是否处于定位模式

        // 将最新的关键帧作为reference frame
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        // step 2.2：在帧间匹配得到初始的姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
        // NOTICE local map:当mpReferenceKF前帧、当前帧的MapPoints、当前关键帧与其它关键帧共视关系
        //? 没有其他帧的地图点mpReferenceKF吗? 
        // 在步骤2.1中主要是两mpReferenceKF两跟踪（恒速模型跟踪上一帧、跟踪参考帧），这里搜索局部关键帧后搜集所有局部MapPoints，
        // 然后将局部MapPointsmpReferenceKF和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
            //如果前面的两两帧匹配过程进行得不好,这里也就直接放弃了
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.

            // 重定位成功
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        //根据上面的操作来判断是否追踪成功
        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer 中的帧副本的信息
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        //只有在成功追踪时才考虑生成关键帧的问题
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                // step 2.3：更新恒速运动模型 TrackWithMotionModel 中的mVelocity
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                //? 这个是转换成为了相机相对世界坐标系的旋转?
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc; // 其实就是 Tcl
            }
            else
                //否则生成空矩阵
                mVelocity = cv::Mat();

            //相当于更新了地图绘制器
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            // step 2.4：清除UpdateLastFrame中为当前帧临时添加的MapPoints   
            //? 
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    // 排除 UpdateLastFrame 函数中为了跟踪增加的MapPoints
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            // step 2.5：清除临时的MapPoints，这些MapPoints在 TrackWithMotionModel 的 UpdateLastFrame 函数里生成（仅双目和rgbd）
            // 步骤2.4中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
            // 这里生成的仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
            //? 为什么生成这些可以提高帧间跟踪效果?
            //? 为什么单目就不能够生成地图点呢
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            // 这里不仅仅是清除mlpTemporalPoints，通过delete pMP还删除了指针指向的MapPoint

            //不能够直接执行这个是因为其中存储的都是指针,之前的操作都是为了避免内存泄露
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            // step 2.6：检测并插入关键帧，对于双目会产生新的MapPoints
            // NOTICE 在关键帧的时候生成地图点
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            // step 2.7 删除那些在bundle adjustment中检测为outlier的3D map点
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                //这里第一个条件还要执行判断是因为, 前面的操作中可能删除了其中的地图点
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        // 跟踪失败，并且relocation也没有搞定，只能重新Reset
        if(mState==LOST)
        {
            //如果地图中的关键帧信息过少的话甚至直接重新进行初始化了
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        //确保已经设置了参考关键帧
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // 保存上一帧的数据,当前帧变上一帧
        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    // step 3：记录位姿信息，用于轨迹复现
    if(!mCurrentFrame.mTcw.empty())
    {
        // 计算相对姿态T_currentFrame_referenceKeyFrame
        //这里的关键帧存储的位姿,表示的也是从参考关键帧的相机坐标系到世界坐标系的变换
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        //保存各种状态
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        // 如果跟踪失败，则相对位姿使用上一次值
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}// Tracking 

/*
 * @brief 双目和rgbd的地图初始化
 *
 * 由于具有深度信息，直接生成MapPoints
 */
void Tracking::StereoInitialization()
{
    //整个函数只有在当前帧的特征点超过500的时候才会进行
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        // step 1：设定初始位姿
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        // step 2：将当前帧构造为初始关键帧
        // mCurrentFrame的数据类型为Frame
        // KeyFrame包含Frame、地图3D点、以及BoW
        // KeyFrame里有一个mpMap，Tracking里有一个mpMap，而KeyFrame里的mpMap都指向Tracking里的这个mpMap
        // KeyFrame里有一个mpKeyFrameDB，Tracking里有一个mpKeyFrameDB，而KeyFrame里的mpMap都指向Tracking里的这个mpKeyFrameDB
        // 提问: 为什么要指向Tracking中的相应的变量呢? -- 因为Tracking是主线程，是它创建和加载的这些模块
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        // KeyFrame中包含了地图、反过来地图中也包含了KeyFrame，相互包含
        // step 3：在地图中添加该初始关键帧
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        // step 4：为每个特征点构造MapPoint
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            //只有具有正深度的点才会被构造地图点
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                // step 4.1：通过反投影得到该特征点的3D坐标
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                // step 4.2：将3D点构造为MapPoint
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);

                // step 4.3：为该MapPoint添加属性：
                // a.观测到该MapPoint的关键帧
                // b.该MapPoint的描述子
                // c.该MapPoint的平均观测方向和深度范围

                // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
                pNewMP->AddObservation(pKFini,i);
                // b.从众多观测到该MapPoint的特征点中挑选区分度最高的描述子
                
                //? 如何定义的这个区分度?
                pNewMP->ComputeDistinctiveDescriptors();
                // c.更新该MapPoint平均观测方向以及观测距离的范围
                pNewMP->UpdateNormalAndDepth();

                // step 4.4：在地图中添加该MapPoint
                mpMap->AddMapPoint(pNewMP);
                // step 4.5：表示该KeyFrame的哪个特征点可以观测到哪个3D点
                pKFini->AddMapPoint(pNewMP,i);

                // step 4.6：将该MapPoint添加到当前帧的mvpMapPoints中
                // 为当前Frame的特征点与MapPoint之间建立索引
                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        // step 4：在局部地图中添加该初始关键帧
        mpLocalMapper->InsertKeyFrame(pKFini);

        //当前帧变上一帧
        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;


        mvpLocalKeyFrames.push_back(pKFini);
        //? 这个局部地图点竟然..不在mpLocalMapper中管理?
        // 我现在的想法是，这个点只是暂时被保存在了 Tracking 线程之中， 所以称之为 local 
        // 初始化之后，通过双目图像生成的地图点，都应该被认为是局部地图点
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        // 把当前（最新的）局部MapPoints作为ReferenceMapPoints
        // ReferenceMapPoints是DrawMapPoints函数画图的时候用的
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
        //? 设置原点?
        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        //追踪成功
        mState=OK;
    }
}

/*
 * @brief 单目的地图初始化
 *
 * 并行地计算基础矩阵和单应性矩阵，选取其中一个模型，恢复出最开始两帧之间的相对姿态以及点云
 * 得到初始两帧的匹配、相对运动、初始MapPoints
 */
void Tracking::MonocularInitialization()
{
    // 如果单目初始器还没有被创建，则创建单目初始器 NOTICE 也只有单目会使用到这个
    if(!mpInitializer)
    {
        // Set Reference Frame
        // 单目初始帧的特征点数必须大于100
        if(mCurrentFrame.mvKeys.size()>100)
        
        {
            // step 1：得到用于初始化的第一帧，初始化需要两帧
            mInitialFrame = Frame(mCurrentFrame);
            // 记录最近的一帧
            mLastFrame = Frame(mCurrentFrame);
            // mvbPrevMatched最大的情况就是所有特征点都被跟踪上
            //因为当前帧会变成"上一帧"",所以存储到了这个变量中
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            // 这句判断其实看上去有点多余，不过也是在析构指针指向的对象之前先检查一下，来避免出现段错误
            // --- 不对，就是多余！ 因为单目初始化器还没有被构建，我们才来到这个分支的
            if(mpInitializer)
                delete mpInitializer;

            // 由当前帧构造初始器 sigma:1.0 iterations:200
            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            // -1 表示没有任何匹配。这里面存储的是匹配的点的id
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else    //如果单目初始化器已经被创建
    {
        // Try to initialize
        // step 2：如果当前帧特征点数大于100，则得到用于单目初始化的第二帧
        // 如果当前帧特征点太少，重新构造初始器
        // NOTICE 因此只有连续两帧的特征点个数都大于100时，才能继续进行初始化过程
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        // step 3：在mInitialFrame与mCurrentFrame中找匹配的特征点对
        // mvbPrevMatched为前一帧的特征点，存储了mInitialFrame中哪些点将进行接下来的匹配
        // mvIniMatches存储mInitialFrame,mCurrentFrame之间匹配的特征点
        ORBmatcher matcher(
            0.9,        //最佳的和次佳评分的比值阈值
            true);      //检查特征点的方向
        //针对单目初始化的时候,进行特征点的匹配
        int nmatches = matcher.SearchForInitialization(
            mInitialFrame,mCurrentFrame,    //初始化时的参考帧和当前帧
            mvbPrevMatched,                 //在初始化参考帧中提取得到的特征点
            mvIniMatches,                   //保存匹配关系
            100);                           //搜索窗口大小

        // Check if there are enough correspondences
        // step 4：如果初始化的两帧之间的匹配点太少，重新初始化
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        // step 5：通过H模型或F模型进行单目初始化，得到两帧间相对运动、初始MapPoints
        if(mpInitializer->Initialize(
            mCurrentFrame,      //当前帧
            mvIniMatches,       //当前帧和参考帧的特征点的匹配关系
            Rcw, tcw,           //初始化得到的相机的位姿
            mvIniP3D,           //进行三角化得到的空间点集合
            vbTriangulated))    //以及对应于mvIniMatches来讲,其中哪些点被三角化了
        {
            //当初始化成功的时候
            // step 6：删除那些无法进行三角化的匹配点
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            // 将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            // 由Rcw和tcw构造Tcw,并赋值给mTcw，mTcw为世界坐标系到该帧的变换矩阵
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            // step 6：将三角化得到的3D点包装成MapPoints
            // Initialize函数会得到mvIniP3D，
            // mvIniP3D是cv::Point3f类型的一个容器，是个存放3D点的临时变量，
            // CreateInitialMapMonocular将3D点包装成MapPoint类型存入KeyFrame和Map中
            CreateInitialMapMonocular();
        }//当初始化成功的时候进行
    }//如果单目初始化器已经被创建
}

/*
 * @brief CreateInitialMapMonocular
 *
 * 为单目摄像头三角化得到的点生成MapPoints
 */
//使用在单目初始化过程中三角化得到的点,包装成为地图点,并且生成初始地图
void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames 认为单目初始化时候的参考帧和当前帧都是关键帧
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);  // 第一帧
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);  // 第二帧

    // step 1：将初始关键帧的描述子转为BoW
    pKFini->ComputeBoW();
    // step 2：将当前关键帧的描述子转为BoW
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    // step 3：将关键帧插入到地图
    // 凡是关键帧，都要插入地图
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    // step 4：将3D点包装成MapPoints
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        //空间点的世界坐标
        cv::Mat worldPos(mvIniP3D[i]);

        // step 4.1：用3D点构造MapPoint
        MapPoint* pMP = new MapPoint(
            worldPos,
            pKFcur,     // NOTE 这里之所以让新建的地图点的参考关键帧是第二帧，原因是在Tracking过程中有个阶段需要根据上一帧的地图点来进行
                        // 重投影得到在图像区域上的匹配；如果设置了参考关键帧是第一帧的话，那么第三帧就会很尴尬。。。找不到第二帧的地图点
                        // ? 可是下面的 4.3 中，pKFcur也会添加观测信息？
                        // 我怀疑这样做的目的是给这些初始地图点删除的机会。第一帧不会被优化也不会被删除，如果参考关键帧设置成为了第一帧的话，
                        // 就会导致这些初始地图点没有被删除？？
                        // 额。。。好像这样子也说不通
            mpMap);

        // step 4.2：为该MapPoint添加属性：
        // a.观测到该MapPoint的关键帧
        // b.该MapPoint的描述子
        // c.该MapPoint的平均观测方向和深度范围

        // step 4.3：表示该KeyFrame的哪个特征点可以观测到哪个3D点
        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        // b.从众多观测到该MapPoint的特征点中挑选区分度最高的描述子
        pMP->ComputeDistinctiveDescriptors();
        // c.更新该MapPoint平均观测方向以及观测距离的范围
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        //mvIniMatches下标i表示在初始化参考帧中的特征点的序号
        //mvIniMatches[i]是初始化当前帧中的特征点的序号
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        // step 4.4：在地图中添加该MapPoint
        mpMap->AddMapPoint(pMP);
    }//将3D点包装成MapPoints

    // Update Connections
    // step 4.5：更新关键帧间的连接关系
    // 在3D点和关键帧之间建立边，每个边有一个权重，边的权重是该关键帧与当前帧公共3D点的个数
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    // step 5：BA优化
    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    // step 6：!!!将MapPoints的中值深度归一化到1，并归一化两帧之间变换      NOTICE
    // 评估关键帧场景深度，q=2表示中值
    //? 为什么是 pKFini 而不是 pKCur ?
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;
    
    //两个条件,一个是平均深度要大于0,另外一个是在当前帧中被观测到的地图点的数目应该大于100
    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    //将两帧之间的变换归一化到平均深度1的尺度下
    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    // x/z y/z 将z归一化到1 
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    // 把3D点的尺度也归一化到1
    //? 为什么是pKFini? 是不是就算是使用 pKFcur 得到的结果也是相同的? 
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    // 这部分和SteroInitialization()相似
    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    // 单目初始化之后，得到的初始地图中的所有点都是局部地图点
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    //也只能这样子设置了,毕竟是最近的关键帧
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;// 初始化成功，至此，初始化过程完成
}

/*
 * @brief 检查上一帧中的MapPoints是否被替换
 * 
 * Local Mapping线程可能会将关键帧中某些MapPoints进行替换，由于tracking中需要用到mLastFrame，这里检查并更新上一帧中被替换的MapPoints
 * @see LocalMapping::SearchInNeighbors()
 */
//? 目测会发生替换,是因为合并三角化之后特别近的点吗? 
void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        //如果这个地图点存在
        if(pMP)
        {
            //获取其是否被替换,以及替换后的点
            // 这也是程序不选择间这个地图点删除的原因，因为删除了就。。。段错误了
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {   
                //然后重设一下
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

/*
 * @brief 对参考关键帧的MapPoints进行跟踪
 * 
 * 1. 计算当前帧的词包，将当前帧的特征点分到特定层的nodes上
 * 2. 对属于同一node的描述子进行匹配
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配
 * @return 如果匹配数大于10，返回true
 */
bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    // step 1：将当前帧的描述子转化为BoW向量
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    // step 2：通过特征点的BoW加快当前帧与参考帧之间的特征点匹配
    //NOTICE 之前师兄说的，通过词袋模型加速匹配就是在这里哇
    // 特征点的匹配关系由MapPoints进行维护
    int nmatches = matcher.SearchByBoW(
        mpReferenceKF,          //参考关键帧
        mCurrentFrame,          //当前帧
        vpMapPointMatches);     //存储匹配关系

    //这里的匹配数超过15才继续
    if(nmatches<15)
        return false;

    // step 3:将上一帧的位姿态作为当前帧位姿的初始值
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw); // 用上一次的Tcw设置初值，在PoseOptimization可以收敛快一些

    // step 4:通过优化3D-2D的重投影误差来获得位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    // step 5：剔除优化后的outlier匹配点（MapPoints）
    //之所以在优化之后才剔除外点，是因为在优化的过程中就有了对这些外点的标记
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            //如果对应到的某个特征点是外点
            if(mCurrentFrame.mvbOutlier[i])
            {
                //清除它在当前帧中存在过的痕迹
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                //其实这里--也没有什么用了,因为后面也用不到它了
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                //匹配的内点计数++
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

/**
 * @brief 双目或rgbd摄像头根据深度值为上一帧产生新的MapPoints
 *
 * 在双目和rgbd情况下，选取一些深度小一些的点（可靠一些） \n
 * 可以通过深度值产生一些新的MapPoints,用来补充当前视野中的地图点数目,这些新补充的地图点就被称之为"临时地图点""
 */
void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    // step 1：更新最近一帧的位姿
    KeyFrame* pRef = mLastFrame.mpReferenceKF;  //上一帧的参考KF
    // ref_keyframe 到 lastframe的位姿
    cv::Mat Tlr = mlRelativeFramePoses.back();
    // 单目的时候，相当于只是完成了将上一帧的世界坐标系下的位姿计算出来
    mLastFrame.SetPose(Tlr*pRef->GetPose()); // Tlr*Trw = Tlw l:last r:reference w:world

    // 如果上一帧为关键帧，或者单目的情况，则退出
    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR)
        return;

    // step 2：对于双目或rgbd摄像头，为上一帧临时生成新的MapPoints
    // 注意这些MapPoints不加入到Map中，在tracking的最后会删除
    // 跟踪过程中需要将将上一帧的MapPoints投影到当前帧可以缩小匹配范围，加快当前帧与上一帧进行特征点匹配

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    // step 2.1：得到上一帧有深度值的特征点
    //第一个元素是某个点的深度,第二个元素是对应的特征点id
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);

    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)

        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    //如果上一帧中没有有效深度的点,那么就直接退出了
    if(vDepthIdx.empty())
        return;

    // step 2.2：按照深度从小到大排序
    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    // step 2.3：将距离比较近的点包装成MapPoints
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        //如果这个点对应在上一帧中的地图点没有,或者创建后就没有被观测到,那么就生成一个临时的地图点
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)      //? 从地图点被创建后就没有观测到,意味这是在上一帧中新添加的地图点吗
        {
            bCreateNew = true;
        }

        //如果需要创建新的临时地图点
        if(bCreateNew)
        {
            // 这些生UpdateLastFrameints后并没有通过：
            // a.AddMaUpdateLastFrame、
            // b.AddObUpdateLastFrameion、
            // c.CompuUpdateLastFrameinctiveDescriptors、
            // d.UpdatUpdateLastFramelAndDepth添加属性，
            // 这些MapPoint仅仅为了提高双目和RGBD的跟踪成功率   -- 我觉得可以这么说是因为在临时地图中增加了地图点，能够和局部地图一并进行定位工作
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(
                x3D,            //该点对应的空间点坐标
                mpMap,          //? 不明白为什么还要有这个参数
                &mLastFrame,    //存在这个特征点的帧(上一帧)
                i);             //特征点id

            //? 上一帧在处理结束的时候,没有进行添加的操作吗?
            mLastFrame.mvpMapPoints[i]=pNewMP; // 添加新的MapPoint

            // 标记为临时添加的MapPoint，之后在CreateNewKeyFrame之前会全部删除
            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else//如果不需要创建新的 临时地图点
        {
            nPoints++;
        }


        //当当前的点的深度已经超过了远点的阈值,并且已经这样处理了超过100个点的时候,说明就足够了
        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

/**
 * @brief 根据匀速度模型对上一帧的MapPoints进行跟踪
 * 
 * 1. 非单目情况，需要对上一帧产生一些新的MapPoints（临时） (因为传感器的原因，单目情况下仅仅凭借一帧没法生成可靠的地图点)
 * 2. 将上一帧的MapPoints投影到当前帧的图像平面上，在投影的位置进行区域匹配  NOTICE 加快了匹配的速度
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配
 * @return 如果匹配数大于10，返回true
 * @see V-B Initial Pose Estimation From Previous Frame
 */
bool Tracking::TrackWithMotionModel()
{
    // 最小距离 < 0.9*次小距离 匹配成功
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points
    // step 1：更新上一帧的位姿；对于双目或rgbd摄像头，还会根据深度值为上一关键帧生成新的MapPoints
    // （跟踪过程中需要将当前帧与上一帧进行特征点匹配，将上一帧的MapPoints投影到当前帧可以缩小匹配范围）
    // 在跟踪过程中，去除outlier的MapPoint，如果不及时增加MapPoint会逐渐减少
    // 这个函数的功能就是补充增加RGBD和双目相机上一帧的MapPoints数  
    // 那么单目的时候呢? - 单目的时候就不加呗，单目的时候只计算上一帧相对于世界坐标系的位姿
    // ？因为这里来看，计算的位姿是相对于参考关键帧的
    UpdateLastFrame();

    // 根据Const Velocity Model( NOTICE  认为这两帧之间的相对运动和之前两帧间相对运动相同)估计当前帧的位姿
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    //清空当前帧的地图点
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    //这个阈值和下面的的跟踪有关,表示了匹配过程中的搜索半径
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;

    // step 2：根据匀速度模型进行对上一帧的MapPoints进行跟踪, 根据上一帧特征点对应的3D点投影的位置缩小特征点匹配范围
    //我觉的这个才是使用恒速模型的根本目的
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    // 如果跟踪的点少，则扩大搜索半径再来一次
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR); // 2*th
    }

    //如果就算是这样还是不能够获得足够的跟踪点,那么就认为运动跟踪失败了.
    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    // step 3：优化位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    // step 4：优化位姿后剔除outlier的mvpMapPoints,这个和前面相似
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                //累加成功匹配到的地图点数目
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        //如果在纯定位过程中追踪的地图点非常少,那么这里的 mbVO 标志就会置位
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

/**
 * @brief 对Local Map的MapPoints进行跟踪
 * 
 * 1. 更新局部地图，包括局部关键帧和关键点
 * 2. 对局部MapPoints进行投影匹配
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配
 * @return true if success
 * @see V-D track Local Map
 */
bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    // Update Local KeyFrames and Local Points
    // step 1：更新局部关键帧mvpLocalKeyFrames和局部地图点mvpLocalMapPoints -- NO, 局部关键帧在这里面根本没有更新，只是利用而已
    UpdateLocalMap();

    // step 2：在局部地图中查找与当前帧匹配的MapPoints, 其实也就是对局部地图点进行跟踪
    //? 这里的跟踪怎么理解?
    SearchLocalPoints();

    // Optimize Pose
    // 在这个函数之前，在 Relocalization、TrackReferenceKeyFrame、TrackWithMotionModel 中都有位姿优化，
    // step 3：更新局部所有MapPoints后对位姿再次优化
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    // step 3：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            // 由于当前帧的MapPoints可以被当前帧观测到，其被观测统计量加1 -- NOTICE 严格来说这里应该不是被观测
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                //查看当前是否是在纯定位过程
                if(!mbOnlyTracking)
                {
                    // 该MapPoint被其它关键帧观测到过
                    //NOTICE 注意这里的"Obervation"和上面的"Found"并不是一个东西
                    //? 那么区别又在哪里呢?
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    // 记录当前帧跟踪到的MapPoints，用于统计跟踪效果
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)    //如果这个地图点是外点,并且当前相机输入还是双目的时候,就删除这个点;如果是其他类型的输入反而不管了...
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    // step 4：决定是否跟踪成功
    //如果最近刚刚发生了重定位,那么至少跟踪上了50个点我们才认为是跟踪上了
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    //如果是正常的状态话只要跟踪的地图点大于30个我们就认为成功了
    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}

/*
 * @brief 判断当前帧是否为关键帧
 * @return true if needed
 */
bool Tracking::NeedNewKeyFrame()
{
    // step 1：如果用户在界面上选择重定位，那么将不插入关键帧
    // 由于插入关键帧过程中会生成MapPoint，因此用户选择重定位后地图上的点云和关键帧都不会再增加
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    // 如果局部地图被闭环检测使用，则不插入关键帧
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    // step 2：判断是否距离上一次插入关键帧的时间太短
    // mCurrentFrame.mnId是当前帧的ID
    // mnLastRelocFrameId是最近一次重定位帧的ID
    // mMaxFrames等于图像输入的帧率
    // 如果关键帧比较少，则考虑插入关键帧
    // 或距离上一次重定位超过1s，则考虑插入关键帧
    if( mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames &&     //距离上一次重定位不超过1s
        nKFs>mMaxFrames)                                            //地图中的关键帧已经足够
        return false;

    // Tracked MapPoints in the reference keyframe
    // step 3：得到参考关键帧跟踪到的MapPoints数量
	// NOTICE 在 UpdateLocalKeyFrames 函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧 -- 一般的参考关键帧的选择原则
    //地图点的最小观测次数
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    //获取地图点的数目, which 参考帧观测的数目大于等于 nMinObs
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    // step 4：查询局部地图管理器是否繁忙,也就是当前能否接受新的关键帧
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();pan
    // "total matches = matches to map + visual odometry matches"
    // Visual odometry matches will become MapPoints if we insert a keyframe.
    // This ratio measures how many MapPoints we could create if we insert a keyframe.
    // step 5：对于双目或RGBD摄像头，统计 总的可以添加的MapPoints数量 和 跟踪到地图中的MapPoints数量
    int nMap = 0;       //现有地图中,可以被关键帧观测到的地图点数目
    int nTotal= 0;      //当前帧中可以添加到地图中的地图点数量
    if(mSensor!=System::MONOCULAR)// 双目或rgbd
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            //如果是近点,并且这个特征点的深度合法,就可以被添加到地图中
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                nTotal++;// 总的可以添加mappoints数
                if(mCurrentFrame.mvpMapPoints[i])
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        nMap++;// 被关键帧观测到的mappoints数，即观测到地图中的MapPoints数量
            }
        }
    }
    else
    {
        // There are no visual odometry matches in the monocular case
        //? 提问:究竟什么才是 visual odometry matches ?
        nMap=1;
        nTotal=1;
    }

    //计算这个比例,当前帧中观测到的地图点数目和当前帧中总共的地图点数目之比.这个值越接近1越好,越接近0说明跟踪上的地图点太少,tracking is weak
    const float ratioMap = (float)nMap/(float)(std::max(1,nTotal));

    // step 6：决策是否需要插入关键帧
    // Thresholds
    // 设定inlier阈值，和之前帧特征点匹配的inlier比例
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;// 关键帧只有一帧，那么插入关键帧的阈值设置很低 //? 这句话不应该放在下面这句话的后面吗? 
    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;//单目情况下插入关键帧的阈值很高

    // MapPoints中和地图关联的比例阈值
    float thMapRatio = 0.35f;
    if(mnMatchesInliers>300)
        thMapRatio = 0.20f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    // 很长时间没有插入关键帧
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    // localMapper处于空闲状态,才有生成关键帧的基本条件
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    // Condition 1c: tracking is weak
    // 跟踪要跪的节奏，0.25和0.3是一个比较低的阈值
    const bool c1c =  mSensor!=System::MONOCULAR &&             //只有在双目的时候才成立
                    (mnMatchesInliers<nRefMatches*0.25 ||       //和地图点匹配的数目非常少
                      ratioMap<0.3f) ;                          //地图点跟踪成功的比例非常小,要挂了
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    // 阈值比c1c要高，与之前参考帧（最近的一个关键帧）重复度不是太高
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio ||    // 总的来说,还是参考关键帧观测到的地图点的数目太少,少于给定的阈值
                        ratioMap<thMapRatio) &&                     // 追踪到的地图点的数目比例太少,少于阈值
                    mnMatchesInliers>15);                           //匹配到的内点太少

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            //可以插入关键帧
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                // 队列里不能阻塞太多关键帧
                // tracking插入关键帧不是直接插入，而且先插入到mlNewKeyFrames中，
                // 然后localmapper再逐个pop出来插入到mspKeyFrames
                if(mpLocalMapper->KeyframesInQueue()<3)
                    //队列中的关键帧数目不是很多,可以插入
                    return true;
                else
                    //队列中缓冲的关键帧数目太多,暂时不能插入
                    return false;
            }
            else
                //对于单目情况,就直接无法插入关键帧了
                //? 为什么这里对单目情况的处理不一样?
                return false;
        }
    }
    else
        //不满足上面的条件,自然不能插入关键帧
        return false;
}

/**
 * @brief 创建新的关键帧
 *
 * 对于非单目的情况，同时创建新的MapPoints
 */
void Tracking::CreateNewKeyFrame()
{
    //如果不能保持局部建图器开启的状态,就无法顺利插入关键帧
    if(!mpLocalMapper->SetNotStop(true))
        return;

    // step 1：将当前帧构造成关键帧
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    // step 2：将当前关键帧设置为当前帧的参考关键帧
    // 在UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    // 这段代码和 UpdateLastFrame 中的那一部分代码功能相同
    // step 3：对于双目或rgbd摄像头，为当前帧生成新的MapPoints
    if(mSensor!=System::MONOCULAR)
    {
        // 根据Tcw计算mRcw、mtcw和mRwc、mOw
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        // step 3.1：得到当前帧深度小于阈值的特征点
        // 创建新的MapPoint, depth < mThDepth
        //第一个元素是深度,第二个元素是对应的特征点的id
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            // step 3.2：按照深度从小到大排序
            sort(vDepthIdx.begin(),vDepthIdx.end());

            // step 3.3：将距离比较近的点包装成MapPoints
            //处理的近点的个数
            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                //如果当前帧中无这个地图点
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    //或者是刚刚创立
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                //如果需要新建地图点.这里是实打实的在全局地图中新建地图点
                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    // 这些添加属性的操作是每次创建MapPoint后都要做的
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    //? 这里?????
                    nPoints++;
                }

                // 这里决定了双目和rgbd摄像头时地图点云的稠密程度
                // 但是仅仅为了让地图稠密直接改这些不太好，
                // 因为这些MapPoints会参与之后整个slam过程
                //当当前处理的点大于深度阈值或者已经处理的点超过阈值的时候,就不再进行了
                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    //执行插入关键帧的操作,其实也是在列表中等待
    mpLocalMapper->InsertKeyFrame(pKF);

    //然后现在允许局部建图器停止了
    mpLocalMapper->SetNotStop(false);

    //当前帧成为新的关键帧
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}


/**
 * @brief 对 Local MapPoints 进行跟踪
 * 
 * 在局部地图中查找在当前帧视野范围内的点，将视野范围内的点和当前帧的特征点进行投影匹配
 */
void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    // step 1：遍历当前帧的mvpMapPoints，标记这些MapPoints不参与之后的搜索
    // 因为当前的mvpMapPoints一定在当前帧的视野中
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                // 更新能观测到该点的帧数加1(被当前帧看到了)
                pMP->IncreaseVisible();
                // 标记该点被当前帧观测到
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                // 标记该点将来不被投影，因为已经匹配过(指的是使用恒速运动模型进行投影)
                pMP->mbTrackInView = false;
            }
        }
    }//遍历当前帧中所有的地图点

    //准备进行投影匹配的点的数目
    int nToMatch=0;

    // Project points in frame and check its visibility
    // step 2：将所有 局部MapPoints 投影到当前帧，判断是否在视野范围内，然后进行投影匹配
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        // 已经被当前帧观测到MapPoint不再判断是否能被当前帧观测到
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        //局部地图中的坏点也是
        if(pMP->isBad())
            continue;
        
        // Project (this fills MapPoint variables for matching)
        // step 2.1：判断LocalMapPoints中的点是否在在视野内
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
        	// 观测到该点的帧数加1，该MapPoint在某些帧的视野范围内
            pMP->IncreaseVisible();
            // 只有在视野范围内的MapPoints才参与之后的投影匹配
            nToMatch++;
        }
    }

    //如果的确存在需要进行投影匹配的点的数目
    if(nToMatch>0)
    {
        
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)   //RGBD相机输入的时候,搜索的阈值会变得稍微大一些
            th=3;

        // If the camera has been relocalised recently, perform a coarser search
        // 如果不久前进行过重定位，那么进行一个更加宽泛的搜索，阈值需要增大
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        // step 2.2：对视野范围内的MapPoints通过投影进行特征点匹配
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

/**
 * @brief 更新LocalMap
 *
 * 局部地图包括： \n
 * - K1个关键帧、K2个临近关键帧和参考关键帧
 * - 由这些关键帧观测到的MapPoints
 */
void Tracking::UpdateLocalMap()
{
    // This is for visualization
    // 这行程序放在UpdateLocalPoints函数后面是不是好一些
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    // 更新局部关键帧和局部MapPoints
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

/*
 * @brief 更新局部关键点，called by UpdateLocalMap()
 * 
 * 局部关键帧mvpLocalKeyFrames的MapPoints，更新mvpLocalMapPoints
 * \n 我觉得就是先把局部地图清空，然后将局部关键帧的地图点添加到局部地图中
 */
void Tracking::UpdateLocalPoints()
{
    // step 1：清空局部MapPoints
    mvpLocalMapPoints.clear();

    // step 2：遍历局部关键帧 in mvpLocalKeyFrames
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        // step 2：将局部关键帧的MapPoints添加到mvpLocalMapPoints
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            // mnTrackReferenceForFrame 防止重复添加局部MapPoint
            //? 目前不太懂,变量名为什么取这个,以及在之前的什么地方会出现 这个条件被满足 的可能
            //取这个名字的原因可能是表示最近一次追踪到这个地图点是在哪一帧
            //应该是已经添加过了
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

/**
 * @brief 更新局部关键帧，called by UpdateLocalMap()
 *
 * 遍历当前帧的MapPoints，将观测到这些MapPoints的关键帧和相邻的关键帧取出，更新 mvpLocalKeyFrames
 * //?怎么定义"相邻关键帧?" -- 从程序中来看指的就是他们的具有较好的共视关键帧,以及其父关键帧和子关键帧
 */
void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    // step 1：遍历当前帧的MapPoints，记录所有能观测到当前帧MapPoints的关键帧 -- 也就是投票
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                // 能观测到当前帧MapPoints的关键帧
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                //这里由于一个地图点可以被多个关键帧观测到,因此对于每一次观测,都获得观测到这个地图点的关键帧,并且对关键帧进行投票
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    //意味着没有任何一个关键这观测到当前的地图点
    if(keyframeCounter.empty())
        return;

    //? 存储具有最多观测次数的关键帧?
    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    // step 2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧有三个策略
    // 先清空局部关键帧
    mvpLocalKeyFrames.clear();
    //? 提问:为什么要乘3呢? 
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    // V-D K1: shares the map points with current frame
    // 策略1：能观测到当前帧MapPoints的关键帧作为局部关键帧
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;
        
        //更新具有最大观测是-护目的关键帧
        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        
        // mnTrackReferenceForFrame防止重复添加局部关键帧
        //? 这里我可以理解成为,某个关键帧已经被设置为当前帧的 局部关键帧了吗?
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    // V-D K2: neighbors to K1 in the covisibility graph
    // 策略2：与策略1得到的局部关键帧共视程度很高的关键帧作为局部关键帧
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        // 策略2.1:最佳共视的10帧; 如果共视帧不足10帧,那么就返回所有具有共视关系的关键帧
        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);
        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                // mnTrackReferenceForFrame防止重复添加局部关键帧
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // 策略2.2:自己的子关键帧
        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // 策略2.3:自己的父关键帧
        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            // mnTrackReferenceForFrame防止重复添加局部关键帧
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    // V-D Kref： shares the most map points with current frame
    // step 3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
    // 我觉得设立参考关键帧的一个目的就是，后面的各种优化过程中只会优化关键帧的位姿，那普通帧的位姿就放弃？nonono，但是更新吧我们的
    // 计算量又承担不过来，so就设立一个参考关键帧，我管你参考关键帧有没有进行优化，只要我（普通帧）保证能够和你参考关键帧的相对位姿
    // 不变就阔以了
    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

/**
 * @details 重定位过程
 * 
 * Step 1：计算当前帧特征点的Bow映射
 * 
 * Step 2：找到与当前帧相似的候选关键帧
 * 
 * Step 3：通过BoW进行匹配
 * 
 * Step 4：通过EPnP算法估计姿态
 * 
 * Step 5：通过PoseOptimization对姿态进行优化求解
 * 
 * Step 6：如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，再进行优化求解
 */
bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    // Step 1： 计算当前帧特征点的Bow映射
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    // Step 2：找到与当前帧相似的候选关键帧
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
    
    //如果没有候选关键帧，则退出
    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);
    //每个关键帧的解算器
    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    //每个关键帧和当前帧中特征点的匹配关系
    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);
    
    //放弃某个关键帧的标记
    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    //有效的候选关键帧数目
    int nCandidates=0;

    //遍历所有的候选关键帧
    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            //? 前面的查找候选关键帧的时候,为什么不会检查一下这个呢? 为什么非要返回bad的关键帧呢? 关键帧为bad意味着什么呢? 
            //? 此外这个变量的初始值也不一定全部都是false吧
            vbDiscarded[i] = true;
        else
        {
            // Step 3：通过BoW进行匹配
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            //如果和当前帧的匹配数小于15,那么只能放弃这个关键帧
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                // 初始化PnPsolver
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(
                    0.99,   //用于计算RANSAC迭代次数理论值的概率
                    10,     //最小内点数, NOTICE 但是要注意在程序中实际上是min(给定最小内点数,最小集,内点数理论值),不一定使用这个
                    300,    //最大迭代次数
                    4,      //最小集(求解这个问题在一次采样中所需要采样的最少的点的个数,对于Sim3是3,EPnP是4),参与到最小内点数的确定过程中
                    0.5,    //这个是表示(最小内点数/样本总数);实际上的RANSAC正常退出的时候所需要的最小内点数其实是根据这个量来计算得到的
                    5.991); // 目测是自由度为2的卡方检验的阈值,作为内外点判定时的距离的baseline(程序中还会根据特征点所在的图层对这个阈值进行缩放的)
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }//遍历所有的候选关键帧,确定出满足进一步要求的候选关键帧并且为其创建pnp优化器

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    //? 这里的 P4P RANSAC 是啥意思啊   @lishuwei0424:我认为是Epnp，每次迭代需要4个点
    //是否已经找到相匹配的关键帧的标志
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    //通过一系列骚操作,直到找到能够进行重定位的匹配上的关键帧
    while(nCandidates>0 && !bMatch)
    {
        //遍历当前所有的候选关键帧
        for(int i=0; i<nKFs; i++)
        {
            //如果刚才已经放弃了,那么这里也放弃了
            if(vbDiscarded[i])
                continue;
    
            // Perform 5 Ransac Iterations
            //内点标记
            vector<bool> vbInliers;     
            
            //内点数
            int nInliers;
            
            // 表示RANSAC已经没有更多的迭代次数可用 -- 也就是说数据不够好，RANSAC也已经尽力了。。。
            bool bNoMore;

            // Step 4：通过EPnP算法估计姿态
            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            // 如果这里的迭代已经尽力了。。。
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);
                
                //成功被再次找到的地图点的集合,其实就是经过RANSAC之后的内点
                set<MapPoint*> sFound;

                const int np = vbInliers.size();
                //遍历所有内点
                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                // Step 5：通过PoseOptimization对姿态进行优化求解
                //只优化位姿,不优化地图点的坐标;返回的是内点的数量
                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                // ? 如果优化之后的内点数目不多,注意这里是直接跳过了本次循环,但是却没有放弃当前的这个关键帧
                if(nGood<10)
                    continue;

                //删除外点对应的地图点
                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                // Step 6：如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，再进行优化求解
                // 前面的匹配关系是用词袋匹配过程得到的
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(
                        mCurrentFrame,          //当前帧
                        vpCandidateKFs[i],      //关键帧
                        sFound,                 //已经找到的地图点集合
                        10,                     //窗口阈值
                        100);                   //ORB描述子距离

                    //如果通过投影过程获得了比较多的特征点
                    if(nadditional+nGood>=50)
                    {
                        //根据投影匹配的结果，采用3D-2D pnp非线性优化求解
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        //如果这样依赖内点数还是比较少的话,就使用更小的窗口搜索投影点;由于相机位姿已经使用了更多的点进行了优化,所以可以认为使用更小的窗口搜索能够取得意料之内的效果
                        //我觉得可以理解为 垂死挣扎 2333
                        if(nGood>30 && nGood<50)
                        {
                            //重新进行搜索
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(
                                mCurrentFrame,          //当前帧
                                vpCandidateKFs[i],      //候选的关键帧
                                sFound,                 //已经找到的地图点
                                3,                      //新的窗口阈值
                                64);                    //ORB距离? 

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                                //更新地图点
                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                            //如果还是不能够满足就放弃了
                        }//如果地图点还是比较少的话
                    }//如果通过投影过程获得了比较多的特征点
                }//如果内点较少,那么尝试通过投影关系再次进行匹配

                // If the pose is supported by enough inliers stop ransacs and continue
                //如果对于当前的关键帧已经有足够的内点(50个)了,那么就认为当前的这个关键帧已经和当前帧匹配上了
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }//遍历所有的候选关键帧
            // ? 大哥，这里PnPSolver 可不能够保证一定能够得到相机位姿啊？怎么办？
        }//一直运行,知道已经没有足够的关键帧,或者是已经有成功匹配上的关键帧
    }

    //折腾了这么久还是没有匹配上
    if(!bMatch)
    {
        return false;
    }
    else
    {
        //如果匹配上了,说明当前帧重定位成功了(也就是在上面的优化过程中,当前帧已经拿到了属于自己的位姿).因此记录当前帧的id
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }
}//重定位

//整个追踪线程执行复位操作
void Tracking::Reset()
{
    //基本上是挨个请求各个线程终止

    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    cout << "System Reseting" << endl;

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    //然后复位各种变量
    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

//? 目测是根据配置文件中的参数重新改变已经设置在系统中的参数,但是当前文件中没有找到对它的调用
void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    //做标记,表示在初始化帧的时候将会是第一个帧,要对它进行一些特殊的初始化操作
    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

} //namespace ORB_SLAM
