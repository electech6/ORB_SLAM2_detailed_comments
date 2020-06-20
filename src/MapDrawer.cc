/**
 * @file MapDrawer.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 绘制地图点
 * @version 0.1
 * @date 2019-02-19 update 2020-06-20
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


#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{

// 构造函数
MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    // 从配置文件中读取可视化有关的设置
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize       = fSettings["Viewer.KeyFrameSize"];             // 关键帧大小
    mKeyFrameLineWidth  = fSettings["Viewer.KeyFrameLineWidth"];        // 关键帧线宽
    mGraphLineWidth     = fSettings["Viewer.GraphLineWidth"];           // 共视图线宽
    mPointSize          = fSettings["Viewer.PointSize"];                // 地图点大小
    mCameraSize         = fSettings["Viewer.CameraSize"];               // 当前帧大小
    mCameraLineWidth    = fSettings["Viewer.CameraLineWidth"];          // 当前帧线宽
}

// 绘制地图点
void MapDrawer::DrawMapPoints()
{
    // Step 1 数据准备
    // 取出所有的地图点
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    //取出 mvpReferenceMapPoints, 局部地图点/参考地图点
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    // 将vpRefMPs从vector容器类型转化为set容器类型，便于使用set::count快速统计 - 我觉得称之为"重新构造"可能更加合适一些
    // std::set<T>::count 用于返回集合中为某个值的元素的个数
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    // 没有地图点, 那我们就跑路
    if(vpMPs.empty())
        return;

    // Step 2 绘制所有的地图点
    // for AllMapPoints
    // 设置点大小
    glPointSize(mPointSize);
    // 开始绘制点
    glBegin(GL_POINTS);
    // 设置绘制颜色为黑色
    glColor3f(0.0, 0.0, 0.0);
    // 遍历所有的地图点
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        // 不包括 已经被标记为删除了的地图点和 ReferenceMapPoints (局部地图点)
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        // 获取点的位置, 并绘制
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
    }
    // 结束绘制
    glEnd();

    // Step 3 绘制局部地图点
    // for ReferenceMapPoints
    // 设置绘制大小
    glPointSize(mPointSize);
    // 开始绘制点
    glBegin(GL_POINTS);
    // 设置绘制颜色为红色
    glColor3f(1.0, 0.0, 0.0);
    // 遍历每一个局部地图点
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        // 如果这个地图点被标记为删除, 就不绘制了
        if((*sit)->isBad())
            continue;
        // 一样的套路, 获取该点的坐标, 然后直接绘制
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));

    }
    // 结束绘制
    glEnd();
}

// 绘制关键帧
void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    // Step 1 准备数据: 宽度占总宽度比例为0.05
    const float &w = mKeyFrameSize;
    const float h  = w * 0.75;
    const float z  = w * 0.6;

    // Step 1 从地图中取出所有的关键帧
    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    // Step 2 显示所有关键帧图标
    // 通过显示界面选择是否显示历史关键帧图标
    if(bDrawKF)
    {
        // 遍历每一个关键帧
        for(size_t i = 0; i < vpKFs.size(); i++)
        {
            // Step 2.1 获取该关键帧的位姿 Twc
            KeyFrame* pKF = vpKFs[i];
            /* 
             * 这里有个小技巧. pKF->GetPoseInverse() 得到的是该关键帧的 Twc, 转置以后应该是 Tcw, 那么为什么程序中的变量还是 Twc 呢?
             * 因为在 OpenGL 中矩阵按照列优先的方式存储, 为了可以直接将 cv::Mat 类型的数据直接送给 OpenGL, 需要加个转置
             * 但因为它在 OpenGL 中代表的含义还是 Twc, 所以这里的变量名还是 Twc 以示区分.
             * 不过每次绘制都要重新计算这个, 有点浪费计算资源, 可以优化这里, 用存储量换计算量
             */
            cv::Mat Twc   = pKF->GetPoseInverse().t();

            // Step 2.2 设置变换矩阵, 相当于设置绘制时所在的坐标系
            glPushMatrix();

            // 设置绘制时的坐标系为该相机坐标系
            glMultMatrixf(Twc.ptr<GLfloat>(0));

            // 设置图元线宽
            glLineWidth(mKeyFrameLineWidth);
            // 设置图元颜色为蓝色 (关键帧图标显示为蓝色) 
            glColor3f(0.0f, 0.0f, 1.0f);
            // 用线将下面的顶点两两相连. 可以以 glBegin 或 GL_LINES 为关键字搜索相关博客
            glBegin(GL_LINES);
            glVertex3f( 0, 0, 0);
            glVertex3f( w, h, z);
            glVertex3f( 0, 0, 0);
            glVertex3f( w,-h, z);
            glVertex3f( 0, 0, 0);
            glVertex3f(-w,-h, z);
            glVertex3f( 0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f( w, h, z);
            glVertex3f( w,-h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w,-h, z);

            glVertex3f(-w, h, z);
            glVertex3f( w, h, z);

            glVertex3f(-w,-h, z);
            glVertex3f( w,-h, z);
            glEnd();

            // Step 2.3 恢复之前的绘制坐标系
            glPopMatrix();
        }
    }

    // Step 3 显示所有关键帧的共视图
    /*
     * 共视图中存储了所有关键帧的共视关系
     * 本征图中对边进行了优化, 保存了所有节点, 只存储了具有较多共视点的边,用于进行优化
     * 生成树则进一步进行了优化, 保存了所有节点, 但是只保存具有最多共视地图点的关键帧的边
     * 
     * 为了好看, 原程序中在实际绘制过程中有限制, 只绘制和当前关键帧共视关系 Top 100 的共视边;
     * 另外程序也绘制了最小生成树和回环边
     * 
     * 注意下面的程序都是在世界坐标系下进行绘制的
     */

    // 通过显示界面选择是否显示关键帧连接关系
    if(bDrawGraph)
    {
        // 设置图元线宽
        glLineWidth(mGraphLineWidth);
        // 设置共视图连接线为绿色，透明度为0.6f
        glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
        // 按照添加顺序,每两个点之间绘制一条直线
        glBegin(GL_LINES);  

        // 遍历所有的关键帧
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph (共视图)
            // Step 3.1 共视程度比较高的共视关键帧用线连接
            // 得到和该关键帧共视点数量 Top 100 的关键帧
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            // 得到该关键帧的相机光心位置, 方便后面绘制连线使用
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                // 如果该关键帧的共视关系不为空, 那么遍历所有的共视关键帧
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    // 避免重复绘制
                    if((*vit)->mnId < vpKFs[i]->mnId)
                        continue;
                    // 获取共视关键帧的位置
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    // 绘制连线
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            // Step 3.2 连接最小生成树, 为了避免重复绘制, 这里只绘制该关键帧和其父关键帧的连线
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                // 一样的套路, 获取父关键帧的位置, 并且绘制连线
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            // Step 3.3 连接闭环时形成的连接关系
            // 获取并遍历当前关键帧的每一个回环边
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                // 避免重复绘制
                if((*sit)->mnId < vpKFs[i]->mnId)
                    continue;
                // 和前面的一样套路, 获取闭环关键帧的位置, 然后进行连线
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }
        // 绘制过程结束
        glEnd();
    }
}

// 绘制当前帧相机图元
void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    // Step 1 相机模型大小：宽度占总宽度比例为 0.08
    const float &w = mCameraSize;
    const float h  = w * 0.75;
    const float z  = w * 0.6;

    // Step 2 将当前已设置的位姿变换矩阵压入 OpenGL 堆栈
    // 百度搜索：glPushMatrix 百度百科, 这里用到的都是相对非常简单的 OpenGL 立即模式, 效率低但程序异常好写, 不需要操心着色器和缓冲区的事情
    // 如果你不知道这步有啥用, 先往后看吧
    glPushMatrix();

    // Step 3 在当前已设置的变换矩阵基础上, 右乘当前位姿矩阵
    /*
     * 这一步相当于设置下面要绘制的内容的位姿. 这样虽然绘制图元使用的顶点都是在当前帧相机坐标系下表示的, 但是 OpenGL 会自动计算它们的世界坐标
     * 可以理解为设置绘制这些顶点数据时所在的坐标系
     * 如果正确安装了显卡驱动, 这个步骤是显卡处理的, 术语 T&L 中的 T 就是这个, 当年老黄就是凭借这些特性的加持打败了 Voodo 加速卡
     * 感兴趣的同学可以了解一下相关的历史, 很有趣
     * NOTE 注意 OpenGL 中矩阵为列优先存储
     */
    
    // 对于嵌入式设备一般使用 OpenGL ES, 为了兼容才有下面的宏定义
#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    // Step 4 绘制当前帧相机图元
    // 设置线宽
    glLineWidth(mCameraLineWidth);
    // 设置颜色为绿色(也就是相机图标显示为绿色). 
    // NOTE 为了兼容性, OpenGL 每个颜色通道使用 0~1 表示, 这点和 OpenCV 不同, 需要注意
    glColor3f(0.0f, 1.0f, 0.0f);
    // 用线将下面的顶点两两相连. glBegin 和 GL_LINES 可以搜相关博客
    glBegin(GL_LINES);
    glVertex3f( 0, 0, 0);
    glVertex3f( w, h, z);
    glVertex3f( 0, 0, 0);
    glVertex3f( w,-h, z);
    glVertex3f( 0, 0, 0);
    glVertex3f(-w,-h, z);
    glVertex3f( 0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f( w, h, z);
    glVertex3f( w,-h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w,-h, z);

    glVertex3f(-w, h, z);
    glVertex3f( w, h, z);

    glVertex3f(-w,-h, z);
    glVertex3f( w,-h, z);
    glEnd();

    // Step 5 设置的当前帧相机位姿出栈, 相当于回退到之前的坐标系
    glPopMatrix();
}

// 设置当前帧相机的位姿, 设置这个函数是因为要处理多线程的操作, 将线程锁的使用和销毁放在一起方便操作
void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    // 保存一下当前帧相机的位姿
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

// 将相机位姿 mCameraPose 由 Mat 类型转化为 OpenGlMatrix 类型
void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    // Step 1 检查是否是有效的相机位姿矩阵, 这里仅仅是检查是否为全零矩阵, 适用于系统刚开始运行的时候
    if(!mCameraPose.empty())
    {
        // Step 2 Tcw => Twc
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        // Step 3 组装成为 OpenGL 所接受的矩阵格式, 注意 OpenGL 中矩阵列优先存储
        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        // 如果送入的相机位姿矩阵为空, 那么就认为相机为默认位姿, 直接设置为单位阵
        M.SetIdentity();
}

} //namespace ORB_SLAM
