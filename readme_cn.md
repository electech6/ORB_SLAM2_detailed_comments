# ORB-SLAM2 超详细注释

![DEMO](https://github.com/electech6/ORB_SLAM2_detailed_comments/blob/master/demo.gif)

独家视频课程《ORB-SLAM2精讲：原理推导+逐行代码分析》！ [点击进入官网](https://cvlife.net/)

点击进入 [小六的机器人SLAM学习圈](https://mp.weixin.qq.com/s?__biz=MzIxOTczOTM4NA==&mid=2247567299&idx=1&sn=2d6b673b6d5e396bd87ec2a28cd3d27d&chksm=97d52254a0a2ab42ff833b5fa733f2213a1ad1ec8ebdb199d96c237aa4cb21fe8e05cc77db8a#rd)，4500+人都在这里交流！

----


# ORB-SLAM2
**作者：** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) 和 [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**2017年1月13日**：现已支持 OpenCV 3 和 Eigen 3.3。

**2016年12月22日**：添加了 AR 演示（见第7节）。

ORB-SLAM2 是一个用于**单目**、**双目**和**RGB-D**相机的实时 SLAM 库，能够计算相机轨迹和稀疏 3D 重建（在双目和 RGB-D 情况下具有真实尺度）。它能够实时检测回环并重新定位相机。我们提供了在 [KITTI 数据集](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)上以双目或单目模式运行 SLAM 系统的示例，在 [TUM 数据集](http://vision.in.tum.de/data/datasets/rgbd-dataset)上以 RGB-D 或单目模式运行，以及在 [EuRoC 数据集](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)上以双目或单目模式运行。我们还提供了一个 ROS 节点来处理实时的单目、双目或 RGB-D 数据流。**该库可以在没有 ROS 的情况下编译**。ORB-SLAM2 提供了一个 GUI 来在*SLAM 模式*和*定位模式*之间切换，请参见本文档的第9节。

<a href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>


### 相关论文：

[单目] Raúl Mur-Artal, J. M. M. Montiel 和 Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics 最佳论文奖**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[双目和 RGB-D] Raúl Mur-Artal 和 Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[DBoW2 地点识别器] Dorian Gálvez-López 和 Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. 许可证

ORB-SLAM2 在 [GPLv3 许可证](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt)下发布。有关所有代码/库依赖项（及其相关许可证）的列表，请参阅 [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md)。

如需用于商业目的的 ORB-SLAM2 闭源版本，请联系作者：orbslam (at) unizar (dot) es。

如果您在学术工作中使用 ORB-SLAM2（单目），请引用：

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }

如果您在学术工作中使用 ORB-SLAM2（双目或 RGB-D），请引用：

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={33},
      number={5},
      pages={1255--1262},
      doi = {10.1109/TRO.2017.2705103},
      year={2017}
     }

# 2. 先决条件

我们已在 **Ubuntu 12.04**、**14.04** 和 **16.04** 上测试了该库，但在其他平台上应该也很容易编译。强大的计算机（例如 i7）将确保实时性能并提供更稳定和准确的结果。

## C++11 或 C++0x 编译器
我们使用 C++11 的新线程和 chrono 功能。

## Pangolin
我们使用 [Pangolin](https://github.com/stevenlovegrove/Pangolin) 进行可视化和用户界面。下载和安装说明可以在以下网址找到：https://github.com/stevenlovegrove/Pangolin。

## OpenCV
我们使用 [OpenCV](http://opencv.org) 来处理图像和特征。下载和安装说明可以在以下网址找到：http://opencv.org。**至少需要 2.4.3 版本。已测试 OpenCV 2.4.11 和 OpenCV 3.2**。

## Eigen3
g2o 需要（见下文）。下载和安装说明可以在以下网址找到：http://eigen.tuxfamily.org。**至少需要 3.1.0 版本**。

## DBoW2 和 g2o（包含在 Thirdparty 文件夹中）
我们使用修改版的 [DBoW2](https://github.com/dorian3d/DBoW2) 库来执行地点识别，使用 [g2o](https://github.com/RainerKuemmerle/g2o) 库来执行非线性优化。这两个修改后的库（均为 BSD 许可证）都包含在 *Thirdparty* 文件夹中。

## ROS（可选）
我们提供了一些使用 [ROS](ros.org) 处理单目、双目或 RGB-D 相机实时输入的示例。构建这些示例是可选的。如果您想使用 ROS，需要 Hydro 或更高版本。

# 3. 构建 ORB-SLAM2 库和示例

克隆仓库：
```
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
```

我们提供了一个脚本 `build.sh` 来构建 *Thirdparty* 库和 *ORB-SLAM2*。请确保您已安装所有必需的依赖项（见第2节）。执行：

```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

这将在 *lib* 文件夹中创建 **libORB_SLAM2.so**，并在 *Examples* 文件夹中创建可执行文件 **mono_tum**、**mono_kitti**、**rgbd_tum**、**stereo_kitti**、**mono_euroc** 和 **stereo_euroc**。

# 4. 单目示例

## TUM 数据集

1. 从 http://vision.in.tum.de/data/datasets/rgbd-dataset/download 下载一个序列并解压。

2. 执行以下命令。将 `TUMX.yaml` 更改为 TUM1.yaml、TUM2.yaml 或 TUM3.yaml，分别对应 freiburg1、freiburg2 和 freiburg3 序列。将 `PATH_TO_SEQUENCE_FOLDER` 更改为解压后的序列文件夹。

```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER
```

## KITTI 数据集

1. 从 http://www.cvlibs.net/datasets/kitti/eval_odometry.php 下载数据集（灰度图像）。

2. 执行以下命令。将 `KITTIX.yaml` 更改为 KITTI00-02.yaml、KITTI03.yaml 或 KITTI04-12.yaml，分别对应序列 0 到 2、3 和 4 到 12。将 `PATH_TO_DATASET_FOLDER` 更改为解压后的数据集文件夹。将 `SEQUENCE_NUMBER` 更改为 00、01、02、...、11。

```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC 数据集

1. 从 http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets 下载一个序列（ASL 格式）。

2. 对于 V1 和 V2 序列执行第一个命令，对于 MH 序列执行第二个命令。根据您要运行的序列更改 PATH_TO_SEQUENCE_FOLDER 和 SEQUENCE。

```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

# 5. 双目示例

## KITTI 数据集

1. 从 http://www.cvlibs.net/datasets/kitti/eval_odometry.php 下载数据集（灰度图像）。

2. 执行以下命令。将 `KITTIX.yaml` 更改为 KITTI00-02.yaml、KITTI03.yaml 或 KITTI04-12.yaml，分别对应序列 0 到 2、3 和 4 到 12。将 `PATH_TO_DATASET_FOLDER` 更改为解压后的数据集文件夹。将 `SEQUENCE_NUMBER` 更改为 00、01、02、...、11。

```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC 数据集

1. 从 http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets 下载一个序列（ASL 格式）。

2. 对于 V1 和 V2 序列执行第一个命令，对于 MH 序列执行第二个命令。根据您要运行的序列更改 PATH_TO_SEQUENCE_FOLDER 和 SEQUENCE。

```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/mav0/cam0/data PATH_TO_SEQUENCE/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```

```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data PATH_TO_SEQUENCE/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```

# 6. RGB-D 示例

## TUM 数据集

1. 从 http://vision.in.tum.de/data/datasets/rgbd-dataset/download 下载一个序列并解压。

2. 使用 Python 脚本 [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools) 关联 RGB 图像和深度图像。我们已经在 *Examples/RGB-D/associations/* 中为某些序列提供了关联文件。您可以通过执行以下命令生成自己的关联文件：

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. 执行以下命令。将 `TUMX.yaml` 更改为 TUM1.yaml、TUM2.yaml 或 TUM3.yaml，分别对应 freiburg1、freiburg2 和 freiburg3 序列。将 `PATH_TO_SEQUENCE_FOLDER` 更改为解压后的序列文件夹。将 `ASSOCIATIONS_FILE` 更改为相应关联文件的路径。

  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

# 7. ROS 示例

### 构建单目、单目AR、双目和 RGB-D 的节点
1. 将包含 *Examples/ROS/ORB_SLAM2* 的路径添加到 ROS_PACKAGE_PATH 环境变量。打开 .bashrc 文件并在末尾添加以下行。将 PATH 替换为您克隆 ORB_SLAM2 的文件夹：

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS
  ```
  
2. 执行 `build_ros.sh` 脚本：

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### 运行单目节点
对于来自话题 `/camera/image_raw` 的单目输入，运行节点 ORB_SLAM2/Mono。您需要提供词汇表文件和设置文件。请参见上面的单目示例。

  ```
  rosrun ORB_SLAM2 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### 运行单目增强现实演示
这是一个增强现实演示，您可以使用界面在场景的平面区域中插入虚拟立方体。
该节点从话题 `/camera/image_raw` 读取图像。

  ```
  rosrun ORB_SLAM2 MonoAR PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### 运行双目节点
对于来自话题 `/camera/left/image_raw` 和 `/camera/right/image_raw` 的双目输入，运行节点 ORB_SLAM2/Stereo。您需要提供词汇表文件和设置文件。如果您**提供校正矩阵**（参见 Examples/Stereo/EuRoC.yaml 示例），节点将在线校正图像，**否则图像必须预先校正**。

  ```
  rosrun ORB_SLAM2 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```
  
**示例**：从 EuRoC 数据集 (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) 下载一个 rosbag（例如 V1_01_easy.bag）。在终端中打开 3 个标签页，在每个标签页中运行以下命令：

  ```
  roscore
  ```
  
  ```
  rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
  ```
  
一旦 ORB-SLAM2 加载了词汇表，在 rosbag 标签页中按空格键。享受吧！注意：运行该数据集中最苛刻的序列需要强大的计算机。

### 运行 RGB-D 节点
对于来自话题 `/camera/rgb/image_raw` 和 `/camera/depth_registered/image_raw` 的 RGB-D 输入，运行节点 ORB_SLAM2/RGBD。您需要提供词汇表文件和设置文件。请参见上面的 RGB-D 示例。

  ```
  rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
# 8. 处理您自己的序列

您需要创建一个包含相机标定的设置文件。请参见为 TUM 和 KITTI 数据集提供的单目、双目和 RGB-D 相机的设置文件。我们使用 OpenCV 的标定模型。请参见示例以了解如何创建一个使用 ORB-SLAM2 库的程序以及如何将图像传递给 SLAM 系统。双目输入必须同步和校正。RGB-D 输入必须同步且深度已注册。

# 9. SLAM 和定位模式

您可以使用地图查看器的 GUI 在*SLAM 模式*和*定位模式*之间切换。

### SLAM 模式
这是默认模式。系统在三个并行线程中运行：跟踪、局部建图和回环闭合。系统定位相机，构建新地图并尝试闭合回环。

### 定位模式
当您有工作区域的良好地图时，可以使用此模式。在此模式下，局部建图和回环闭合被停用。系统在地图中定位相机（地图不再更新），如果需要，使用重定位。
