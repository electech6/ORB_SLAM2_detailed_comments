#!/bin/bash
# ORB-SLAM2 TUM 单目数据集运行脚本

# 数据集路径
DATASET_PATH="datasets/TUM/testing_and_debugging/rgbd_dataset_freiburg1_xyz"

# 检查数据集是否存在
if [ ! -d "$DATASET_PATH" ]; then
    echo "错误：数据集路径不存在: $DATASET_PATH"
    echo "请确保已解压 TUM 数据集到该路径"
    exit 1
fi

# 检查可执行文件是否存在
if [ ! -f "Examples/Monocular/mono_tum" ]; then
    echo "错误：可执行文件不存在: Examples/Monocular/mono_tum"
    echo "请先运行 ./build.sh 编译项目"
    exit 1
fi

# 检查词汇表文件是否存在
if [ ! -f "Vocabulary/ORBvoc.txt" ]; then
    echo "错误：词汇表文件不存在: Vocabulary/ORBvoc.txt"
    exit 1
fi

# 检查配置文件是否存在
if [ ! -f "Examples/Monocular/TUM1.yaml" ]; then
    echo "错误：配置文件不存在: Examples/Monocular/TUM1.yaml"
    exit 1
fi

echo "=========================================="
echo "ORB-SLAM2 TUM 单目数据集运行"
echo "=========================================="
echo "数据集路径: $DATASET_PATH"
echo "配置文件: Examples/Monocular/TUM1.yaml"
echo ""
echo "提示："
echo "- 程序将打开图形界面显示 SLAM 结果"
echo "- 按 'q' 键退出程序"
echo "- 按空格键可以在 SLAM 模式和定位模式之间切换"
echo ""
echo "正在启动..."
echo ""

# 运行程序
./Examples/Monocular/mono_tum \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/TUM1.yaml \
    "$DATASET_PATH"
