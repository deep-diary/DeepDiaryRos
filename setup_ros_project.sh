#!/bin/bash
# 在Jetson Nano上自动执行ROS工程部署步骤
# 使用方法: ./setup_ros_project.sh

# 设置工作目录
WORKSPACE_DIR=~/Desktop/DeepRos
cd $WORKSPACE_DIR || { echo "无法进入工作目录 $WORKSPACE_DIR"; exit 1; }
echo "进入工作目录: $WORKSPACE_DIR"

# 1. 文件格式转换 - 确保脚本使用Unix行尾
echo "正在转换文件格式..."
# 列出所有Python文件进行检查
PYTHON_FILES=$(find src -name "*.py" -type f)
echo "发现 $(echo "$PYTHON_FILES" | wc -l) 个Python文件"

# 显示几个示例文件路径
echo "示例文件路径:"
echo "$PYTHON_FILES" | head -n 5

# 执行格式转换
find src -name "*.py" -type f -exec dos2unix {} \; 2>/dev/null || { 
  echo "警告: dos2unix命令失败，尝试安装..."
  sudo apt-get update && sudo apt-get install -y dos2unix
  find src -name "*.py" -type f -exec dos2unix {} \;
}
echo "文件格式转换完成"

# 2. 添加可执行权限
echo "正在添加执行权限..."
find src -name "*.py" -type f -exec chmod +x {} \;
echo "权限设置完成"

# 特别检查demos目录
if [ -d "src/URDF_armv001_demo/scripts/demos" ]; then
  echo "检查demos目录中的Python文件..."
  DEMO_FILES=$(find src/URDF_armv001_demo/scripts/demos -name "*.py" -type f)
  echo "demos目录中发现 $(echo "$DEMO_FILES" | wc -l) 个Python文件"
  
  # 确保demos目录下的文件有执行权限
  find src/URDF_armv001_demo/scripts/demos -name "*.py" -type f -exec chmod +x {} \;
  echo "demos目录权限设置完成"
fi

# 3. 编译工作空间
echo "正在编译ROS工作空间..."
catkin_make || { echo "编译失败"; exit 1; }
echo "编译成功"

# 4. 更新环境
echo "正在更新ROS环境..."
source devel/setup.bash
echo "环境更新完成"

# 检查是否有launch文件
LAUNCH_FILES=$(find src -name "*.launch" | wc -l)
if [ "$LAUNCH_FILES" -gt 0 ]; then
  echo "发现 $LAUNCH_FILES 个launch文件:"
  find src -name "*.launch" -type f
fi

# 检查是否有测试节点
TEST_SCRIPTS=$(find src -name "*test*.py" | wc -l)
if [ "$TEST_SCRIPTS" -gt 0 ]; then
  echo "发现 $TEST_SCRIPTS 个测试脚本:"
  find src -name "*test*.py" -type f
fi

echo ""
echo "设置完成! 现在您可以运行ROS节点或launch文件了"
echo "例如:"
echo "  roslaunch URDF_armv001_demo test_serial.launch"
echo "或"
echo "  rosrun URDF_armv001_demo demo_launcher.py list"

# 添加到当前shell环境
exec bash 