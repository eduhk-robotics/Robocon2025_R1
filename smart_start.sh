#!/bin/bash

# 获取 NumPy 和 Python 头文件路径（用于编译）
NUMPY_INCLUDE=$(python -c "import numpy; print(numpy.get_include())")
PYTHON_INCLUDE=$(python -c "from sysconfig import get_paths; print(get_paths()['include'])")

# 设置构建目录
WS_DIR=~/Robocon2025_R1/2025R1_ws

echo "🧹 清理旧的构建缓存..."
rm -rf $WS_DIR/build $WS_DIR/install $WS_DIR/log

echo "🔧 重新构建工作空间..."
colcon build --symlink-install \
  --cmake-args \
    -DPython3_NumPy_INCLUDE_DIRS=$NUMPY_INCLUDE \
    -DPython3_INCLUDE_DIR=$PYTHON_INCLUDE

BUILD_RESULT=$?

if [ $BUILD_RESULT -ne 0 ]; then
  echo "❌ 构建失败，请检查上方错误信息。"
  exit 1
fi

echo "✅ 构建成功，启动各个节点..."

# 节点启动函数
function launch_node() {
  PACKAGE=$1
  EXECUTABLE=$2
  LABEL=$3

  if ! ros2 run $PACKAGE $EXECUTABLE --help &> /dev/null; then
    echo "⚠️ 无法找到 $PACKAGE/$EXECUTABLE，跳过 $LABEL"
  else
    echo "🚀 启动 $LABEL..."
    gnome-terminal -- bash -c "ros2 run $PACKAGE $EXECUTABLE; exec bash"
  fi
}

# 启动节点列表
launch_node active_caster vesc_node vesc_node
launch_node active_caster vesc_canbus_speed_control_node vesc_canbus_speed_node
launch_node navigation navigation_node navigation_node
launch_node navigation active_caster_node active_caster_node
launch_node joystick_driver joystick_node joystick_node
#launch_node ps4 ps4_publisher ps4_publisher # 若需要可取消注释

echo "🎉 所有可用节点已尝试启动完毕。"

