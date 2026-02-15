#!/bin/bash
set -e  # 遇到错误立即停止

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
BUILD_DIR="${SCRIPT_DIR}/build"
THIRDPARTY_DIR="${SCRIPT_DIR}/Thirdparty"
BOOST_ROOT="${THIRDPARTY_DIR}/boost_1_83_0"

echo "=============================================="
echo "Project Dir: ${SCRIPT_DIR}"
echo "Boost Root : ${BOOST_ROOT}"
echo "Build Dir  : ${BUILD_DIR}"
echo "=============================================="

# Check if Boost directory exists
if [ ! -d "${BOOST_ROOT}" ]; then
    echo "Error: Boost directory not found at ${BOOST_ROOT}"
    exit 1
fi

# 1. 清理旧构建
if [ -d "${BUILD_DIR}" ]; then
    echo "Cleaning old build directory..."
    rm -rf "${BUILD_DIR}"
fi
mkdir -p "${BUILD_DIR}"

# 2. 配置环境 (如果未加载)
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
else
    echo "Warning: ROS Noetic setup file not found. Make sure ROS is installed."
fi

# 3. 运行 CMake
# 关键修复:
# -DBOOST_ROOT: 强制指向本地 Boost 1.83.0
# -DBoost_NO_SYSTEM_PATHS=ON: 防止 CMake 找到系统安装的旧版 Boost (1.71.0)
# -DBoost_NO_BOOST_CMAKE=ON: 强制使用 CMake 的 FindBoost 模块而不是 Boost 自带的配置（有时会有兼容性问题）
cd "${BUILD_DIR}"
echo "Configuring CMake..."
# 尝试更强力的设置，如果 catkin 依然带入旧库，则可能需手动修剪
cmake .. \
    -DBOOST_ROOT="${BOOST_ROOT}" \
    -DBoost_NO_SYSTEM_PATHS=ON \
    -DBoost_NO_BOOST_CMAKE=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${BOOST_ROOT};/opt/ros/noetic" 

# 4. 编译
echo "Building project..."
make -j$(nproc)

echo "Build successfully finished!"