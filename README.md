# UrbanPlanning README

## 1. 下载代码

## 2. 编译

### 2.1 环境要求
ubuntu 20.04 + ROS1

确保你的`cmake`版本大于3.21。你可以使用以下命令来检查你的`cmake`版本：

```bash
cmake --version
```
如果你的版本低于3.21，请按照[官方指导](https://cmake.org/download/)升级你的`cmake`。


### 2.2 编译步骤

进入`TrajOpt`目录并建立一个`build`文件夹：

```bash
cd urbanplanning/TrajOpt
mkdir build && cd build
```

接下来，运行`cmake`和`make`来编译项目：

```bash
cmake ..
make -j6
```

## 3. 运行代码

在运行代码前，请确保已启动`roscore`，并打开`rviz`：

```bash
roscore
```

在另一个终端中，打开`rviz`并加载配置文件：
rviz显示需要安装 jsk_rviz_plugins插件

```bash
rviz -d urbanplanning/TrajOpt/Config/rviz.rviz
```

运行代码
```bahs
./planning
```

## 4.关于可视化：
在文件planner_node.cpp中，有两个可视化选项，分别是：
```c++
#define plot_flag 1 // matplotlib-cpp显示
#define rviz_flag 0 // rviz显示
```
## 5.包含场景
1. 定速巡航
2. 超车、跟车、变道、减速等
3. 避让(动静态障碍物)
4. 斑马线
5. 红绿灯 
