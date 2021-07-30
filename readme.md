# Plan & Progress

## Progress

### vacation week 3

理论部分：

- 精读了TARE论文

工程部分：

- 利用Logitech游戏手柄编写实现了遥控操控小车功能和手动自动控制切换功能
- 将主控从树莓派替换为妙算，进行了相应软件配置和硬件修改，成功在妙算上复现了在树莓派上实现的手动控制和轨迹跟踪功能
- 编写了Driving on point cloud中针对点云的RRT和RRT\*算法并进行了测试

Others:

- 配置了robomaster小车可以在动捕反馈情况下读取包含路径信息的txt文档并跟踪其中的轨迹

## Plan

- 完成Driving on point cloud中的Terrain Assesement和Local Trajectory Optimation部分代码编写和测试
- 配置并测试雷达，使ROS能从其获取点云信息
- 尝试在实车上复现TARE论文