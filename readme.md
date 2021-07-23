# Plan & Progress

## Progress

### vacation week 2

理论部分：

- 阅读了Driving on Point Cloud，代码未开源，已经开始进行复现
- 阅读了RSS论文TARE，下载了cmu的exploration development environment和TARE源代码，跑通了仿真

工程部分：

- 安装了T265相机，通过其获得Odometry
- 测试了小车线速度与电机转速比
- 利用T265的反馈信息完成了Stanley Control的实物测试，效果良好
- 将Stanley control整合进入了仿真环境

Others:

- 配置了用于控制robomaster小车的妙算环境，整理了相关代码

## Plan

- 使robomaster小车可以在动捕反馈和地图已知情况下完成路径规划和轨迹跟踪（周二前）
- 继续复现Driving on Point Cloud，完成其中的RRT、RRT*算法
- 利用Logitech游戏手柄实现遥控操控小车功能，并且编写小车收到命令的优先级结构
- 继续查找和阅读多机器人联合探索，尤其是地空联合探索论文
