# Plan & Progress

## Progress

### week 7

- 下载运行了MIT racecar_simulator代码，往里整合了pure pursuit轨迹跟踪算法后在rviz里进行了仿真

  仿真程序节点图：

  ![image-20210617164113764](readme.assets/image-20210617164113764.png)

  

  ![Screenshot from 2021-06-17 16-28-04](readme.assets/Screenshot from 2021-06-17 16-28-04.png)

  

- 购买了STM32板（周五会到），基本完成了底层控制部分代码（未调试）

## Plan

- 以STM32为下位机调好底层控制部分（对舵机的控制、对电机的转速反馈控制）
- 实现上下位机之间的通讯部分
- 了解阿克曼底盘赛车的路径规划部分，将路径规划算法整合进仿真包里
- 将仿真代码迁移到实车，利用动捕系统测试其跟踪效果和精度
