# plan May 28th

- mounting plate打印

  参考：https://github.com/mit-racecar/hardware 内含3D模型文件可能可以修改以后使用

  由于我们测试阶段不了解之后最终版的布局，所以可以打印一个具有孔位矩阵的mounting plate，之后的元件暂时用扎带固定；mounting plate通过车架底盘两侧最外部的各4个孔共8个孔用螺丝固定

- 舵机测试

  旧舵机有问题，新舵机还没有拿到，拿到以后测试完成用树莓派控制舵机转动角度部分

- 电路示意

  ```mermaid
  graph TD
  	A[LiPo 3S]
  	A --> B[电机]
  	A --> C[可调稳压模块]
  	C --5V--> D[树莓派]
  	D --> E[舵机]
  	D -.控制.->B
  ```

  注意可调稳压模块配有显示，用螺丝刀调节其电位器使其输出为5V；

  缺USB type-C供电线，要用USB type-C头焊接，另一头接稳压

  稳压输入的线另一头接XT30U-F头（缺）

  缺一条两头都是XT60-M的线连接一分为二的线

- 将元件都固定在车上，先实现开环控制

  目标是将[阿克曼底盘信息](http://wiki.ros.org/ackermann_msgs)解算成舵机和电机控制信号

  参考：https://github.com/mit-racecar/vesc/tree/master/vesc_ackermann

  为了便于测试我们还需要从键盘读入信息并发布到相应话题的程序，这部分可以参考MIT的包（[模拟部分](https://github.com/mit-racecar/racecar_gazebo)无法单独运行，需要[车辆底层控制](https://github.com/mit-racecar/racecar)部分），为了方便我把目前能运行的仿真部分上传到这里，按照readme操作即可，然后理解其代码部分并改装到我们的车上。





