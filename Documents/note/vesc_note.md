**vesc-master：vesc_ackermann：**

ackermann_to_vesc：（自行车模型的速度和转角$\longrightarrow$电机转速和舵机角度）

* 需要接收“ackermann_cmd”→AckermannDriveStamped
* 需要传入参数speed_to_erpm_gain、speed_to_erpm_offset、steering_angle_to_servo_gain、steering_angle_to_servo_offset
* 发布“commands/motor/speed”和“commands/servo/position”→Float64
* 根据阿克曼信息的速度和转角，分别乘以倍数并加上静态补偿输出
* PS：直接可用，或修改阿克曼信息为自定义信息，因为简易控制不需要更多信息？

vesc_to_odom：（电机转速反馈和上一次舵机指令$\longrightarrow$运动学模型$\longrightarrow$里程计信息）

* 需要接收"sensors/core"→VescStateStamped、"sensors/servo_position_command"→Float64
* 需要传入参数ackermann_to_vesc的四项、wheelbase（nh）和odom_frame、base_frame、use_servo_cmd_、publish_tf（private_nh）
* 发布“odom”→Odometry和tf信息
* 根据上一条舵机角度指令和电调状态中的电机转速反馈，根据运动学模型发布里程计信息，还可发布tf信息
* PS：直接可用，或修改电调状态为自定义信息，因为只用到了电机转速反馈？

TODO：driver：（电机转速和舵机角度$\longrightarrow$真的电压信号控制指令+反馈信息）

* 接收“commands/servo/position”和“commands/motor/speed”，并限幅输出真的电压信号控制指令，前者还将限幅后的控制指令作为"sensors/servo_position_command"话题发布
* 发布"sensors/core"，直接读取对应编码器信息并计算转速，封装为VescStateStamped发布
* PS：可与ackermann_to_vesc融合，因为我们的电调比较简单？



