## README

## joystick control and tracking

give the permission first:

```
sudo chmod 777 /dev/ttyUSB0
```

launch:

```
...
roslaunch rugged_car_basics joystick_control.launch
```

this would launch joystick control node, stanley control node which would listen to path message, and realsense node.

We use a logitech F710 joystick to control the car manually. Push X to change to manual control mode(default), push Y to switch to auto control.

## base_control

if you want to give the velocity and steering angle of the car,
you should follow the steps below:

first: wiring the stm32 with encoder, usb2ch340, power, motor, servo
       wiring the Raspberry Pie with stm32, servo, power, screen(optional)

second:  run stm32(only need to connect with power)
         run Raspberry Pie with commands in third step

third: cd ~\Desktop\code\rugged_car
       source devel\setup.bash
       roscore
     [open another terminal]
       rosrun rugged_car_basics mbot_linux_serial
     [open another terminal]
       rostopic pub /down Float64MultiArray "data: [velocity, angle]"
   or: create another node which can publish the topic above