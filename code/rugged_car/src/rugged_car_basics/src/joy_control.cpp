#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#define MANUAL_MODE 0
#define AUTO_MODE 1

class JoystickControl
{
private:
  double max_speed, max_steering_angle;
  double speed, steering_angle;
  double update_cmd_rate;
  int mode = MANUAL_MODE;

  // Joystick parameters
  int joy_speed_axis, joy_angle_axis, joy_manual_button, joy_auto_button;
  double joy_max_speed;

  // Control parameters
  float speed_k, steering_k, steering_b;

  // A ROS node
  ros::NodeHandle n;

  // A timer to update the pose
  ros::Timer update_pose_timer;

  // Listen for drive and joystick commands
  ros::Subscriber drive_sub;
  ros::Subscriber joy_sub;
  ros::Publisher cmd_pub;
  ros::Publisher acm_pub;

public:
  JoystickControl()
  {
    // Initialize the node handle
    n = ros::NodeHandle("~");

    // Get the topic names
    std::string joy_topic, acm_topic, drive_topic;
    n.getParam("joy_topic", joy_topic);
    n.getParam("drive_topic", drive_topic);

    n.getParam("max_speed", max_speed);
    n.getParam("max_steering_angle", max_steering_angle);

    // Get joystick parameters
    bool joy;
    n.getParam("joy", joy);
    n.getParam("joy_speed_axis", joy_speed_axis);
    n.getParam("joy_angle_axis", joy_angle_axis);
    n.getParam("joy_max_speed", joy_max_speed);
    n.getParam("joy_manual_button", joy_manual_button);
    n.getParam("joy_auto_button", joy_auto_button);

    n.getParam("update_pose_rate", update_cmd_rate);
    n.getParam("ackermann_topic", acm_topic);

    // Get control parameters
    n.getParam("speed_k", speed_k);
    n.getParam("steering_k", steering_k);
    n.getParam("steering_b", steering_b);

    // Start a timer to output the pose
    update_pose_timer = n.createTimer(ros::Duration(update_cmd_rate), &JoystickControl::update_cmd, this);

    // If the joystick is enabled
    if (joy)
      // Start a subscriber to listen to joystick commands
      joy_sub = n.subscribe(joy_topic, 1, &JoystickControl::joy_callback, this);

    drive_sub = n.subscribe(drive_topic, 1, &JoystickControl::drive_callback, this);
    cmd_pub = n.advertise<std_msgs::Float64MultiArray>("/down", 10);
    acm_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(acm_topic, 100);
    /*************************************************************************************/
    // std::cout<<"Publishers and subscribers set!"<<std::endl;
  }

  void drive_callback(ackermann_msgs::AckermannDriveStamped msg)
  {
    if (mode == AUTO_MODE)
    {
      set_speed(msg.drive.speed);
      set_steering_angle(
          msg.drive.steering_angle,
          ros::Time::now());
    }
  }

  void update_cmd(const ros::TimerEvent &)
  {
    // publish
    std_msgs::Float64MultiArray command;
    command.data.push_back(speed * speed_k);
    command.data.push_back(steering_angle * steering_k + steering_b);
    cmd_pub.publish(command);
    //publishAckermann(speed, steering_angle);
  }

  void publishAckermann(double v, double steering_angle)
  {
    ackermann_msgs::AckermannDriveStamped acm_msg;
    acm_msg.header.frame_id = "/map";
    acm_msg.header.stamp = ros::Time();
    acm_msg.header.seq = 0; // for convenience
    acm_msg.drive.speed = v;
    acm_msg.drive.steering_angle = steering_angle;
    acm_pub.publish(acm_msg);
  }

  void joy_callback(const sensor_msgs::Joy &msg)
  {
    // std::cout<<"joystick callback!"<<std::endl;
    if (msg.buttons[joy_manual_button])
    {
      mode = MANUAL_MODE;
      ROS_INFO("Switched to manual control.");
    }
    else if (msg.buttons[joy_auto_button])
    {
      mode = AUTO_MODE;
      ROS_INFO("Switched to auto control.");
    }

    if (mode == MANUAL_MODE)
    {
      set_speed(
          joy_max_speed * msg.axes[joy_speed_axis]);
      set_steering_angle(
          max_steering_angle * msg.axes[joy_angle_axis],
          ros::Time::now());
    }
  }

  void set_speed(double speed_)
  {
    speed = std::min(std::max(speed_, -max_speed), max_speed);
  }

  void set_steering_angle(double steering_angle_, ros::Time timestamp)
  {
    steering_angle = std::min(std::max(steering_angle_, -max_steering_angle), max_steering_angle);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_control");
  // std::cout<<"joystick node run!"<<std::endl;
  JoystickControl rs;
  ros::spin();
  return 0;
}
