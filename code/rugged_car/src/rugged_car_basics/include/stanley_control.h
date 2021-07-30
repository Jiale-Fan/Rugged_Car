#pragma once
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include "cubic_spline_planner.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
// #include <time.h>
//#include <nav_msgs/Odometry.h>
// #include "mbot_linux_serial.h"

#define PI 3.1415926

using namespace std;
using namespace Eigen;

struct State
{
	double x, y, yaw, v;
};

class stanley_control
{
private:
	
	vector<Vector3d> path;
	int last_target;
	int target;
	double target_speed;
	double k;
	double Kp;
	double dt;
	double L;
	double max_steer = 25.0 / 180 * PI;
	double control_pub;
	double control_begin;
	double delta_s;

	string frame_id;
public:

	// static stanley_control &Stan;

    State state;
    int end_index;
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Publisher acm_pub;
	ros::Publisher path_pub;
	ros::Subscriber sub;
	ros::Subscriber odom_sub;
	ros::Subscriber path_sub;
	ros::Rate loop_rate;
	stanley_control();
	// void update_state(void);
	// void update_state(const nav_msgs::Odometry & odom_msg);
	double P_control(double target) { return Kp * (target - state.v); }
	static double normalize_angle(double angle);
	void control(void);
	bool done(void) { return path.size()-1 <= target; }
	~stanley_control(){}

	void publishAckermann(double v, double steering_angle);
	void publishPath(vector<Vector3d> path);
	void path_callback(nav_msgs::Path path_msg);
	// void test();
	void update_state(const nav_msgs::Odometry &odom_msg);
};

void update_state(const nav_msgs::Odometry & odom_msg);

