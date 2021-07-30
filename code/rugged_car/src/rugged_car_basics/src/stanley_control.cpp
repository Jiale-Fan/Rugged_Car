#include "stanley_control.h"

stanley_control::stanley_control()
	: delta_s(0.01), state{0, 0, 0, 0}, last_target(-1), target(0), target_speed(0.5), k(0.5), Kp(10.0), dt(1.0 / 50), L(0.325), control_begin(0.0), loop_rate(50)
{
	nh = ros::NodeHandle("~");

	string drive_topic, down_topic, refined_path_topic, global_path_topic, odom_topic;
	
	nh.getParam("drive_topic", drive_topic);
	// nh.getParam("down_topic", down_topic);
	nh.getParam("global_path_topic", global_path_topic);
	nh.getParam("path_topic", refined_path_topic);
	nh.getParam("frame_id", frame_id);
	nh.getParam("odom_topic", odom_topic);

	end_index = - 1;
	// pub = nh.advertise<std_msgs::Float64MultiArray>(down_topic, 100);
	acm_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 100);
	path_pub = nh.advertise<nav_msgs::Path>(refined_path_topic, 100);

	control_pub = ros::Time::now().toSec();
	// odometry subscriber
	odom_sub = nh.subscribe(odom_topic, 1, &stanley_control::update_state, this);
	path_sub = nh.subscribe(global_path_topic, 1, &stanley_control::path_callback, this);

	ROS_INFO("Stanley controller constructed!");
}

void stanley_control::path_callback(nav_msgs::Path path_msg)
{
	// TODO:
	vector<double> x, y;
	int n;
	n = path_msg.poses.size();
	for (int i = 0; i < n; i++)
	{
		x.push_back(path_msg.poses[i].pose.position.x);
		y.push_back(path_msg.poses[i].pose.position.y);
	}

	// ROS_INFO("Path callback function called!");
	cubic_spline_planner C(x, y, delta_s);
	path = C.get_path();

	end_index = path.size() - 1;

	// ROS_INFO("Path callback function called!");

	publishPath(path);
}

void stanley_control::publishPath(vector<Vector3d> path)
{
	// string frame("/map");
	nav_msgs::Path pm;
	pm.header.frame_id = frame_id;
	geometry_msgs::PoseStamped ps;
	ps.header.frame_id = frame_id;
	int count = path.size();
	for (int i = 0; i < count; i++)
	{
		ps.pose.position.x = path[i][0];
		ps.pose.position.y = path[i][1];
		ps.pose.position.z = 0;
		pm.poses.push_back(ps);
	}
	// for (int i = 0; i < 50;i++)
	// {
	//     path_pub.publish(pm);
	// }
	path_pub.publish(pm);
	// ROS_INFO("Path published!");
}

// double down_delta = 0.0;
// double down_v = 0.0;

// void stanley_control::update_state(void)
// {
// 	double down_v(0.0), down_delta(0.0);
// 	unsigned char cf = 0;
// 	readSpeed(down_v, down_delta, cf);
// 	down_v /= 5394.9;
// 	down_delta = (down_delta - 96) / 70.43;

// 	state.x += state.v * cos(state.yaw)*dt;
// 	state.y += state.v * sin(state.yaw)*dt;
// 	state.yaw += state.v / L * tan(down_delta) * dt;
// 	state.yaw = normalize_angle(state.yaw);
// 	state.v = down_v;
// 	//cout << down_v << " ";
// }

void stanley_control::update_state(const nav_msgs::Odometry &odom_msg)
{
	// cout<<"update called"<<endl;

	state.x = odom_msg.pose.pose.position.x;
	state.y = odom_msg.pose.pose.position.y;

	tf::Quaternion temp;
	tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, temp);
	double roll, pitch, yaw;
	tf::Matrix3x3(temp).getRPY(roll, pitch, yaw);
	state.yaw = stanley_control::normalize_angle(yaw);
	state.v = odom_msg.twist.twist.linear.x;
	// cout << down_v << " ";
}

double stanley_control::normalize_angle(double angle)
{
	while (angle > PI)
	{
		angle -= 2.0 * PI;
	}

	while (angle <= -PI)
	{
		angle += 2.0 * PI;
	}

	return angle;
}

void stanley_control::control(void)
{
	// get current state of car
	control_begin = ros::Time::now().toSec();
	dt = control_begin - control_pub;
	// update_state();

	// get target and error
	double fx = state.x + L * cos(state.yaw);
	double fy = state.y + L * sin(state.yaw);
	double dist = std::numeric_limits<double>::infinity();

	if(!path.empty())
	{/// get target through traverse search
		
		for (int i = 0; i < path.size(); i++)
		{
			Vector3d p(path[i]);
			double d = sqrt((fx - p[0]) * (fx - p[0]) + (fy - p[1]) * (fy - p[1]));
			if (d < dist)
			{
				dist = d;
				target = i;
			}
		}
		/// get error
		double error = (fx - path[target][0]) * sin(state.yaw) - (fy - path[target][1]) * cos(state.yaw);
		if (last_target >= target)
			target = last_target;
		if (target == end_index && target == last_target)
			target_speed = 0;

		// stanley_control
		double theta_e = normalize_angle(path[target][2] - state.yaw);
		double theta_d = atan2(k * error, state.v);
		double delta = min(max_steer, max(-max_steer, theta_e + theta_d));
		double v = state.v + P_control(target_speed) * dt;
		last_target = target;

		// publish
		std_msgs::Float64MultiArray command;
		if (target_speed == 0)
		{
			v = 0;
			delta = 0;
		}
		// command.data.push_back(v * 5394.9);
		// command.data.push_back(delta * 70.43 + 96);
		// pub.publish(command);
		control_pub = ros::Time::now().toSec();

		publishAckermann(v, delta);

		// publish path
		publishPath(path);
	}
	else
	{
		// ROS_INFO("Empty path!");
	}
}

void stanley_control::publishAckermann(double v, double steering_angle)
{
	ackermann_msgs::AckermannDriveStamped acm_msg;
	acm_msg.header.frame_id = frame_id;
	acm_msg.header.stamp = ros::Time();
	acm_msg.header.seq = 0; // for convenience
	acm_msg.drive.speed = v;
	acm_msg.drive.steering_angle = steering_angle;
	acm_pub.publish(acm_msg);
}

int main(int argc, char **argv)
{
	// cout<<"no problem"<<endl;
	ros::init(argc, argv, "stanley_controller");

	stanley_control Stan_instance;

	// cout<<"no problem"<<endl;

	// Stan->test();
	// Stan_instance.test();

	while (ros::ok())
	{
		Stan_instance.control();
		ros::spinOnce();
		Stan_instance.loop_rate.sleep();
	}
	return 0;
}

// void stanley_control::test()
// {
	
// 	vector<double> x{0.0, 5.0, 5.0, 2.5, 3.0};
// 	vector<double> y{0.0, 0.0, -1.5, -1.0, 0.0};
// 	ros::Publisher path_pub;
// 	path_pub = nh.advertise<nav_msgs::Path>("/rugged_car/tracking/path", 100);
// 	// string frame("/map");
// 	nav_msgs::Path pm;
// 	pm.header.frame_id = "/map";
// 	geometry_msgs::PoseStamped ps;
// 	ps.header.frame_id = "/map";
// 	for (int i = 0; i < x.size(); i++)
// 	{
// 		ps.pose.position.x = x[i];
// 		ps.pose.position.y = y[i];
// 		ps.pose.position.z = 0;
// 		pm.poses.push_back(ps);
// 	}
// 	// for (int i = 0; i < 50;i++)
// 	// {
// 	//     path_pub.publish(pm);
// 	// }
	
// 	path_pub.publish(pm);
// 	// ROS_INFO("Path published!");s

// }

// int main(int argc, char **argv)
// {
// 	cout<<"no problem"<<endl;
// }
