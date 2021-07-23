#include "mbot_linux_serial.h"
#include "ros/ros.h"

using namespace std;
using namespace boost::asio;

void downcastCallback(const std_msgs::Float64MultiArray& downcast)
{
  writeSpeed(downcast.data.at(0), downcast.data.at(1), 0);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"mbot_linux_serial");
    ros::NodeHandle nh;
    ros::Subscriber c = nh.subscribe("down", 10, downcastCallback);
    ros::Rate loop_rate(100);
    //double v=0,th=0;
    //unsigned char cf=0;
    serialInit();
    while(ros::ok())
    {
        //writeSpeed(1000,90,0);
        //readSpeed(v,th,cf);
        //std_msgs::Float64 vf;
        //vf.data = v;
        //cout<<"vf="<<v<<endl;
        //c.publish(vf);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
