#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>

void setpoint_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
  ROS_INFO("setpoint recieved");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "attitude_control_node");
    ros::NodeHandle pnh("~");

    ros::Subscriber setpoint_sub = pnh.subscribe("attitude_target", 1000, setpoint_cb);

    ros::spin();
    return 0;
}
