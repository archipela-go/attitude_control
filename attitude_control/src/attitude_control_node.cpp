#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "attitude_control_node");
    ros::NodeHandle pnh("~");
    ros::spin();
    return 0;
}
