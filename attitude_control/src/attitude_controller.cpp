#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <Eigen/Geometry>

using namespace std;

namespace attitude_controller {

class Node {
 public:
  explicit Node(const ros::NodeHandle& pnh);
  void setpoint_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg);

 private:
  ros::NodeHandle pnh_;
  ros::Subscriber setpoint_sub;
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh) {

    setpoint_sub = pnh_.subscribe("/attitude_target", 10, &Node::setpoint_cb, this);
    ROS_INFO("init attitude_controller");
}

void Node::setpoint_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
  ROS_INFO("setpoint recieved");
}

}  // namespace attitude_controller


int main(int argc, char** argv) {

  ros::init(argc, argv, "attitude_controller");
  ros::NodeHandle pnh("~");
  attitude_controller::Node node(pnh);
  ros::spin();
  return 0;
}
