#include <ros/ros.h>
#include <Eigen/Geometry>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>

using namespace std;

namespace attitude_controller {

class Node {
 public:
  explicit Node(const ros::NodeHandle& pnh);
  void setpoint_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
  void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);

 private:
  ros::NodeHandle pnh_;

  ros::Subscriber setpoint_sub_;
  ros::Time last_setpoint_time_;
  mavros_msgs::AttitudeTarget last_setpoint_;

  ros::Subscriber imu_sub_;
  Eigen::Quaterniond imu_q_;
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh) {
  setpoint_sub_ = pnh_.subscribe("/attitude_target", 10, &Node::setpoint_cb, this);
  imu_sub_ = pnh_.subscribe("/imu/data", 10, &Node::imu_cb, this);
  ROS_INFO("init attitude_controller");
}

void Node::imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
  ROS_INFO("imu recieved");
  imu_q_ = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

void Node::setpoint_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
  ROS_INFO("setpoint recieved");
  last_setpoint_ = *msg;
  last_setpoint_time_ = ros::Time::now();
}

}  // namespace attitude_controller


int main(int argc, char** argv) {
  ros::init(argc, argv, "attitude_controller");
  ros::NodeHandle pnh("~");
  attitude_controller::Node node(pnh);
  ros::spin();
  return 0;
}
