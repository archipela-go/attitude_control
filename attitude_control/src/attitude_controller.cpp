#include <ros/ros.h>
#include <Eigen/Geometry>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <kingfisher_msgs/Drive.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

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
  double throttle_;
  Eigen::Quaterniond setpoint_q_;
  bool setpoint_set_;

  ros::Publisher drive_pub_;
  ros::Publisher actuator_pub_;

  ros::Subscriber imu_sub_;
  Eigen::Quaterniond imu_q_;

  double kp_;
  double kd_;
  double ki_;

  double last_error_;
  ros::Time last_error_time_;
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh) {
  setpoint_sub_ = pnh_.subscribe("/att_control/attitude_target", 10, &Node::setpoint_cb, this);
  imu_sub_ = pnh_.subscribe("/mavros/imu/data", 10, &Node::imu_cb, this);
  drive_pub_ = pnh_.advertise<kingfisher_msgs::Drive>("/cmd_drive", 10);
  actuator_pub_ = pnh_.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);

  setpoint_set_ = false;

  kp_ = pnh_.param("kp", kp_, 1.0);
  kd_ = pnh_.param("kd", kd_, 1.0);
  ki_ = pnh_.param("ki", ki_, 1.0);

  last_error_ = 0.0;
  last_error_time_ = ros::Time::now();

  ROS_INFO("init attitude_controller");
}

void Node::imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
  ROS_INFO("imu recieved");
  imu_q_ = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  // perform control
  if (setpoint_set_){
    // convert to tf::Quaternions
    tf::Quaternion imu_tf =
           tf::Quaternion(imu_q_.x(), imu_q_.y(), imu_q_.z(), imu_q_.w());
    tf::Quaternion setpoint_tf =
             tf::Quaternion(setpoint_q_.x(), setpoint_q_.y(), setpoint_q_.z(), setpoint_q_.w());

    // get RPY
    double imu_roll, imu_pitch, imu_yaw;
    double setpoint_roll, setpoint_pitch, setpoint_yaw;
    tf::Matrix3x3(imu_tf).getRPY(imu_roll, imu_pitch, imu_yaw);
    tf::Matrix3x3(setpoint_tf).getRPY(setpoint_roll, setpoint_pitch, setpoint_yaw);
    ROS_INFO_STREAM("imu yaw: " << imu_yaw);
    ROS_INFO_STREAM("setpoint yaw: " << setpoint_yaw);

    // calculate control effort
    double error = setpoint_yaw - imu_yaw;
    double d_error = (error - last_error_)/(ros::Time::now() - last_error_time_).toSec();
    double yaw_effort = 1.0 * error + 0.7 * d_error;
    last_error_ = error;
    last_error_time_ = ros::Time::now();

    // clamp yaw
    yaw_effort = std::min(1.0, yaw_effort);
    yaw_effort = std::max(-1.0, yaw_effort);

    // clamp throttle
    double throttle = throttle_;
    throttle = std::min(1.0, throttle);
    throttle = std::max(-1.0, throttle);

    ROS_INFO_STREAM("yaw effort: " << yaw_effort);
    ROS_INFO_STREAM("throttle: " << throttle);

    double left = throttle - yaw_effort;
    double right = throttle + yaw_effort;

    // clamp left
    left = std::min(1.0, left);
    left = std::max(-1.0, left);

    // clamp right
    right = std::min(1.0, right);
    right = std::max(-1.0, right);

    // publish motor command
    auto drive_msg = boost::make_shared<kingfisher_msgs::Drive>();
    drive_msg->left = left;
    drive_msg->right = right;

    drive_pub_.publish(drive_msg);

    // create array for publishing

    float controls[] = {0, 1, 2, 3, 4, 5, 6, 7};

    // publish actuator control
    auto control_msg = boost::make_shared<mavros_msgs::ActuatorControl>();
    control_msg->group_mix = 0;
    control_msg->header.stamp = ros::Time::now();
    control_msg->controls[1] = throttle;
    control_msg->controls[2] = yaw_effort;

    if ((ros::Time::now() - last_setpoint_time_).toSec() < 0.5)
      actuator_pub_.publish(control_msg);
  }
}

void Node::setpoint_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
  setpoint_set_ = true;
  ROS_INFO("setpoint recieved");
  last_setpoint_ = *msg;
  setpoint_q_ = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  throttle_ = msg->thrust;
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
