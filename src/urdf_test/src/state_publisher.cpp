#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <stddef.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define WHEEL_R 0.05 //wheel radius
#define HALF_CAR_L 0.085 //distance from the middle of the car to wheel origin
#define TRAJECTORY_R 2 //trajectory circle radius

class StatePublisher : public rclcpp::Node {
public:
  StatePublisher() : Node("state_publisher"), rate_(30) {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO(this->get_logger(), "%s started", this->get_name());
    auto odom_trans = new geometry_msgs::msg::TransformStamped();
    odom_trans->header.frame_id = "odom";
    odom_trans->child_frame_id = "base_link";
    auto joint_state = new sensor_msgs::msg::JointState();
    rclcpp::Time now;

    try {
      while (rclcpp::ok()) {
        now = this->get_clock()->now();

        odom_trans->header.stamp = now;
        odom_trans->transform.translation.x = TRAJECTORY_R*cos(angle);
        odom_trans->transform.translation.y = TRAJECTORY_R*sin(angle);
        odom_trans->transform.translation.z = WHEEL_R;
        odom_trans->transform.rotation = eulerToQuaternion(0, 0, angle);

        joint_state->header.stamp = now;
        joint_state->name = {"chassis_to_l_wheel", "chassis_to_r_wheel"};
        // calculated using arc length formula s = theta*r twice
        // once to calculate the arc length of the trajectory of each wheel (s_wheel = angle*(traj_r+-half_car_r))
        // and once to calculate the corresponding wheel angle change (theta_wheel = s_wheel/wheel_r)
        joint_state->position = {angle*(TRAJECTORY_R-HALF_CAR_L)/WHEEL_R, angle*(TRAJECTORY_R+HALF_CAR_L)/WHEEL_R};

        publisher_->publish(*joint_state);
        tf_broadcaster_->sendTransform(*odom_trans);

        angle += (M_PI/180) / 4; // 1/4 degrees in rad

        rate_.sleep();
      }
    }
    catch(const std::exception& e) {}
    
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Rate rate_;
  float angle;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::Quaternion eulerToQuaternion(float roll, float pitch, float yaw);
};

geometry_msgs::msg::Quaternion StatePublisher::eulerToQuaternion(float roll, float pitch, float yaw) {
    float qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    float qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    float qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    float qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
    return tf2::toMsg(tf2::Quaternion(qx, qy, qz, qw));
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatePublisher>());
  rclcpp::shutdown();
  return 0;
}
