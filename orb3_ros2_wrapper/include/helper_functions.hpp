#ifndef HELPER_FUNCTION_HPP_
#define HELPER_FUNCTION_HPP_

#include "common.hpp"

// Function declaration to convert ROS timestamp to seconds
double timestamp_to_sec(const std_msgs::msg::Header &header);

// Function declaration to compute camera-to-vehicle transformation
Sophus::SE3f transform_camera_to_vehicle(Sophus::SE3f Tcc0_SE3f, Sophus::SE3f Tc0w_SE3f);

// Function declaration to update ROS camera pose message with translation and orientation
void publish_ros_camera_pose(Sophus::SE3f Twc_SE3f, geometry_msgs::msg::PoseStamped &pose_in);

// Function declaration to convert Sophus::SE3f to ROS geometry_msgs::msg::Transform
geometry_msgs::msg::Transform SE3f_to_geometry_msgs_transform(const Sophus::SE3f &T_SE3f);

// Function declaration to publish ROS tf2 transform
void publish_ros_tf_transform(const Sophus::SE3f &Twc_SE3f,
                              const std::string &frame_id,
                              const std::string &child_frame_id,
                              const rclcpp::Time &msg_time);

#endif
