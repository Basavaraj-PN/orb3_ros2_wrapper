#ifndef HELPER_FUNCTION_HPP_
#define HELPER_FUNCTION_HPP_

#include "common.hpp"

double timestamp_to_sec(const std_msgs::msg::Header &header);

Sophus::SE3f transform_camera_to_vehicle(Sophus::SE3f Tcc0_SE3f, Sophus::SE3f Tc0w_SE3f);

void publish_ros_camera_pose(Sophus::SE3f Twc_SE3f, geometry_msgs::msg::PoseStamped &pose_in);

geometry_msgs::msg::Transform SE3f_to_geometry_msgs_transform(const Sophus::SE3f &T_SE3f);

void publish_ros_tf_transform(const Sophus::SE3f &Twc_SE3f,
                              const std::string &frame_id,
                              const std::string &child_frame_id,
                              const rclcpp::Time &msg_time);

#endif