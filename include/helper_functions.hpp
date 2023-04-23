#ifndef HELPER_FUNCTION_HPP_
#define HELPER_FUNCTION_HPP_

#include "common.hpp"

/**
 * \brief Convert ROS timestamp to seconds.
 *
 * This function takes a reference to a ROS timestamp header and converts it to seconds.
 *
 * @param header The ROS timestamp header to be converted.
 * @return The converted timestamp in seconds.
 */
double timestamp_to_sec(const std_msgs::msg::Header &header);

/**
 * \brief Compute camera-to-vehicle transformation.
 *
 * This function takes two Sophus::SE3f transformations, Tcc0_SE3f and Tc0w_SE3f, and computes
 * the camera-to-vehicle transformation.
 *
 * @param Tcc_SE3f The transformation from camera-to-camera at time 0.
 * @param Tcw_SE3f The transformation from camera-to-world at time 0.
 * @return The camera-to-vehicle transformation.
 */
Sophus::SE3f transform_camera_to_vehicle(Sophus::SE3f Tcc_SE3f, Sophus::SE3f Tcw_SE3f);

/**
 * \brief Update ROS camera pose message with translation and orientation.
 *
 * This function takes a Sophus::SE3f transformation Twc_SE3f and updates a ROS camera pose message
 * with its translation and orientation.
 *
 * @param Twc_SE3f The camera-to-world transformation.
 * @param pose_in The ROS camera pose message to be updated.
 */
void SE3f_to_geometry_msg_stamped(Sophus::SE3f Twc_SE3f, geometry_msgs::msg::PoseStamped &pose_in);

/**
 * \brief Publish ROS tf2 transform.
 *
 * This function publishes a ROS tf2 transform using a Sophus::SE3f transformation, frame_id,
 * child_frame_id, and message time.
 *
 * @param Twc_SE3f The camera-to-world transformation.
 * @param frame_id The parent frame ID for the tf2 transform.
 * @param child_frame_id The child frame ID for the tf2 transform.
 * @param msg_time The time stamp for the tf2 transform message.
 */
geometry_msgs::msg::Transform SE3f_to_geometry_msgs_transform(const Sophus::SE3f &T_SE3f);

// Function declaration to publish ROS tf2 transform
void publish_ros_tf_transform(const Sophus::SE3f &Twc_SE3f,
                              const std::string &frame_id,
                              const std::string &child_frame_id,
                              const rclcpp::Time &msg_time);

#endif
