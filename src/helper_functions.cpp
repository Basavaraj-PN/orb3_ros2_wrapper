#include "helper_functions.hpp"

double
timestamp_to_sec(const std_msgs::msg::Header &header)
{
    // Access the timestamp field in the header
    rclcpp::Time stamp(header.stamp);
    // Convert the timestamp to seconds as a double value
    double stamp_sec = stamp.seconds();
    return stamp_sec;
}

Sophus::SE3f
transform_camera_to_vehicle(Sophus::SE3f Tcc_SE3f, Sophus::SE3f Tcw_SE3f)
{
    return (Tcc_SE3f * Tcw_SE3f).inverse();
}

void SE3f_to_geometry_msg_stamped(Sophus::SE3f Twc_SE3f, geometry_msgs::msg::PoseStamped &pose_in)
{
    pose_in.pose.position.x = Twc_SE3f.translation().x();
    pose_in.pose.position.y = Twc_SE3f.translation().y();
    pose_in.pose.position.z = Twc_SE3f.translation().z();
    pose_in.pose.orientation.w = Twc_SE3f.unit_quaternion().coeffs().w();
    pose_in.pose.orientation.x = Twc_SE3f.unit_quaternion().coeffs().x();
    pose_in.pose.orientation.y = Twc_SE3f.unit_quaternion().coeffs().y();
    pose_in.pose.orientation.z = Twc_SE3f.unit_quaternion().coeffs().z();
}

geometry_msgs::msg::Transform
SE3f_to_geometry_msgs_transform(const Sophus::SE3f &T_SE3f)
{
    Eigen::Matrix3f R_mat = T_SE3f.rotationMatrix();
    Eigen::Vector3f t_vec = T_SE3f.translation();
    geometry_msgs::msg::Transform transform;
    transform.translation.x = t_vec(0);
    transform.translation.y = t_vec(1);
    transform.translation.z = t_vec(2);
    Eigen::Quaternionf q(R_mat);
    transform.rotation.x = q.x();
    transform.rotation.y = q.y();
    transform.rotation.z = q.z();
    transform.rotation.w = q.w();
    return transform;
}
void publish_ros_tf_transform(const Sophus::SE3f &Twc_SE3f,
                              const std::string &frame_id,
                              const std::string &child_frame_id,
                              const rclcpp::Time &msg_time)
{
    static tf2_ros::TransformBroadcaster tf_broadcaster(
        rclcpp::Node::make_shared("tf_broadcaster"));
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = msg_time;
    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame_id;
    transformStamped.transform = SE3f_to_geometry_msgs_transform(Twc_SE3f);
    tf_broadcaster.sendTransform(transformStamped);
}
