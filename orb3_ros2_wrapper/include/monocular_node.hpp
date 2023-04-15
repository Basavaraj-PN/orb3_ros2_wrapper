#include "adapters.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class Monocular : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for Monocular class.
     * @param node_name The name of the ROS2 node.
     * @param image_topic The topic name for receiving sensor_msgs::msg::Image messages.
     * @param Tcw The initial camera-to-world transformation (Sophus::SE3f).
     * @param pSLAM Pointer to the ORB_SLAM3::System instance.
     */
    Monocular(std::string node_name, std::string image_topic, Sophus::SE3f Tcw, ORB_SLAM3::System *pSLAM);

private:
    std::string image_topic;
    Sophus::SE3f Tcw;
    ORB_SLAM3::System *mpSLAM;
    Sophus::SE3f Twc;

    // Parent frame or link where camera is connected
    std::string world_frame_id, cam_frame_id;
    std::unique_ptr<CAMERA::ImageGrabber> imgGrab{nullptr};
    geometry_msgs::msg::PoseStamped camera_pose_;
    std::shared_ptr<tf2_ros::Buffer> buffer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_{nullptr};

    // Subscribers and callbacks
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr orb_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformed_pose_publisher_;

    /**
     * @brief Timer callback function for publishing pose, tf, and transformed pose.
     */
    void timer_callback();
    /**
     * @brief Publishes the camera pose in the ORB_SLAM3 format.
     */
    void publish_pose();
    /**
     * \brief Compute camera-to-vehicle transformation.
     */
    void T_camera_to_vehicle();

    /**
     * @brief Gets the camera and world frame IDs from the ROS2 parameter server.
     */
    void get_parameters();
    /**
     * \brief Publish transformed camera pose.
     */
    void publish_transformed_pose();
};
