#include "monocular_node.hpp"
#include "helper_functions.hpp"

Monocular::Monocular(std::string node_name, std::string image_topic, Sophus::SE3f Tc0w, ORB_SLAM3::System *pSLAM)
    : Node(node_name), image_topic(image_topic), Tc0w(Tc0w), mpSLAM(pSLAM)
{
    // Read camera and world frame IDs from ROS parameter server

    this->get_parameters();
    imgGrab = std::make_unique<CAMERA::ImageGrabber>(mpSLAM);
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, 10, [this](const sensor_msgs::msg::Image::SharedPtr image)
        { imgGrab->GrabImage(image); });

    orb_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("o3_pose", 20);
    timer_ = this->create_wall_timer(
        33.33ms, std::bind(&Monocular::timer_callback, this));
}

void Monocular::get_parameters()
{
    this->declare_parameter<std::string>("camera_frame", "default/camera");
    this->declare_parameter<std::string>("parent_frame", "default/world");
    this->get_parameter("camera_frame", cam_frame_id);
    this->get_parameter("parent_frame", world_frame_id);
}
void Monocular::timer_callback()
{
    Twc = transform_camera_to_vehicle(imgGrab->Tcc0, Tc0w);
    publish_pose();
    publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, imgGrab->header.stamp);
}

void Monocular::publish_pose()
{
    camera_pose_.header = imgGrab->header;
    publish_ros_camera_pose(Twc, camera_pose_);
    orb_pose_publisher_->publish(camera_pose_);
}