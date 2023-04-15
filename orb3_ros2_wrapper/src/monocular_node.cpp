#include "monocular_node.hpp"
#include "helper_functions.hpp"

Monocular::Monocular(std::string node_name, std::string image_topic, Sophus::SE3f Tcw, ORB_SLAM3::System *pSLAM)
    : Node(node_name), image_topic(image_topic), Tcw(Tcw), mpSLAM(pSLAM)
{

    this->get_parameters();
    buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    imgGrab = std::make_unique<CAMERA::ImageGrabber>(mpSLAM);

    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, 10, [this](const sensor_msgs::msg::Image::SharedPtr image)
        { imgGrab->GrabImage(image); });

    orb_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("orb3/camera/pose", 20);
    transformed_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("orb3/transformed/pose", 20);

    timer_ = this->create_wall_timer(
        33.33ms, std::bind(&Monocular::timer_callback, this));
}

void Monocular::get_parameters()
{
    // Read camera and world frame IDs from  parameter server
    this->declare_parameter<std::string>("camera_frame", "default/camera");
    this->declare_parameter<std::string>("parent_frame", "default/world");
    this->get_parameter("camera_frame", cam_frame_id);
    this->get_parameter("parent_frame", world_frame_id);
}
void Monocular::timer_callback()
{
    // Transform from camera to vehicle coordinate
    Twc = transform_camera_to_vehicle(imgGrab->Tcc, Tcw);

    // publish pose, camera pose w.r.t vehicle
    publish_pose();

    // Publish transformation between world_frame_id and cam_frame_id
    publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, imgGrab->header.stamp);

    // calculate camera pose in map frame and publish
    publish_transformed_pose();
}

void Monocular::publish_pose()
{
    camera_pose_.header = imgGrab->header;
    publish_ros_camera_pose(Twc, camera_pose_);
    orb_pose_publisher_->publish(camera_pose_);
}

void Monocular::publish_transformed_pose()
{
    try
    {
        // Look up the transform from "map" to "camera" at the latest time available
        geometry_msgs::msg::TransformStamped transform = buffer_->lookupTransform("map", "camera", tf2::TimePointZero);

        // Create a pose stamped message in "map" frame
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = transform.header.stamp;
        pose.header.frame_id = "map";
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;

        // Publish the transformed pose
        transformed_pose_publisher_->publish(pose);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "Failed to transform pose: %s", ex.what());
    }
}