#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/rclcpp.hpp>
class TfListener : public rclcpp::Node
{
private:
    // Subscription to pose published by sensor node
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    // Listener for the broadcast transform message
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // Buffer that stores several seconds of transforms for easy lookup by the listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // Pose in source frame (`sensor_link`)
    geometry_msgs::msg::PoseStamped pose_in_;
    // Pose in target frame (`arm_end_link`)
    geometry_msgs::msg::PoseStamped pose_out_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformed_pose_publisher_;
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        try
        {
            pose_in_ = *msg;
            // Transforms the pose between the source frame and target frame
            tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(pose_in_, pose_out_, "map",
                                                                   tf2::Duration(std::chrono::seconds(1)));
            // Log coordinates of pose in target frame
            RCLCPP_INFO(get_logger(), "Object pose in 'map' is:\n x,y,z = %.1f,%.1f,%.1f",
                        pose_out_.pose.position.x,
                        pose_out_.pose.position.y,
                        pose_out_.pose.position.z);
            transformed_pose_publisher_->publish(pose_out_);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not find object position in 'map' frame.");
            return;
        }
    }

public:
    TfListener(const std::string &name) : Node(name)
    {
        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // Listen to the buffer of transforms
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        // Subscribe to pose published by sensor node (check every second)
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/orb3/camera/pose", 10,
                                                                         std::bind(&TfListener::poseCallback, this, std::placeholders::_1));
        transformed_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("orb3/camera/transformed/pose", 20);
    }
    ~TfListener() {}
};
int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfListener>("tf_listener");
    rclcpp::spin(node);
    return 0;
}
