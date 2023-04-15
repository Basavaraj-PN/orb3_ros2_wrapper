#include "adapters.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class Monocular : public rclcpp::Node
{
public:
    Monocular(std::string node_name, std::string image_topic, Sophus::SE3f Tcw, ORB_SLAM3::System *pSLAM);

private:
    std::string image_topic;
    Sophus::SE3f Tcw;
    ORB_SLAM3::System *mpSLAM;
    std::string world_frame_id, cam_frame_id;
    Sophus::SE3f Twc;
    std::unique_ptr<CAMERA::ImageGrabber> imgGrab;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped camera_pose_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr orb_pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_publisher_;

    void timer_callback();
    void publish_pose();
    void T_camera_to_vehicle();
    void get_parameters();
};
