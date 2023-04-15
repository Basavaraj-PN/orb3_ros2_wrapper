#include "monocular_node.hpp"
#include <cstdio>
#include <memory>

std::string ORBvoc_file;
std::string camera_params_file;
std::string image_topic;
bool enable_pangolin;

void readParametersFromParameterServer()
{
  auto node = rclcpp::Node::make_shared("parameter_node"); // Create a ROS2 node

  // Declare parameters for ORBvoc.txt and camera_params.yaml with default values
  node->declare_parameter<std::string>("ORBvoc_file", "ORBvoc.txt");
  node->declare_parameter<std::string>("camera_params_file", "camera_params.yaml");
  node->declare_parameter<bool>("enable_pangolin", true);
  node->declare_parameter<std::string>("image_topic", "/carla/ego_vehicle/rgb_front/image");

  // Get parameter values from the parameter server

  if (node->get_parameter("ORBvoc_file", ORBvoc_file))
  {
    RCLCPP_INFO(node->get_logger(), "Loaded ORBvoc_file: %s", ORBvoc_file.c_str());
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get ORBvoc_file parameter, using default value");
    rclcpp::shutdown();
  }

  if (node->get_parameter("camera_params_file", camera_params_file))
  {
    RCLCPP_INFO(node->get_logger(), "Loaded camera_params_file: %s", camera_params_file.c_str());
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get camera_params_file parameter, using default value");
    rclcpp::shutdown();
  }

  if (node->get_parameter("image_topic", image_topic))
  {
    RCLCPP_INFO(node->get_logger(), "image_topic: %s", image_topic.c_str());
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get image_topic parameter, using default value");
    rclcpp::shutdown();
  }

  if (node->get_parameter("enable_pangolin", enable_pangolin))
  {
    RCLCPP_INFO(node->get_logger(), "Loaded enable_pangolin: %s", enable_pangolin ? "true" : "false");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get enable_pangolin parameter, using default value");
    rclcpp::shutdown();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  readParametersFromParameterServer();

  ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::MONOCULAR;
  ORB_SLAM3::System SLAM(ORBvoc_file, camera_params_file, sensor_type, enable_pangolin);
  rclcpp::spin(std::make_shared<Monocular>("monocular_node", image_topic, Sophus::SE3f(), &SLAM));
  rclcpp::shutdown();
  return 0;
}