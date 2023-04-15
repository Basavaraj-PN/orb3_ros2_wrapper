#include "mono.hpp"
#include <cstdio>
#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::MONOCULAR;
  ORB_SLAM3::System SLAM("src/orb3_wrapper/config/ORBvoc.txt", "src/orb3_wrapper/config/realsense_d435i.yaml", sensor_type, true);
  rclcpp::spin(std::make_shared<Monocular>("monocular_node", "/carla/ego_vehicle/rgb_front/image", Sophus::SE3f(), &SLAM));
  rclcpp::shutdown();
  return 0;
}
