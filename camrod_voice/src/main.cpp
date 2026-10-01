#include <rclcpp/rclcpp.hpp>
#include "voice_announcer/voice_announcer_node.hpp"

int main(int argc, char * argv[])
{
  // HH_260616 - Run the standalone announcer through the ROS executor.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<voice_announcer::VoiceAnnouncerNode>());
  rclcpp::shutdown();
  return 0;
}
