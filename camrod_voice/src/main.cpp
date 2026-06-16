#include <rclcpp/rclcpp.hpp>
#include "voice_announcer/voice_announcer_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<voice_announcer::VoiceAnnouncerNode>());
  rclcpp::shutdown();
  return 0;
}
