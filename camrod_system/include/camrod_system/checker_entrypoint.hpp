#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

// HH_260805 - Build every checker from one source as either a normal executable
// or a registered component. This preserves standalone field-debug commands
// while the production launch shares executors across checker categories.
#ifdef CAMROD_SYSTEM_COMPONENT_BUILD
#include <rclcpp_components/register_node_macro.hpp>
#define CAMROD_SYSTEM_CHECKER_ENTRYPOINT(NodeType) \
  RCLCPP_COMPONENTS_REGISTER_NODE(NodeType)
#else
#define CAMROD_SYSTEM_CHECKER_ENTRYPOINT(NodeType) \
  int main(int argc, char ** argv) \
  { \
    rclcpp::init(argc, argv); \
    rclcpp::spin(std::make_shared<NodeType>()); \
    rclcpp::shutdown(); \
    return 0; \
  }
#endif
