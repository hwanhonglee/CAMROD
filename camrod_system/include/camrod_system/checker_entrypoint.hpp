#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

// HH_260805 - Build a system node from one source as either a normal executable
// or a registered component. This preserves standalone field-debug commands
// while the production launch shares executors across related system nodes.
#ifdef CAMROD_SYSTEM_COMPONENT_BUILD
#include <rclcpp_components/register_node_macro.hpp>
#define CAMROD_SYSTEM_NODE_ENTRYPOINT(NodeType) \
  RCLCPP_COMPONENTS_REGISTER_NODE(NodeType)
#else
#define CAMROD_SYSTEM_NODE_ENTRYPOINT(NodeType) \
  int main(int argc, char ** argv) \
  { \
    rclcpp::init(argc, argv); \
    rclcpp::spin(std::make_shared<NodeType>()); \
    rclcpp::shutdown(); \
    return 0; \
  }
#endif

#define CAMROD_SYSTEM_CHECKER_ENTRYPOINT(NodeType) \
  CAMROD_SYSTEM_NODE_ENTRYPOINT(NodeType)
