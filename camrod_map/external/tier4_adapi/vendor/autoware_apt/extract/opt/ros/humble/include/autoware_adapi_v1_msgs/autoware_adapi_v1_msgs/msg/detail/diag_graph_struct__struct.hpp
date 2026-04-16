// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagGraphStruct.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_GRAPH_STRUCT__STRUCT_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_GRAPH_STRUCT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'nodes'
#include "autoware_adapi_v1_msgs/msg/detail/diag_node_struct__struct.hpp"
// Member 'diags'
#include "autoware_adapi_v1_msgs/msg/detail/diag_leaf_struct__struct.hpp"
// Member 'links'
#include "autoware_adapi_v1_msgs/msg/detail/diag_link_struct__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_adapi_v1_msgs__msg__DiagGraphStruct __attribute__((deprecated))
#else
# define DEPRECATED__autoware_adapi_v1_msgs__msg__DiagGraphStruct __declspec(deprecated)
#endif

namespace autoware_adapi_v1_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DiagGraphStruct_
{
  using Type = DiagGraphStruct_<ContainerAllocator>;

  explicit DiagGraphStruct_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = "";
    }
  }

  explicit DiagGraphStruct_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = "";
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _id_type id;
  using _nodes_type =
    std::vector<autoware_adapi_v1_msgs::msg::DiagNodeStruct_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_adapi_v1_msgs::msg::DiagNodeStruct_<ContainerAllocator>>>;
  _nodes_type nodes;
  using _diags_type =
    std::vector<autoware_adapi_v1_msgs::msg::DiagLeafStruct_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_adapi_v1_msgs::msg::DiagLeafStruct_<ContainerAllocator>>>;
  _diags_type diags;
  using _links_type =
    std::vector<autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator>>>;
  _links_type links;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__nodes(
    const std::vector<autoware_adapi_v1_msgs::msg::DiagNodeStruct_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_adapi_v1_msgs::msg::DiagNodeStruct_<ContainerAllocator>>> & _arg)
  {
    this->nodes = _arg;
    return *this;
  }
  Type & set__diags(
    const std::vector<autoware_adapi_v1_msgs::msg::DiagLeafStruct_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_adapi_v1_msgs::msg::DiagLeafStruct_<ContainerAllocator>>> & _arg)
  {
    this->diags = _arg;
    return *this;
  }
  Type & set__links(
    const std::vector<autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator>>> & _arg)
  {
    this->links = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_adapi_v1_msgs::msg::DiagGraphStruct_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_adapi_v1_msgs::msg::DiagGraphStruct_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagGraphStruct_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagGraphStruct_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::DiagGraphStruct_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::DiagGraphStruct_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::DiagGraphStruct_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::DiagGraphStruct_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::DiagGraphStruct_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::DiagGraphStruct_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__DiagGraphStruct
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagGraphStruct_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__DiagGraphStruct
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagGraphStruct_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DiagGraphStruct_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->nodes != other.nodes) {
      return false;
    }
    if (this->diags != other.diags) {
      return false;
    }
    if (this->links != other.links) {
      return false;
    }
    return true;
  }
  bool operator!=(const DiagGraphStruct_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DiagGraphStruct_

// alias to use template instance with default allocator
using DiagGraphStruct =
  autoware_adapi_v1_msgs::msg::DiagGraphStruct_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_GRAPH_STRUCT__STRUCT_HPP_
