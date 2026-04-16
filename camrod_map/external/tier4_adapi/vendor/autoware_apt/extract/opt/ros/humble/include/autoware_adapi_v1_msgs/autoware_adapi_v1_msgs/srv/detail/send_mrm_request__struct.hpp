// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_adapi_v1_msgs:srv/SendMrmRequest.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__SEND_MRM_REQUEST__STRUCT_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__SEND_MRM_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'request'
#include "autoware_adapi_v1_msgs/msg/detail/mrm_request__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_adapi_v1_msgs__srv__SendMrmRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__autoware_adapi_v1_msgs__srv__SendMrmRequest_Request __declspec(deprecated)
#endif

namespace autoware_adapi_v1_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SendMrmRequest_Request_
{
  using Type = SendMrmRequest_Request_<ContainerAllocator>;

  explicit SendMrmRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request(_init)
  {
    (void)_init;
  }

  explicit SendMrmRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _request_type =
    autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator>;
  _request_type request;

  // setters for named parameter idiom
  Type & set__request(
    const autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator> & _arg)
  {
    this->request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_adapi_v1_msgs__srv__SendMrmRequest_Request
    std::shared_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_adapi_v1_msgs__srv__SendMrmRequest_Request
    std::shared_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendMrmRequest_Request_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendMrmRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendMrmRequest_Request_

// alias to use template instance with default allocator
using SendMrmRequest_Request =
  autoware_adapi_v1_msgs::srv::SendMrmRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autoware_adapi_v1_msgs


// Include directives for member types
// Member 'status'
#include "autoware_adapi_v1_msgs/msg/detail/response_status__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_adapi_v1_msgs__srv__SendMrmRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__autoware_adapi_v1_msgs__srv__SendMrmRequest_Response __declspec(deprecated)
#endif

namespace autoware_adapi_v1_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SendMrmRequest_Response_
{
  using Type = SendMrmRequest_Response_<ContainerAllocator>;

  explicit SendMrmRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_init)
  {
    (void)_init;
  }

  explicit SendMrmRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _status_type =
    autoware_adapi_v1_msgs::msg::ResponseStatus_<ContainerAllocator>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const autoware_adapi_v1_msgs::msg::ResponseStatus_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_adapi_v1_msgs__srv__SendMrmRequest_Response
    std::shared_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_adapi_v1_msgs__srv__SendMrmRequest_Response
    std::shared_ptr<autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendMrmRequest_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendMrmRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendMrmRequest_Response_

// alias to use template instance with default allocator
using SendMrmRequest_Response =
  autoware_adapi_v1_msgs::srv::SendMrmRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autoware_adapi_v1_msgs

namespace autoware_adapi_v1_msgs
{

namespace srv
{

struct SendMrmRequest
{
  using Request = autoware_adapi_v1_msgs::srv::SendMrmRequest_Request;
  using Response = autoware_adapi_v1_msgs::srv::SendMrmRequest_Response;
};

}  // namespace srv

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__SEND_MRM_REQUEST__STRUCT_HPP_
