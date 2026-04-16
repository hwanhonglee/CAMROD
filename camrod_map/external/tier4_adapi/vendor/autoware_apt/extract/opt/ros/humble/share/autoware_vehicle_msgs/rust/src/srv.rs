#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to autoware_vehicle_msgs__srv__ControlModeCommand_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlModeCommand_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: u8,

}

impl ControlModeCommand_Request {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NO_COMMAND: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AUTONOMOUS: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AUTONOMOUS_STEER_ONLY: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AUTONOMOUS_VELOCITY_ONLY: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MANUAL: u8 = 4;

}


impl Default for ControlModeCommand_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ControlModeCommand_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ControlModeCommand_Request {
  type RmwMsg = super::srv::rmw::ControlModeCommand_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        mode: msg.mode,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      mode: msg.mode,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      mode: msg.mode,
    }
  }
}


// Corresponds to autoware_vehicle_msgs__srv__ControlModeCommand_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlModeCommand_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for ControlModeCommand_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ControlModeCommand_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ControlModeCommand_Response {
  type RmwMsg = super::srv::rmw::ControlModeCommand_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
  }
}






#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_vehicle_msgs__srv__ControlModeCommand() -> *const std::ffi::c_void;
}

// Corresponds to autoware_vehicle_msgs__srv__ControlModeCommand
#[allow(missing_docs, non_camel_case_types)]
pub struct ControlModeCommand;

impl rosidl_runtime_rs::Service for ControlModeCommand {
    type Request = ControlModeCommand_Request;
    type Response = ControlModeCommand_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_vehicle_msgs__srv__ControlModeCommand() }
    }
}


