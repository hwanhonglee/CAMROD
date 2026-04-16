#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__srv__ControlModeCommand_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_vehicle_msgs__srv__ControlModeCommand_Request__init(msg: *mut ControlModeCommand_Request) -> bool;
    fn autoware_vehicle_msgs__srv__ControlModeCommand_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ControlModeCommand_Request>, size: usize) -> bool;
    fn autoware_vehicle_msgs__srv__ControlModeCommand_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ControlModeCommand_Request>);
    fn autoware_vehicle_msgs__srv__ControlModeCommand_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ControlModeCommand_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ControlModeCommand_Request>) -> bool;
}

// Corresponds to autoware_vehicle_msgs__srv__ControlModeCommand_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlModeCommand_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_vehicle_msgs__srv__ControlModeCommand_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_vehicle_msgs__srv__ControlModeCommand_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ControlModeCommand_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__srv__ControlModeCommand_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__srv__ControlModeCommand_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__srv__ControlModeCommand_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ControlModeCommand_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ControlModeCommand_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_vehicle_msgs/srv/ControlModeCommand_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__srv__ControlModeCommand_Request() }
  }
}


#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__srv__ControlModeCommand_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_vehicle_msgs__srv__ControlModeCommand_Response__init(msg: *mut ControlModeCommand_Response) -> bool;
    fn autoware_vehicle_msgs__srv__ControlModeCommand_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ControlModeCommand_Response>, size: usize) -> bool;
    fn autoware_vehicle_msgs__srv__ControlModeCommand_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ControlModeCommand_Response>);
    fn autoware_vehicle_msgs__srv__ControlModeCommand_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ControlModeCommand_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ControlModeCommand_Response>) -> bool;
}

// Corresponds to autoware_vehicle_msgs__srv__ControlModeCommand_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlModeCommand_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for ControlModeCommand_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_vehicle_msgs__srv__ControlModeCommand_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_vehicle_msgs__srv__ControlModeCommand_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ControlModeCommand_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__srv__ControlModeCommand_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__srv__ControlModeCommand_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__srv__ControlModeCommand_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ControlModeCommand_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ControlModeCommand_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_vehicle_msgs/srv/ControlModeCommand_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__srv__ControlModeCommand_Response() }
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


