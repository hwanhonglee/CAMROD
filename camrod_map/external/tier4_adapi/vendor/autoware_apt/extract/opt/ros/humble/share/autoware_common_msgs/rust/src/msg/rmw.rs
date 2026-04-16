#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "autoware_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_common_msgs__msg__ResponseStatus() -> *const std::ffi::c_void;
}

#[link(name = "autoware_common_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_common_msgs__msg__ResponseStatus__init(msg: *mut ResponseStatus) -> bool;
    fn autoware_common_msgs__msg__ResponseStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ResponseStatus>, size: usize) -> bool;
    fn autoware_common_msgs__msg__ResponseStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ResponseStatus>);
    fn autoware_common_msgs__msg__ResponseStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ResponseStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<ResponseStatus>) -> bool;
}

// Corresponds to autoware_common_msgs__msg__ResponseStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// error code

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ResponseStatus {
    /// variables
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub code: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}

impl ResponseStatus {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u16 = 50000;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SERVICE_UNREADY: u16 = 50001;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SERVICE_TIMEOUT: u16 = 50002;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TRANSFORM_ERROR: u16 = 50003;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PARAMETER_ERROR: u16 = 50004;

    /// warning code
    pub const DEPRECATED: u16 = 60000;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NO_EFFECT: u16 = 60001;

}


impl Default for ResponseStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_common_msgs__msg__ResponseStatus__init(&mut msg as *mut _) {
        panic!("Call to autoware_common_msgs__msg__ResponseStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ResponseStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_common_msgs__msg__ResponseStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_common_msgs__msg__ResponseStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_common_msgs__msg__ResponseStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ResponseStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ResponseStatus where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_common_msgs/msg/ResponseStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_common_msgs__msg__ResponseStatus() }
  }
}


