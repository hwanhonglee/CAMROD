#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__InitializeLocalization_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__InitializeLocalization_Request__init(msg: *mut InitializeLocalization_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__InitializeLocalization_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<InitializeLocalization_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__InitializeLocalization_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<InitializeLocalization_Request>);
    fn autoware_adapi_v1_msgs__srv__InitializeLocalization_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<InitializeLocalization_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<InitializeLocalization_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__InitializeLocalization_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct InitializeLocalization_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: rosidl_runtime_rs::BoundedSequence<geometry_msgs::msg::rmw::PoseWithCovarianceStamped, 1>,

}



impl Default for InitializeLocalization_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__InitializeLocalization_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__InitializeLocalization_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for InitializeLocalization_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__InitializeLocalization_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__InitializeLocalization_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__InitializeLocalization_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for InitializeLocalization_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for InitializeLocalization_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/InitializeLocalization_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__InitializeLocalization_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__InitializeLocalization_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__InitializeLocalization_Response__init(msg: *mut InitializeLocalization_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__InitializeLocalization_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<InitializeLocalization_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__InitializeLocalization_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<InitializeLocalization_Response>);
    fn autoware_adapi_v1_msgs__srv__InitializeLocalization_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<InitializeLocalization_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<InitializeLocalization_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__InitializeLocalization_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct InitializeLocalization_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}

impl InitializeLocalization_Response {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_UNSAFE: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_GNSS_SUPPORT: u16 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_GNSS: u16 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_ESTIMATION: u16 = 4;

}


impl Default for InitializeLocalization_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__InitializeLocalization_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__InitializeLocalization_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for InitializeLocalization_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__InitializeLocalization_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__InitializeLocalization_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__InitializeLocalization_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for InitializeLocalization_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for InitializeLocalization_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/InitializeLocalization_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__InitializeLocalization_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ListManualControlMode_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__ListManualControlMode_Request__init(msg: *mut ListManualControlMode_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__ListManualControlMode_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ListManualControlMode_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__ListManualControlMode_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ListManualControlMode_Request>);
    fn autoware_adapi_v1_msgs__srv__ListManualControlMode_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ListManualControlMode_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ListManualControlMode_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ListManualControlMode_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ListManualControlMode_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ListManualControlMode_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__ListManualControlMode_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__ListManualControlMode_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ListManualControlMode_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ListManualControlMode_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ListManualControlMode_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ListManualControlMode_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ListManualControlMode_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ListManualControlMode_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/ListManualControlMode_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ListManualControlMode_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ListManualControlMode_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__ListManualControlMode_Response__init(msg: *mut ListManualControlMode_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__ListManualControlMode_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ListManualControlMode_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__ListManualControlMode_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ListManualControlMode_Response>);
    fn autoware_adapi_v1_msgs__srv__ListManualControlMode_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ListManualControlMode_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ListManualControlMode_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ListManualControlMode_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ListManualControlMode_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,


    // This member is not documented.
    #[allow(missing_docs)]
    pub modes: rosidl_runtime_rs::Sequence<super::super::msg::rmw::ManualControlMode>,

}



impl Default for ListManualControlMode_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__ListManualControlMode_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__ListManualControlMode_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ListManualControlMode_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ListManualControlMode_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ListManualControlMode_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ListManualControlMode_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ListManualControlMode_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ListManualControlMode_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/ListManualControlMode_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ListManualControlMode_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request__init(msg: *mut SelectManualControlMode_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SelectManualControlMode_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SelectManualControlMode_Request>);
    fn autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SelectManualControlMode_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SelectManualControlMode_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SelectManualControlMode_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: super::super::msg::rmw::ManualControlMode,

}



impl Default for SelectManualControlMode_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SelectManualControlMode_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SelectManualControlMode_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SelectManualControlMode_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SelectManualControlMode_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response__init(msg: *mut SelectManualControlMode_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SelectManualControlMode_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SelectManualControlMode_Response>);
    fn autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SelectManualControlMode_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SelectManualControlMode_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SelectManualControlMode_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}



impl Default for SelectManualControlMode_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SelectManualControlMode_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SelectManualControlMode_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SelectManualControlMode_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SelectManualControlMode_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ClearRoute_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__ClearRoute_Request__init(msg: *mut ClearRoute_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__ClearRoute_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearRoute_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__ClearRoute_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearRoute_Request>);
    fn autoware_adapi_v1_msgs__srv__ClearRoute_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearRoute_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearRoute_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ClearRoute_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ClearRoute_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__ClearRoute_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__ClearRoute_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearRoute_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ClearRoute_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ClearRoute_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ClearRoute_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearRoute_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearRoute_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/ClearRoute_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ClearRoute_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ClearRoute_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__ClearRoute_Response__init(msg: *mut ClearRoute_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__ClearRoute_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearRoute_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__ClearRoute_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearRoute_Response>);
    fn autoware_adapi_v1_msgs__srv__ClearRoute_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearRoute_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearRoute_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ClearRoute_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}



impl Default for ClearRoute_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__ClearRoute_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__ClearRoute_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearRoute_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ClearRoute_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ClearRoute_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ClearRoute_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearRoute_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearRoute_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/ClearRoute_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ClearRoute_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetRoute_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SetRoute_Request__init(msg: *mut SetRoute_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetRoute_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetRoute_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetRoute_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetRoute_Request>);
    fn autoware_adapi_v1_msgs__srv__SetRoute_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetRoute_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetRoute_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetRoute_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub option: super::super::msg::rmw::RouteOption,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub segments: rosidl_runtime_rs::Sequence<super::super::msg::rmw::RouteSegment>,

}



impl Default for SetRoute_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SetRoute_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SetRoute_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetRoute_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetRoute_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetRoute_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetRoute_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetRoute_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetRoute_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SetRoute_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetRoute_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetRoute_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SetRoute_Response__init(msg: *mut SetRoute_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetRoute_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetRoute_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetRoute_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetRoute_Response>);
    fn autoware_adapi_v1_msgs__srv__SetRoute_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetRoute_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetRoute_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetRoute_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}

impl SetRoute_Response {
    /// Deprecated. Use ERROR_INVALID_STATE.
    pub const ERROR_ROUTE_EXISTS: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_INVALID_STATE: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_PLANNER_UNREADY: u16 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_PLANNER_FAILED: u16 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_REROUTE_FAILED: u16 = 4;

}


impl Default for SetRoute_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SetRoute_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SetRoute_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetRoute_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetRoute_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetRoute_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetRoute_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetRoute_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetRoute_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SetRoute_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetRoute_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetRoutePoints_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SetRoutePoints_Request__init(msg: *mut SetRoutePoints_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetRoutePoints_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetRoutePoints_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetRoutePoints_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetRoutePoints_Request>);
    fn autoware_adapi_v1_msgs__srv__SetRoutePoints_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetRoutePoints_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetRoutePoints_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetRoutePoints_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetRoutePoints_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub option: super::super::msg::rmw::RouteOption,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub waypoints: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Pose>,

}



impl Default for SetRoutePoints_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SetRoutePoints_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SetRoutePoints_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetRoutePoints_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetRoutePoints_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetRoutePoints_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetRoutePoints_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetRoutePoints_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetRoutePoints_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SetRoutePoints_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetRoutePoints_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetRoutePoints_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SetRoutePoints_Response__init(msg: *mut SetRoutePoints_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetRoutePoints_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetRoutePoints_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetRoutePoints_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetRoutePoints_Response>);
    fn autoware_adapi_v1_msgs__srv__SetRoutePoints_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetRoutePoints_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetRoutePoints_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetRoutePoints_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetRoutePoints_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}

impl SetRoutePoints_Response {
    /// Deprecated. Use ERROR_INVALID_STATE.
    pub const ERROR_ROUTE_EXISTS: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_INVALID_STATE: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_PLANNER_UNREADY: u16 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_PLANNER_FAILED: u16 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_REROUTE_FAILED: u16 = 4;

}


impl Default for SetRoutePoints_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SetRoutePoints_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SetRoutePoints_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetRoutePoints_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetRoutePoints_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetRoutePoints_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetRoutePoints_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetRoutePoints_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetRoutePoints_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SetRoutePoints_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetRoutePoints_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request__init(msg: *mut ChangeOperationMode_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ChangeOperationMode_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ChangeOperationMode_Request>);
    fn autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ChangeOperationMode_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ChangeOperationMode_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChangeOperationMode_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ChangeOperationMode_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ChangeOperationMode_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ChangeOperationMode_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ChangeOperationMode_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/ChangeOperationMode_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response__init(msg: *mut ChangeOperationMode_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ChangeOperationMode_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ChangeOperationMode_Response>);
    fn autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ChangeOperationMode_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ChangeOperationMode_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChangeOperationMode_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}

impl ChangeOperationMode_Response {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_NOT_AVAILABLE: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_IN_TRANSITION: u16 = 2;

}


impl Default for ChangeOperationMode_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ChangeOperationMode_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ChangeOperationMode_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ChangeOperationMode_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/ChangeOperationMode_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__AcceptStart_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__AcceptStart_Request__init(msg: *mut AcceptStart_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__AcceptStart_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AcceptStart_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__AcceptStart_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AcceptStart_Request>);
    fn autoware_adapi_v1_msgs__srv__AcceptStart_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AcceptStart_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<AcceptStart_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__AcceptStart_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AcceptStart_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for AcceptStart_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__AcceptStart_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__AcceptStart_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AcceptStart_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__AcceptStart_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__AcceptStart_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__AcceptStart_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AcceptStart_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AcceptStart_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/AcceptStart_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__AcceptStart_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__AcceptStart_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__AcceptStart_Response__init(msg: *mut AcceptStart_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__AcceptStart_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AcceptStart_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__AcceptStart_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AcceptStart_Response>);
    fn autoware_adapi_v1_msgs__srv__AcceptStart_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AcceptStart_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<AcceptStart_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__AcceptStart_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AcceptStart_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}

impl AcceptStart_Response {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_NOT_STARTING: u16 = 1;

}


impl Default for AcceptStart_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__AcceptStart_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__AcceptStart_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AcceptStart_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__AcceptStart_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__AcceptStart_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__AcceptStart_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AcceptStart_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AcceptStart_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/AcceptStart_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__AcceptStart_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request__init(msg: *mut SetCooperationCommands_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetCooperationCommands_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetCooperationCommands_Request>);
    fn autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetCooperationCommands_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetCooperationCommands_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCooperationCommands_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub commands: rosidl_runtime_rs::Sequence<super::super::msg::rmw::CooperationCommand>,

}



impl Default for SetCooperationCommands_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetCooperationCommands_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetCooperationCommands_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetCooperationCommands_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SetCooperationCommands_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response__init(msg: *mut SetCooperationCommands_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetCooperationCommands_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetCooperationCommands_Response>);
    fn autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetCooperationCommands_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetCooperationCommands_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCooperationCommands_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}



impl Default for SetCooperationCommands_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetCooperationCommands_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetCooperationCommands_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetCooperationCommands_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SetCooperationCommands_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request__init(msg: *mut SetCooperationPolicies_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetCooperationPolicies_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetCooperationPolicies_Request>);
    fn autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetCooperationPolicies_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetCooperationPolicies_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCooperationPolicies_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub policies: rosidl_runtime_rs::Sequence<super::super::msg::rmw::CooperationPolicy>,

}



impl Default for SetCooperationPolicies_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetCooperationPolicies_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetCooperationPolicies_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetCooperationPolicies_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SetCooperationPolicies_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response__init(msg: *mut SetCooperationPolicies_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetCooperationPolicies_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetCooperationPolicies_Response>);
    fn autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetCooperationPolicies_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetCooperationPolicies_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCooperationPolicies_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}



impl Default for SetCooperationPolicies_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetCooperationPolicies_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetCooperationPolicies_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetCooperationPolicies_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SetCooperationPolicies_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request__init(msg: *mut GetCooperationPolicies_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetCooperationPolicies_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetCooperationPolicies_Request>);
    fn autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetCooperationPolicies_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetCooperationPolicies_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCooperationPolicies_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetCooperationPolicies_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetCooperationPolicies_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetCooperationPolicies_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetCooperationPolicies_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/GetCooperationPolicies_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response__init(msg: *mut GetCooperationPolicies_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetCooperationPolicies_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetCooperationPolicies_Response>);
    fn autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetCooperationPolicies_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetCooperationPolicies_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCooperationPolicies_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,


    // This member is not documented.
    #[allow(missing_docs)]
    pub policies: rosidl_runtime_rs::Sequence<super::super::msg::rmw::CooperationPolicy>,

}



impl Default for GetCooperationPolicies_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetCooperationPolicies_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetCooperationPolicies_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetCooperationPolicies_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/GetCooperationPolicies_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SendMrmRequest_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SendMrmRequest_Request__init(msg: *mut SendMrmRequest_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__SendMrmRequest_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SendMrmRequest_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SendMrmRequest_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SendMrmRequest_Request>);
    fn autoware_adapi_v1_msgs__srv__SendMrmRequest_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SendMrmRequest_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SendMrmRequest_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SendMrmRequest_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SendMrmRequest_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub request: super::super::msg::rmw::MrmRequest,

}



impl Default for SendMrmRequest_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SendMrmRequest_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SendMrmRequest_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SendMrmRequest_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SendMrmRequest_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SendMrmRequest_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SendMrmRequest_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SendMrmRequest_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SendMrmRequest_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SendMrmRequest_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SendMrmRequest_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SendMrmRequest_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SendMrmRequest_Response__init(msg: *mut SendMrmRequest_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__SendMrmRequest_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SendMrmRequest_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SendMrmRequest_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SendMrmRequest_Response>);
    fn autoware_adapi_v1_msgs__srv__SendMrmRequest_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SendMrmRequest_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SendMrmRequest_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SendMrmRequest_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SendMrmRequest_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}



impl Default for SendMrmRequest_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SendMrmRequest_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SendMrmRequest_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SendMrmRequest_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SendMrmRequest_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SendMrmRequest_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SendMrmRequest_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SendMrmRequest_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SendMrmRequest_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SendMrmRequest_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SendMrmRequest_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ListMrmDescription_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__ListMrmDescription_Request__init(msg: *mut ListMrmDescription_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__ListMrmDescription_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ListMrmDescription_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__ListMrmDescription_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ListMrmDescription_Request>);
    fn autoware_adapi_v1_msgs__srv__ListMrmDescription_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ListMrmDescription_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ListMrmDescription_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ListMrmDescription_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ListMrmDescription_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ListMrmDescription_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__ListMrmDescription_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__ListMrmDescription_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ListMrmDescription_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ListMrmDescription_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ListMrmDescription_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ListMrmDescription_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ListMrmDescription_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ListMrmDescription_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/ListMrmDescription_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ListMrmDescription_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ListMrmDescription_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__ListMrmDescription_Response__init(msg: *mut ListMrmDescription_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__ListMrmDescription_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ListMrmDescription_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__ListMrmDescription_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ListMrmDescription_Response>);
    fn autoware_adapi_v1_msgs__srv__ListMrmDescription_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ListMrmDescription_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ListMrmDescription_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ListMrmDescription_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ListMrmDescription_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub descriptions: rosidl_runtime_rs::Sequence<super::super::msg::rmw::MrmDescription>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}



impl Default for ListMrmDescription_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__ListMrmDescription_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__ListMrmDescription_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ListMrmDescription_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ListMrmDescription_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ListMrmDescription_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ListMrmDescription_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ListMrmDescription_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ListMrmDescription_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/ListMrmDescription_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ListMrmDescription_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request__init(msg: *mut ResetDiagGraph_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ResetDiagGraph_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ResetDiagGraph_Request>);
    fn autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ResetDiagGraph_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ResetDiagGraph_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ResetDiagGraph_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ResetDiagGraph_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ResetDiagGraph_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ResetDiagGraph_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ResetDiagGraph_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/ResetDiagGraph_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response__init(msg: *mut ResetDiagGraph_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ResetDiagGraph_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ResetDiagGraph_Response>);
    fn autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ResetDiagGraph_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ResetDiagGraph_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ResetDiagGraph_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}



impl Default for ResetDiagGraph_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ResetDiagGraph_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ResetDiagGraph_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ResetDiagGraph_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/ResetDiagGraph_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetDoorCommand_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SetDoorCommand_Request__init(msg: *mut SetDoorCommand_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetDoorCommand_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetDoorCommand_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetDoorCommand_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetDoorCommand_Request>);
    fn autoware_adapi_v1_msgs__srv__SetDoorCommand_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetDoorCommand_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetDoorCommand_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetDoorCommand_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetDoorCommand_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub doors: rosidl_runtime_rs::Sequence<super::super::msg::rmw::DoorCommand>,

}



impl Default for SetDoorCommand_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SetDoorCommand_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SetDoorCommand_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetDoorCommand_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetDoorCommand_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetDoorCommand_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetDoorCommand_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetDoorCommand_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetDoorCommand_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SetDoorCommand_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetDoorCommand_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetDoorCommand_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__SetDoorCommand_Response__init(msg: *mut SetDoorCommand_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetDoorCommand_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetDoorCommand_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__SetDoorCommand_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetDoorCommand_Response>);
    fn autoware_adapi_v1_msgs__srv__SetDoorCommand_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetDoorCommand_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetDoorCommand_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetDoorCommand_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetDoorCommand_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,

}



impl Default for SetDoorCommand_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__SetDoorCommand_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__SetDoorCommand_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetDoorCommand_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetDoorCommand_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetDoorCommand_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__SetDoorCommand_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetDoorCommand_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetDoorCommand_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/SetDoorCommand_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__SetDoorCommand_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetDoorLayout_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__GetDoorLayout_Request__init(msg: *mut GetDoorLayout_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetDoorLayout_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetDoorLayout_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetDoorLayout_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetDoorLayout_Request>);
    fn autoware_adapi_v1_msgs__srv__GetDoorLayout_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetDoorLayout_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetDoorLayout_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__GetDoorLayout_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetDoorLayout_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetDoorLayout_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__GetDoorLayout_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__GetDoorLayout_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetDoorLayout_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetDoorLayout_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetDoorLayout_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetDoorLayout_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetDoorLayout_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetDoorLayout_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/GetDoorLayout_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetDoorLayout_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetDoorLayout_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__GetDoorLayout_Response__init(msg: *mut GetDoorLayout_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetDoorLayout_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetDoorLayout_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetDoorLayout_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetDoorLayout_Response>);
    fn autoware_adapi_v1_msgs__srv__GetDoorLayout_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetDoorLayout_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetDoorLayout_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__GetDoorLayout_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetDoorLayout_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,


    // This member is not documented.
    #[allow(missing_docs)]
    pub doors: rosidl_runtime_rs::Sequence<super::super::msg::rmw::DoorLayout>,

}



impl Default for GetDoorLayout_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__GetDoorLayout_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__GetDoorLayout_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetDoorLayout_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetDoorLayout_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetDoorLayout_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetDoorLayout_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetDoorLayout_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetDoorLayout_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/GetDoorLayout_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetDoorLayout_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request__init(msg: *mut GetVehicleDimensions_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetVehicleDimensions_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetVehicleDimensions_Request>);
    fn autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetVehicleDimensions_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetVehicleDimensions_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetVehicleDimensions_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetVehicleDimensions_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetVehicleDimensions_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetVehicleDimensions_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetVehicleDimensions_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/GetVehicleDimensions_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response__init(msg: *mut GetVehicleDimensions_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetVehicleDimensions_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetVehicleDimensions_Response>);
    fn autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetVehicleDimensions_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetVehicleDimensions_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetVehicleDimensions_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dimensions: super::super::msg::rmw::VehicleDimensions,

}



impl Default for GetVehicleDimensions_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetVehicleDimensions_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetVehicleDimensions_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetVehicleDimensions_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/GetVehicleDimensions_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request__init(msg: *mut GetVehicleSpecs_Request) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetVehicleSpecs_Request>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetVehicleSpecs_Request>);
    fn autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetVehicleSpecs_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetVehicleSpecs_Request>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetVehicleSpecs_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetVehicleSpecs_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetVehicleSpecs_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetVehicleSpecs_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetVehicleSpecs_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/GetVehicleSpecs_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response__init(msg: *mut GetVehicleSpecs_Response) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetVehicleSpecs_Response>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetVehicleSpecs_Response>);
    fn autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetVehicleSpecs_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetVehicleSpecs_Response>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetVehicleSpecs_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::super::msg::rmw::ResponseStatus,


    // This member is not documented.
    #[allow(missing_docs)]
    pub specs: super::super::msg::rmw::VehicleSpecs,

}



impl Default for GetVehicleSpecs_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetVehicleSpecs_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetVehicleSpecs_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetVehicleSpecs_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/srv/GetVehicleSpecs_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response() }
  }
}






#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__InitializeLocalization() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__InitializeLocalization
#[allow(missing_docs, non_camel_case_types)]
pub struct InitializeLocalization;

impl rosidl_runtime_rs::Service for InitializeLocalization {
    type Request = InitializeLocalization_Request;
    type Response = InitializeLocalization_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__InitializeLocalization() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__ListManualControlMode() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ListManualControlMode
#[allow(missing_docs, non_camel_case_types)]
pub struct ListManualControlMode;

impl rosidl_runtime_rs::Service for ListManualControlMode {
    type Request = ListManualControlMode_Request;
    type Response = ListManualControlMode_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__ListManualControlMode() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SelectManualControlMode() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SelectManualControlMode
#[allow(missing_docs, non_camel_case_types)]
pub struct SelectManualControlMode;

impl rosidl_runtime_rs::Service for SelectManualControlMode {
    type Request = SelectManualControlMode_Request;
    type Response = SelectManualControlMode_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SelectManualControlMode() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__ClearRoute() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ClearRoute
#[allow(missing_docs, non_camel_case_types)]
pub struct ClearRoute;

impl rosidl_runtime_rs::Service for ClearRoute {
    type Request = ClearRoute_Request;
    type Response = ClearRoute_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__ClearRoute() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SetRoute() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetRoute
#[allow(missing_docs, non_camel_case_types)]
pub struct SetRoute;

impl rosidl_runtime_rs::Service for SetRoute {
    type Request = SetRoute_Request;
    type Response = SetRoute_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SetRoute() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SetRoutePoints() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetRoutePoints
#[allow(missing_docs, non_camel_case_types)]
pub struct SetRoutePoints;

impl rosidl_runtime_rs::Service for SetRoutePoints {
    type Request = SetRoutePoints_Request;
    type Response = SetRoutePoints_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SetRoutePoints() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__ChangeOperationMode() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ChangeOperationMode
#[allow(missing_docs, non_camel_case_types)]
pub struct ChangeOperationMode;

impl rosidl_runtime_rs::Service for ChangeOperationMode {
    type Request = ChangeOperationMode_Request;
    type Response = ChangeOperationMode_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__ChangeOperationMode() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__AcceptStart() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__AcceptStart
#[allow(missing_docs, non_camel_case_types)]
pub struct AcceptStart;

impl rosidl_runtime_rs::Service for AcceptStart {
    type Request = AcceptStart_Request;
    type Response = AcceptStart_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__AcceptStart() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SetCooperationCommands() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetCooperationCommands
#[allow(missing_docs, non_camel_case_types)]
pub struct SetCooperationCommands;

impl rosidl_runtime_rs::Service for SetCooperationCommands {
    type Request = SetCooperationCommands_Request;
    type Response = SetCooperationCommands_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SetCooperationCommands() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SetCooperationPolicies() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetCooperationPolicies
#[allow(missing_docs, non_camel_case_types)]
pub struct SetCooperationPolicies;

impl rosidl_runtime_rs::Service for SetCooperationPolicies {
    type Request = SetCooperationPolicies_Request;
    type Response = SetCooperationPolicies_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SetCooperationPolicies() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__GetCooperationPolicies() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__GetCooperationPolicies
#[allow(missing_docs, non_camel_case_types)]
pub struct GetCooperationPolicies;

impl rosidl_runtime_rs::Service for GetCooperationPolicies {
    type Request = GetCooperationPolicies_Request;
    type Response = GetCooperationPolicies_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__GetCooperationPolicies() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SendMrmRequest() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SendMrmRequest
#[allow(missing_docs, non_camel_case_types)]
pub struct SendMrmRequest;

impl rosidl_runtime_rs::Service for SendMrmRequest {
    type Request = SendMrmRequest_Request;
    type Response = SendMrmRequest_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SendMrmRequest() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__ListMrmDescription() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ListMrmDescription
#[allow(missing_docs, non_camel_case_types)]
pub struct ListMrmDescription;

impl rosidl_runtime_rs::Service for ListMrmDescription {
    type Request = ListMrmDescription_Request;
    type Response = ListMrmDescription_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__ListMrmDescription() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__ResetDiagGraph() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__ResetDiagGraph
#[allow(missing_docs, non_camel_case_types)]
pub struct ResetDiagGraph;

impl rosidl_runtime_rs::Service for ResetDiagGraph {
    type Request = ResetDiagGraph_Request;
    type Response = ResetDiagGraph_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__ResetDiagGraph() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SetDoorCommand() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__SetDoorCommand
#[allow(missing_docs, non_camel_case_types)]
pub struct SetDoorCommand;

impl rosidl_runtime_rs::Service for SetDoorCommand {
    type Request = SetDoorCommand_Request;
    type Response = SetDoorCommand_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__SetDoorCommand() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__GetDoorLayout() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__GetDoorLayout
#[allow(missing_docs, non_camel_case_types)]
pub struct GetDoorLayout;

impl rosidl_runtime_rs::Service for GetDoorLayout {
    type Request = GetDoorLayout_Request;
    type Response = GetDoorLayout_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__GetDoorLayout() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__GetVehicleDimensions() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__GetVehicleDimensions
#[allow(missing_docs, non_camel_case_types)]
pub struct GetVehicleDimensions;

impl rosidl_runtime_rs::Service for GetVehicleDimensions {
    type Request = GetVehicleDimensions_Request;
    type Response = GetVehicleDimensions_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__GetVehicleDimensions() }
    }
}




#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__GetVehicleSpecs() -> *const std::ffi::c_void;
}

// Corresponds to autoware_adapi_v1_msgs__srv__GetVehicleSpecs
#[allow(missing_docs, non_camel_case_types)]
pub struct GetVehicleSpecs;

impl rosidl_runtime_rs::Service for GetVehicleSpecs {
    type Request = GetVehicleSpecs_Request;
    type Response = GetVehicleSpecs_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_adapi_v1_msgs__srv__GetVehicleSpecs() }
    }
}


