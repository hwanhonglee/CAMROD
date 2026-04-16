#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__ClearRoute_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__srv__ClearRoute_Request__init(msg: *mut ClearRoute_Request) -> bool;
    fn autoware_planning_msgs__srv__ClearRoute_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearRoute_Request>, size: usize) -> bool;
    fn autoware_planning_msgs__srv__ClearRoute_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearRoute_Request>);
    fn autoware_planning_msgs__srv__ClearRoute_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearRoute_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearRoute_Request>) -> bool;
}

// Corresponds to autoware_planning_msgs__srv__ClearRoute_Request
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
      if !autoware_planning_msgs__srv__ClearRoute_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__srv__ClearRoute_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearRoute_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__ClearRoute_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__ClearRoute_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__ClearRoute_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearRoute_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearRoute_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/srv/ClearRoute_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__ClearRoute_Request() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__ClearRoute_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__srv__ClearRoute_Response__init(msg: *mut ClearRoute_Response) -> bool;
    fn autoware_planning_msgs__srv__ClearRoute_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearRoute_Response>, size: usize) -> bool;
    fn autoware_planning_msgs__srv__ClearRoute_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearRoute_Response>);
    fn autoware_planning_msgs__srv__ClearRoute_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearRoute_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearRoute_Response>) -> bool;
}

// Corresponds to autoware_planning_msgs__srv__ClearRoute_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: autoware_common_msgs::msg::rmw::ResponseStatus,

}



impl Default for ClearRoute_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__srv__ClearRoute_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__srv__ClearRoute_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearRoute_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__ClearRoute_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__ClearRoute_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__ClearRoute_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearRoute_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearRoute_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/srv/ClearRoute_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__ClearRoute_Response() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__SetLaneletRoute_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__srv__SetLaneletRoute_Request__init(msg: *mut SetLaneletRoute_Request) -> bool;
    fn autoware_planning_msgs__srv__SetLaneletRoute_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetLaneletRoute_Request>, size: usize) -> bool;
    fn autoware_planning_msgs__srv__SetLaneletRoute_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetLaneletRoute_Request>);
    fn autoware_planning_msgs__srv__SetLaneletRoute_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetLaneletRoute_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetLaneletRoute_Request>) -> bool;
}

// Corresponds to autoware_planning_msgs__srv__SetLaneletRoute_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetLaneletRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_pose: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub segments: rosidl_runtime_rs::Sequence<super::super::msg::rmw::LaneletSegment>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub uuid: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub allow_modification: bool,

}



impl Default for SetLaneletRoute_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__srv__SetLaneletRoute_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__srv__SetLaneletRoute_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetLaneletRoute_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetLaneletRoute_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetLaneletRoute_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetLaneletRoute_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetLaneletRoute_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetLaneletRoute_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/srv/SetLaneletRoute_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__SetLaneletRoute_Request() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__SetLaneletRoute_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__srv__SetLaneletRoute_Response__init(msg: *mut SetLaneletRoute_Response) -> bool;
    fn autoware_planning_msgs__srv__SetLaneletRoute_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetLaneletRoute_Response>, size: usize) -> bool;
    fn autoware_planning_msgs__srv__SetLaneletRoute_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetLaneletRoute_Response>);
    fn autoware_planning_msgs__srv__SetLaneletRoute_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetLaneletRoute_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetLaneletRoute_Response>) -> bool;
}

// Corresponds to autoware_planning_msgs__srv__SetLaneletRoute_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetLaneletRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: autoware_common_msgs::msg::rmw::ResponseStatus,

}



impl Default for SetLaneletRoute_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__srv__SetLaneletRoute_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__srv__SetLaneletRoute_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetLaneletRoute_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetLaneletRoute_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetLaneletRoute_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetLaneletRoute_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetLaneletRoute_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetLaneletRoute_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/srv/SetLaneletRoute_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__SetLaneletRoute_Response() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__SetPreferredPrimitive_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__srv__SetPreferredPrimitive_Request__init(msg: *mut SetPreferredPrimitive_Request) -> bool;
    fn autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetPreferredPrimitive_Request>, size: usize) -> bool;
    fn autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetPreferredPrimitive_Request>);
    fn autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetPreferredPrimitive_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetPreferredPrimitive_Request>) -> bool;
}

// Corresponds to autoware_planning_msgs__srv__SetPreferredPrimitive_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPreferredPrimitive_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub preferred_primitives: rosidl_runtime_rs::Sequence<super::super::msg::rmw::LaneletPrimitive>,

    /// reset flag for preferred primitives in route
    /// If set to true, this signals to mission_planner that the preferred-primitives have been reverted to those of the original path
    pub reset: bool,

    /// ID of the route that will be modified.
    pub uuid: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for SetPreferredPrimitive_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__srv__SetPreferredPrimitive_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__srv__SetPreferredPrimitive_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetPreferredPrimitive_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetPreferredPrimitive_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetPreferredPrimitive_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/srv/SetPreferredPrimitive_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__SetPreferredPrimitive_Request() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__SetPreferredPrimitive_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__srv__SetPreferredPrimitive_Response__init(msg: *mut SetPreferredPrimitive_Response) -> bool;
    fn autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetPreferredPrimitive_Response>, size: usize) -> bool;
    fn autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetPreferredPrimitive_Response>);
    fn autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetPreferredPrimitive_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetPreferredPrimitive_Response>) -> bool;
}

// Corresponds to autoware_planning_msgs__srv__SetPreferredPrimitive_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPreferredPrimitive_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: autoware_common_msgs::msg::rmw::ResponseStatus,

}



impl Default for SetPreferredPrimitive_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__srv__SetPreferredPrimitive_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__srv__SetPreferredPrimitive_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetPreferredPrimitive_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetPreferredPrimitive_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetPreferredPrimitive_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/srv/SetPreferredPrimitive_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__SetPreferredPrimitive_Response() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__SetWaypointRoute_Request() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__srv__SetWaypointRoute_Request__init(msg: *mut SetWaypointRoute_Request) -> bool;
    fn autoware_planning_msgs__srv__SetWaypointRoute_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetWaypointRoute_Request>, size: usize) -> bool;
    fn autoware_planning_msgs__srv__SetWaypointRoute_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetWaypointRoute_Request>);
    fn autoware_planning_msgs__srv__SetWaypointRoute_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetWaypointRoute_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetWaypointRoute_Request>) -> bool;
}

// Corresponds to autoware_planning_msgs__srv__SetWaypointRoute_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetWaypointRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_pose: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub waypoints: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Pose>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub uuid: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub allow_modification: bool,

}



impl Default for SetWaypointRoute_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__srv__SetWaypointRoute_Request__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__srv__SetWaypointRoute_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetWaypointRoute_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetWaypointRoute_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetWaypointRoute_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetWaypointRoute_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetWaypointRoute_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetWaypointRoute_Request where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/srv/SetWaypointRoute_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__SetWaypointRoute_Request() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__SetWaypointRoute_Response() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__srv__SetWaypointRoute_Response__init(msg: *mut SetWaypointRoute_Response) -> bool;
    fn autoware_planning_msgs__srv__SetWaypointRoute_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetWaypointRoute_Response>, size: usize) -> bool;
    fn autoware_planning_msgs__srv__SetWaypointRoute_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetWaypointRoute_Response>);
    fn autoware_planning_msgs__srv__SetWaypointRoute_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetWaypointRoute_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetWaypointRoute_Response>) -> bool;
}

// Corresponds to autoware_planning_msgs__srv__SetWaypointRoute_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetWaypointRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: autoware_common_msgs::msg::rmw::ResponseStatus,

}



impl Default for SetWaypointRoute_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__srv__SetWaypointRoute_Response__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__srv__SetWaypointRoute_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetWaypointRoute_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetWaypointRoute_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetWaypointRoute_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__srv__SetWaypointRoute_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetWaypointRoute_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetWaypointRoute_Response where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/srv/SetWaypointRoute_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__srv__SetWaypointRoute_Response() }
  }
}






#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_planning_msgs__srv__ClearRoute() -> *const std::ffi::c_void;
}

// Corresponds to autoware_planning_msgs__srv__ClearRoute
#[allow(missing_docs, non_camel_case_types)]
pub struct ClearRoute;

impl rosidl_runtime_rs::Service for ClearRoute {
    type Request = ClearRoute_Request;
    type Response = ClearRoute_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_planning_msgs__srv__ClearRoute() }
    }
}




#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_planning_msgs__srv__SetLaneletRoute() -> *const std::ffi::c_void;
}

// Corresponds to autoware_planning_msgs__srv__SetLaneletRoute
#[allow(missing_docs, non_camel_case_types)]
pub struct SetLaneletRoute;

impl rosidl_runtime_rs::Service for SetLaneletRoute {
    type Request = SetLaneletRoute_Request;
    type Response = SetLaneletRoute_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_planning_msgs__srv__SetLaneletRoute() }
    }
}




#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_planning_msgs__srv__SetPreferredPrimitive() -> *const std::ffi::c_void;
}

// Corresponds to autoware_planning_msgs__srv__SetPreferredPrimitive
#[allow(missing_docs, non_camel_case_types)]
pub struct SetPreferredPrimitive;

impl rosidl_runtime_rs::Service for SetPreferredPrimitive {
    type Request = SetPreferredPrimitive_Request;
    type Response = SetPreferredPrimitive_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_planning_msgs__srv__SetPreferredPrimitive() }
    }
}




#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__autoware_planning_msgs__srv__SetWaypointRoute() -> *const std::ffi::c_void;
}

// Corresponds to autoware_planning_msgs__srv__SetWaypointRoute
#[allow(missing_docs, non_camel_case_types)]
pub struct SetWaypointRoute;

impl rosidl_runtime_rs::Service for SetWaypointRoute {
    type Request = SetWaypointRoute_Request;
    type Response = SetWaypointRoute_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__autoware_planning_msgs__srv__SetWaypointRoute() }
    }
}


