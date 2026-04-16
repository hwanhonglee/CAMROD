#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__LaneletPrimitive() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__msg__LaneletPrimitive__init(msg: *mut LaneletPrimitive) -> bool;
    fn autoware_planning_msgs__msg__LaneletPrimitive__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LaneletPrimitive>, size: usize) -> bool;
    fn autoware_planning_msgs__msg__LaneletPrimitive__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LaneletPrimitive>);
    fn autoware_planning_msgs__msg__LaneletPrimitive__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LaneletPrimitive>, out_seq: *mut rosidl_runtime_rs::Sequence<LaneletPrimitive>) -> bool;
}

// Corresponds to autoware_planning_msgs__msg__LaneletPrimitive
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaneletPrimitive {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: i64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub primitive_type: rosidl_runtime_rs::String,

}



impl Default for LaneletPrimitive {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__msg__LaneletPrimitive__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__msg__LaneletPrimitive__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LaneletPrimitive {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__LaneletPrimitive__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__LaneletPrimitive__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__LaneletPrimitive__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LaneletPrimitive {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LaneletPrimitive where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/msg/LaneletPrimitive";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__LaneletPrimitive() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__LaneletRoute() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__msg__LaneletRoute__init(msg: *mut LaneletRoute) -> bool;
    fn autoware_planning_msgs__msg__LaneletRoute__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LaneletRoute>, size: usize) -> bool;
    fn autoware_planning_msgs__msg__LaneletRoute__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LaneletRoute>);
    fn autoware_planning_msgs__msg__LaneletRoute__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LaneletRoute>, out_seq: *mut rosidl_runtime_rs::Sequence<LaneletRoute>) -> bool;
}

// Corresponds to autoware_planning_msgs__msg__LaneletRoute
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaneletRoute {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start_pose: geometry_msgs::msg::rmw::Pose,


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



impl Default for LaneletRoute {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__msg__LaneletRoute__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__msg__LaneletRoute__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LaneletRoute {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__LaneletRoute__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__LaneletRoute__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__LaneletRoute__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LaneletRoute {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LaneletRoute where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/msg/LaneletRoute";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__LaneletRoute() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__LaneletSegment() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__msg__LaneletSegment__init(msg: *mut LaneletSegment) -> bool;
    fn autoware_planning_msgs__msg__LaneletSegment__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LaneletSegment>, size: usize) -> bool;
    fn autoware_planning_msgs__msg__LaneletSegment__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LaneletSegment>);
    fn autoware_planning_msgs__msg__LaneletSegment__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LaneletSegment>, out_seq: *mut rosidl_runtime_rs::Sequence<LaneletSegment>) -> bool;
}

// Corresponds to autoware_planning_msgs__msg__LaneletSegment
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaneletSegment {

    // This member is not documented.
    #[allow(missing_docs)]
    pub preferred_primitive: super::super::msg::rmw::LaneletPrimitive,


    // This member is not documented.
    #[allow(missing_docs)]
    pub primitives: rosidl_runtime_rs::Sequence<super::super::msg::rmw::LaneletPrimitive>,

}



impl Default for LaneletSegment {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__msg__LaneletSegment__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__msg__LaneletSegment__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LaneletSegment {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__LaneletSegment__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__LaneletSegment__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__LaneletSegment__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LaneletSegment {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LaneletSegment where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/msg/LaneletSegment";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__LaneletSegment() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__PoseWithUuidStamped() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__msg__PoseWithUuidStamped__init(msg: *mut PoseWithUuidStamped) -> bool;
    fn autoware_planning_msgs__msg__PoseWithUuidStamped__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PoseWithUuidStamped>, size: usize) -> bool;
    fn autoware_planning_msgs__msg__PoseWithUuidStamped__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PoseWithUuidStamped>);
    fn autoware_planning_msgs__msg__PoseWithUuidStamped__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PoseWithUuidStamped>, out_seq: *mut rosidl_runtime_rs::Sequence<PoseWithUuidStamped>) -> bool;
}

// Corresponds to autoware_planning_msgs__msg__PoseWithUuidStamped
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PoseWithUuidStamped {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub uuid: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for PoseWithUuidStamped {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__msg__PoseWithUuidStamped__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__msg__PoseWithUuidStamped__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PoseWithUuidStamped {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__PoseWithUuidStamped__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__PoseWithUuidStamped__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__PoseWithUuidStamped__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PoseWithUuidStamped {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PoseWithUuidStamped where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/msg/PoseWithUuidStamped";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__PoseWithUuidStamped() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__Trajectory() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__msg__Trajectory__init(msg: *mut Trajectory) -> bool;
    fn autoware_planning_msgs__msg__Trajectory__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Trajectory>, size: usize) -> bool;
    fn autoware_planning_msgs__msg__Trajectory__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Trajectory>);
    fn autoware_planning_msgs__msg__Trajectory__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Trajectory>, out_seq: *mut rosidl_runtime_rs::Sequence<Trajectory>) -> bool;
}

// Corresponds to autoware_planning_msgs__msg__Trajectory
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Trajectory {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub points: rosidl_runtime_rs::Sequence<super::super::msg::rmw::TrajectoryPoint>,

}



impl Default for Trajectory {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__msg__Trajectory__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__msg__Trajectory__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Trajectory {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__Trajectory__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__Trajectory__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__Trajectory__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Trajectory {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Trajectory where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/msg/Trajectory";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__Trajectory() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__TrajectoryPoint() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__msg__TrajectoryPoint__init(msg: *mut TrajectoryPoint) -> bool;
    fn autoware_planning_msgs__msg__TrajectoryPoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TrajectoryPoint>, size: usize) -> bool;
    fn autoware_planning_msgs__msg__TrajectoryPoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TrajectoryPoint>);
    fn autoware_planning_msgs__msg__TrajectoryPoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TrajectoryPoint>, out_seq: *mut rosidl_runtime_rs::Sequence<TrajectoryPoint>) -> bool;
}

// Corresponds to autoware_planning_msgs__msg__TrajectoryPoint
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrajectoryPoint {

    // This member is not documented.
    #[allow(missing_docs)]
    pub time_from_start: builtin_interfaces::msg::rmw::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub longitudinal_velocity_mps: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lateral_velocity_mps: f32,

    /// acceleration_mps2 increases/decreases based on absolute vehicle motion and does not consider vehicle direction (forward/backward)
    pub acceleration_mps2: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub heading_rate_rps: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub front_wheel_angle_rad: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rear_wheel_angle_rad: f32,

}



impl Default for TrajectoryPoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__msg__TrajectoryPoint__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__msg__TrajectoryPoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TrajectoryPoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__TrajectoryPoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__TrajectoryPoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__TrajectoryPoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TrajectoryPoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TrajectoryPoint where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/msg/TrajectoryPoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__TrajectoryPoint() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__Path() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__msg__Path__init(msg: *mut Path) -> bool;
    fn autoware_planning_msgs__msg__Path__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Path>, size: usize) -> bool;
    fn autoware_planning_msgs__msg__Path__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Path>);
    fn autoware_planning_msgs__msg__Path__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Path>, out_seq: *mut rosidl_runtime_rs::Sequence<Path>) -> bool;
}

// Corresponds to autoware_planning_msgs__msg__Path
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Path {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub points: rosidl_runtime_rs::Sequence<super::super::msg::rmw::PathPoint>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub left_bound: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Point>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_bound: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Point>,

}



impl Default for Path {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__msg__Path__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__msg__Path__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Path {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__Path__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__Path__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__Path__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Path {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Path where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/msg/Path";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__Path() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__PathPoint() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__msg__PathPoint__init(msg: *mut PathPoint) -> bool;
    fn autoware_planning_msgs__msg__PathPoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PathPoint>, size: usize) -> bool;
    fn autoware_planning_msgs__msg__PathPoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PathPoint>);
    fn autoware_planning_msgs__msg__PathPoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PathPoint>, out_seq: *mut rosidl_runtime_rs::Sequence<PathPoint>) -> bool;
}

// Corresponds to autoware_planning_msgs__msg__PathPoint
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PathPoint {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub longitudinal_velocity_mps: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lateral_velocity_mps: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub heading_rate_rps: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_final: bool,

}



impl Default for PathPoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__msg__PathPoint__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__msg__PathPoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PathPoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__PathPoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__PathPoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__PathPoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PathPoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PathPoint where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/msg/PathPoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__PathPoint() }
  }
}


#[link(name = "autoware_planning_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__RouteState() -> *const std::ffi::c_void;
}

#[link(name = "autoware_planning_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_planning_msgs__msg__RouteState__init(msg: *mut RouteState) -> bool;
    fn autoware_planning_msgs__msg__RouteState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RouteState>, size: usize) -> bool;
    fn autoware_planning_msgs__msg__RouteState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RouteState>);
    fn autoware_planning_msgs__msg__RouteState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RouteState>, out_seq: *mut rosidl_runtime_rs::Sequence<RouteState>) -> bool;
}

// Corresponds to autoware_planning_msgs__msg__RouteState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: u8,

}

impl RouteState {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INITIALIZING: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNSET: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ROUTING: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SET: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const REROUTING: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ARRIVED: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ABORTED: u8 = 7;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INTERRUPTED: u8 = 8;

}


impl Default for RouteState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_planning_msgs__msg__RouteState__init(&mut msg as *mut _) {
        panic!("Call to autoware_planning_msgs__msg__RouteState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RouteState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__RouteState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__RouteState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_planning_msgs__msg__RouteState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RouteState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RouteState where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_planning_msgs/msg/RouteState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_planning_msgs__msg__RouteState() }
  }
}


