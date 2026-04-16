#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__ControlModeReport() -> *const std::ffi::c_void;
}

#[link(name = "autoware_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_vehicle_msgs__msg__ControlModeReport__init(msg: *mut ControlModeReport) -> bool;
    fn autoware_vehicle_msgs__msg__ControlModeReport__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ControlModeReport>, size: usize) -> bool;
    fn autoware_vehicle_msgs__msg__ControlModeReport__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ControlModeReport>);
    fn autoware_vehicle_msgs__msg__ControlModeReport__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ControlModeReport>, out_seq: *mut rosidl_runtime_rs::Sequence<ControlModeReport>) -> bool;
}

// Corresponds to autoware_vehicle_msgs__msg__ControlModeReport
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlModeReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: u8,

}

impl ControlModeReport {

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


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DISENGAGED: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NOT_READY: u8 = 6;

}


impl Default for ControlModeReport {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_vehicle_msgs__msg__ControlModeReport__init(&mut msg as *mut _) {
        panic!("Call to autoware_vehicle_msgs__msg__ControlModeReport__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ControlModeReport {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__ControlModeReport__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__ControlModeReport__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__ControlModeReport__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ControlModeReport {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ControlModeReport where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_vehicle_msgs/msg/ControlModeReport";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__ControlModeReport() }
  }
}


#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__Engage() -> *const std::ffi::c_void;
}

#[link(name = "autoware_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_vehicle_msgs__msg__Engage__init(msg: *mut Engage) -> bool;
    fn autoware_vehicle_msgs__msg__Engage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Engage>, size: usize) -> bool;
    fn autoware_vehicle_msgs__msg__Engage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Engage>);
    fn autoware_vehicle_msgs__msg__Engage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Engage>, out_seq: *mut rosidl_runtime_rs::Sequence<Engage>) -> bool;
}

// Corresponds to autoware_vehicle_msgs__msg__Engage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Engage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub engage: bool,

}



impl Default for Engage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_vehicle_msgs__msg__Engage__init(&mut msg as *mut _) {
        panic!("Call to autoware_vehicle_msgs__msg__Engage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Engage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__Engage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__Engage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__Engage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Engage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Engage where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_vehicle_msgs/msg/Engage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__Engage() }
  }
}


#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__GearCommand() -> *const std::ffi::c_void;
}

#[link(name = "autoware_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_vehicle_msgs__msg__GearCommand__init(msg: *mut GearCommand) -> bool;
    fn autoware_vehicle_msgs__msg__GearCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GearCommand>, size: usize) -> bool;
    fn autoware_vehicle_msgs__msg__GearCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GearCommand>);
    fn autoware_vehicle_msgs__msg__GearCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GearCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<GearCommand>) -> bool;
}

// Corresponds to autoware_vehicle_msgs__msg__GearCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GearCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub command: u8,

}

impl GearCommand {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NONE: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NEUTRAL: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_2: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_3: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_4: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_5: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_6: u8 = 7;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_7: u8 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_8: u8 = 9;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_9: u8 = 10;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_10: u8 = 11;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_11: u8 = 12;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_12: u8 = 13;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_13: u8 = 14;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_14: u8 = 15;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_15: u8 = 16;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_16: u8 = 17;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_17: u8 = 18;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_18: u8 = 19;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const REVERSE: u8 = 20;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const REVERSE_2: u8 = 21;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PARK: u8 = 22;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LOW: u8 = 23;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LOW_2: u8 = 24;

}


impl Default for GearCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_vehicle_msgs__msg__GearCommand__init(&mut msg as *mut _) {
        panic!("Call to autoware_vehicle_msgs__msg__GearCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GearCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__GearCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__GearCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__GearCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GearCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GearCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_vehicle_msgs/msg/GearCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__GearCommand() }
  }
}


#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__GearReport() -> *const std::ffi::c_void;
}

#[link(name = "autoware_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_vehicle_msgs__msg__GearReport__init(msg: *mut GearReport) -> bool;
    fn autoware_vehicle_msgs__msg__GearReport__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GearReport>, size: usize) -> bool;
    fn autoware_vehicle_msgs__msg__GearReport__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GearReport>);
    fn autoware_vehicle_msgs__msg__GearReport__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GearReport>, out_seq: *mut rosidl_runtime_rs::Sequence<GearReport>) -> bool;
}

// Corresponds to autoware_vehicle_msgs__msg__GearReport
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GearReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub report: u8,

}

impl GearReport {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NONE: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NEUTRAL: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_2: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_3: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_4: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_5: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_6: u8 = 7;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_7: u8 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_8: u8 = 9;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_9: u8 = 10;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_10: u8 = 11;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_11: u8 = 12;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_12: u8 = 13;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_13: u8 = 14;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_14: u8 = 15;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_15: u8 = 16;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_16: u8 = 17;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_17: u8 = 18;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE_18: u8 = 19;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const REVERSE: u8 = 20;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const REVERSE_2: u8 = 21;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PARK: u8 = 22;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LOW: u8 = 23;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LOW_2: u8 = 24;

}


impl Default for GearReport {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_vehicle_msgs__msg__GearReport__init(&mut msg as *mut _) {
        panic!("Call to autoware_vehicle_msgs__msg__GearReport__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GearReport {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__GearReport__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__GearReport__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__GearReport__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GearReport {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GearReport where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_vehicle_msgs/msg/GearReport";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__GearReport() }
  }
}


#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__HazardLightsCommand() -> *const std::ffi::c_void;
}

#[link(name = "autoware_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_vehicle_msgs__msg__HazardLightsCommand__init(msg: *mut HazardLightsCommand) -> bool;
    fn autoware_vehicle_msgs__msg__HazardLightsCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<HazardLightsCommand>, size: usize) -> bool;
    fn autoware_vehicle_msgs__msg__HazardLightsCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<HazardLightsCommand>);
    fn autoware_vehicle_msgs__msg__HazardLightsCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<HazardLightsCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<HazardLightsCommand>) -> bool;
}

// Corresponds to autoware_vehicle_msgs__msg__HazardLightsCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HazardLightsCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub command: u8,

}

impl HazardLightsCommand {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NO_COMMAND: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DISABLE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ENABLE: u8 = 2;

}


impl Default for HazardLightsCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_vehicle_msgs__msg__HazardLightsCommand__init(&mut msg as *mut _) {
        panic!("Call to autoware_vehicle_msgs__msg__HazardLightsCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for HazardLightsCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__HazardLightsCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__HazardLightsCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__HazardLightsCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for HazardLightsCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for HazardLightsCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_vehicle_msgs/msg/HazardLightsCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__HazardLightsCommand() }
  }
}


#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__HazardLightsReport() -> *const std::ffi::c_void;
}

#[link(name = "autoware_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_vehicle_msgs__msg__HazardLightsReport__init(msg: *mut HazardLightsReport) -> bool;
    fn autoware_vehicle_msgs__msg__HazardLightsReport__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<HazardLightsReport>, size: usize) -> bool;
    fn autoware_vehicle_msgs__msg__HazardLightsReport__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<HazardLightsReport>);
    fn autoware_vehicle_msgs__msg__HazardLightsReport__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<HazardLightsReport>, out_seq: *mut rosidl_runtime_rs::Sequence<HazardLightsReport>) -> bool;
}

// Corresponds to autoware_vehicle_msgs__msg__HazardLightsReport
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HazardLightsReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub report: u8,

}

impl HazardLightsReport {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DISABLE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ENABLE: u8 = 2;

}


impl Default for HazardLightsReport {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_vehicle_msgs__msg__HazardLightsReport__init(&mut msg as *mut _) {
        panic!("Call to autoware_vehicle_msgs__msg__HazardLightsReport__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for HazardLightsReport {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__HazardLightsReport__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__HazardLightsReport__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__HazardLightsReport__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for HazardLightsReport {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for HazardLightsReport where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_vehicle_msgs/msg/HazardLightsReport";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__HazardLightsReport() }
  }
}


#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__SteeringReport() -> *const std::ffi::c_void;
}

#[link(name = "autoware_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_vehicle_msgs__msg__SteeringReport__init(msg: *mut SteeringReport) -> bool;
    fn autoware_vehicle_msgs__msg__SteeringReport__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SteeringReport>, size: usize) -> bool;
    fn autoware_vehicle_msgs__msg__SteeringReport__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SteeringReport>);
    fn autoware_vehicle_msgs__msg__SteeringReport__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SteeringReport>, out_seq: *mut rosidl_runtime_rs::Sequence<SteeringReport>) -> bool;
}

// Corresponds to autoware_vehicle_msgs__msg__SteeringReport
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SteeringReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub steering_tire_angle: f32,

}



impl Default for SteeringReport {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_vehicle_msgs__msg__SteeringReport__init(&mut msg as *mut _) {
        panic!("Call to autoware_vehicle_msgs__msg__SteeringReport__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SteeringReport {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__SteeringReport__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__SteeringReport__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__SteeringReport__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SteeringReport {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SteeringReport where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_vehicle_msgs/msg/SteeringReport";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__SteeringReport() }
  }
}


#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__TurnIndicatorsCommand() -> *const std::ffi::c_void;
}

#[link(name = "autoware_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_vehicle_msgs__msg__TurnIndicatorsCommand__init(msg: *mut TurnIndicatorsCommand) -> bool;
    fn autoware_vehicle_msgs__msg__TurnIndicatorsCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TurnIndicatorsCommand>, size: usize) -> bool;
    fn autoware_vehicle_msgs__msg__TurnIndicatorsCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TurnIndicatorsCommand>);
    fn autoware_vehicle_msgs__msg__TurnIndicatorsCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TurnIndicatorsCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<TurnIndicatorsCommand>) -> bool;
}

// Corresponds to autoware_vehicle_msgs__msg__TurnIndicatorsCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TurnIndicatorsCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub command: u8,

}

impl TurnIndicatorsCommand {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NO_COMMAND: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DISABLE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ENABLE_LEFT: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ENABLE_RIGHT: u8 = 3;

}


impl Default for TurnIndicatorsCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_vehicle_msgs__msg__TurnIndicatorsCommand__init(&mut msg as *mut _) {
        panic!("Call to autoware_vehicle_msgs__msg__TurnIndicatorsCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TurnIndicatorsCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__TurnIndicatorsCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__TurnIndicatorsCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__TurnIndicatorsCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TurnIndicatorsCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TurnIndicatorsCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_vehicle_msgs/msg/TurnIndicatorsCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__TurnIndicatorsCommand() }
  }
}


#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__TurnIndicatorsReport() -> *const std::ffi::c_void;
}

#[link(name = "autoware_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_vehicle_msgs__msg__TurnIndicatorsReport__init(msg: *mut TurnIndicatorsReport) -> bool;
    fn autoware_vehicle_msgs__msg__TurnIndicatorsReport__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TurnIndicatorsReport>, size: usize) -> bool;
    fn autoware_vehicle_msgs__msg__TurnIndicatorsReport__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TurnIndicatorsReport>);
    fn autoware_vehicle_msgs__msg__TurnIndicatorsReport__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TurnIndicatorsReport>, out_seq: *mut rosidl_runtime_rs::Sequence<TurnIndicatorsReport>) -> bool;
}

// Corresponds to autoware_vehicle_msgs__msg__TurnIndicatorsReport
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TurnIndicatorsReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub report: u8,

}

impl TurnIndicatorsReport {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DISABLE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ENABLE_LEFT: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ENABLE_RIGHT: u8 = 3;

}


impl Default for TurnIndicatorsReport {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_vehicle_msgs__msg__TurnIndicatorsReport__init(&mut msg as *mut _) {
        panic!("Call to autoware_vehicle_msgs__msg__TurnIndicatorsReport__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TurnIndicatorsReport {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__TurnIndicatorsReport__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__TurnIndicatorsReport__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__TurnIndicatorsReport__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TurnIndicatorsReport {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TurnIndicatorsReport where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_vehicle_msgs/msg/TurnIndicatorsReport";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__TurnIndicatorsReport() }
  }
}


#[link(name = "autoware_vehicle_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__VelocityReport() -> *const std::ffi::c_void;
}

#[link(name = "autoware_vehicle_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_vehicle_msgs__msg__VelocityReport__init(msg: *mut VelocityReport) -> bool;
    fn autoware_vehicle_msgs__msg__VelocityReport__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VelocityReport>, size: usize) -> bool;
    fn autoware_vehicle_msgs__msg__VelocityReport__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VelocityReport>);
    fn autoware_vehicle_msgs__msg__VelocityReport__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VelocityReport>, out_seq: *mut rosidl_runtime_rs::Sequence<VelocityReport>) -> bool;
}

// Corresponds to autoware_vehicle_msgs__msg__VelocityReport
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelocityReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub longitudinal_velocity: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lateral_velocity: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub heading_rate: f32,

}



impl Default for VelocityReport {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_vehicle_msgs__msg__VelocityReport__init(&mut msg as *mut _) {
        panic!("Call to autoware_vehicle_msgs__msg__VelocityReport__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VelocityReport {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__VelocityReport__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__VelocityReport__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_vehicle_msgs__msg__VelocityReport__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VelocityReport {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VelocityReport where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_vehicle_msgs/msg/VelocityReport";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_vehicle_msgs__msg__VelocityReport() }
  }
}


