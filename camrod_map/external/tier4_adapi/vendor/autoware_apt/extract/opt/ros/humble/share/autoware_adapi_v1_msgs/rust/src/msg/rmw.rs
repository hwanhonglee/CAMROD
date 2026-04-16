#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__ResponseStatus() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__ResponseStatus__init(msg: *mut ResponseStatus) -> bool;
    fn autoware_adapi_v1_msgs__msg__ResponseStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ResponseStatus>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__ResponseStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ResponseStatus>);
    fn autoware_adapi_v1_msgs__msg__ResponseStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ResponseStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<ResponseStatus>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__ResponseStatus
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
      if !autoware_adapi_v1_msgs__msg__ResponseStatus__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__ResponseStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ResponseStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ResponseStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ResponseStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ResponseStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ResponseStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ResponseStatus where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/ResponseStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__ResponseStatus() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__KvString() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__KvString__init(msg: *mut KvString) -> bool;
    fn autoware_adapi_v1_msgs__msg__KvString__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<KvString>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__KvString__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<KvString>);
    fn autoware_adapi_v1_msgs__msg__KvString__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<KvString>, out_seq: *mut rosidl_runtime_rs::Sequence<KvString>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__KvString
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct KvString {

    // This member is not documented.
    #[allow(missing_docs)]
    pub key: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub value: rosidl_runtime_rs::String,

}



impl Default for KvString {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__KvString__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__KvString__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for KvString {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__KvString__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__KvString__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__KvString__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for KvString {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for KvString where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/KvString";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__KvString() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__LocalizationInitializationState() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__LocalizationInitializationState__init(msg: *mut LocalizationInitializationState) -> bool;
    fn autoware_adapi_v1_msgs__msg__LocalizationInitializationState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LocalizationInitializationState>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__LocalizationInitializationState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LocalizationInitializationState>);
    fn autoware_adapi_v1_msgs__msg__LocalizationInitializationState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LocalizationInitializationState>, out_seq: *mut rosidl_runtime_rs::Sequence<LocalizationInitializationState>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__LocalizationInitializationState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LocalizationInitializationState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: u16,

}

impl LocalizationInitializationState {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u16 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNINITIALIZED: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INITIALIZING: u16 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INITIALIZED: u16 = 3;

}


impl Default for LocalizationInitializationState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__LocalizationInitializationState__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__LocalizationInitializationState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LocalizationInitializationState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__LocalizationInitializationState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__LocalizationInitializationState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__LocalizationInitializationState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LocalizationInitializationState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LocalizationInitializationState where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/LocalizationInitializationState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__LocalizationInitializationState() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__PedalsCommand() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__PedalsCommand__init(msg: *mut PedalsCommand) -> bool;
    fn autoware_adapi_v1_msgs__msg__PedalsCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PedalsCommand>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__PedalsCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PedalsCommand>);
    fn autoware_adapi_v1_msgs__msg__PedalsCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PedalsCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<PedalsCommand>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__PedalsCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PedalsCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub throttle: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub brake: f32,

}



impl Default for PedalsCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__PedalsCommand__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__PedalsCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PedalsCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__PedalsCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__PedalsCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__PedalsCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PedalsCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PedalsCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/PedalsCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__PedalsCommand() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__AccelerationCommand() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__AccelerationCommand__init(msg: *mut AccelerationCommand) -> bool;
    fn autoware_adapi_v1_msgs__msg__AccelerationCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AccelerationCommand>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__AccelerationCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AccelerationCommand>);
    fn autoware_adapi_v1_msgs__msg__AccelerationCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AccelerationCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<AccelerationCommand>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__AccelerationCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AccelerationCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub acceleration: f32,

}



impl Default for AccelerationCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__AccelerationCommand__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__AccelerationCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AccelerationCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__AccelerationCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__AccelerationCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__AccelerationCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AccelerationCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AccelerationCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/AccelerationCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__AccelerationCommand() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VelocityCommand() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__VelocityCommand__init(msg: *mut VelocityCommand) -> bool;
    fn autoware_adapi_v1_msgs__msg__VelocityCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VelocityCommand>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__VelocityCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VelocityCommand>);
    fn autoware_adapi_v1_msgs__msg__VelocityCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VelocityCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<VelocityCommand>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__VelocityCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelocityCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: f32,

}



impl Default for VelocityCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__VelocityCommand__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__VelocityCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VelocityCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VelocityCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VelocityCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VelocityCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VelocityCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VelocityCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/VelocityCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VelocityCommand() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__SteeringCommand() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__SteeringCommand__init(msg: *mut SteeringCommand) -> bool;
    fn autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SteeringCommand>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SteeringCommand>);
    fn autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SteeringCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<SteeringCommand>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__SteeringCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SteeringCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub steering_tire_angle: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub steering_tire_velocity: f32,

}



impl Default for SteeringCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__SteeringCommand__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__SteeringCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SteeringCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__SteeringCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SteeringCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SteeringCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/SteeringCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__SteeringCommand() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__GearCommand() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__GearCommand__init(msg: *mut GearCommand) -> bool;
    fn autoware_adapi_v1_msgs__msg__GearCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GearCommand>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__GearCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GearCommand>);
    fn autoware_adapi_v1_msgs__msg__GearCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GearCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<GearCommand>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__GearCommand
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
    pub command: super::super::msg::rmw::Gear,

}



impl Default for GearCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__GearCommand__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__GearCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GearCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__GearCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__GearCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__GearCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GearCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GearCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/GearCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__GearCommand() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand__init(msg: *mut TurnIndicatorsCommand) -> bool;
    fn autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TurnIndicatorsCommand>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TurnIndicatorsCommand>);
    fn autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TurnIndicatorsCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<TurnIndicatorsCommand>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand
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
    pub command: super::super::msg::rmw::TurnIndicators,

}



impl Default for TurnIndicatorsCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TurnIndicatorsCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TurnIndicatorsCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TurnIndicatorsCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/TurnIndicatorsCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__HazardLightsCommand() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__HazardLightsCommand__init(msg: *mut HazardLightsCommand) -> bool;
    fn autoware_adapi_v1_msgs__msg__HazardLightsCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<HazardLightsCommand>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__HazardLightsCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<HazardLightsCommand>);
    fn autoware_adapi_v1_msgs__msg__HazardLightsCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<HazardLightsCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<HazardLightsCommand>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__HazardLightsCommand
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
    pub command: super::super::msg::rmw::HazardLights,

}



impl Default for HazardLightsCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__HazardLightsCommand__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__HazardLightsCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for HazardLightsCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__HazardLightsCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__HazardLightsCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__HazardLightsCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for HazardLightsCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for HazardLightsCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/HazardLightsCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__HazardLightsCommand() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__ManualControlMode() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__ManualControlMode__init(msg: *mut ManualControlMode) -> bool;
    fn autoware_adapi_v1_msgs__msg__ManualControlMode__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ManualControlMode>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__ManualControlMode__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ManualControlMode>);
    fn autoware_adapi_v1_msgs__msg__ManualControlMode__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ManualControlMode>, out_seq: *mut rosidl_runtime_rs::Sequence<ManualControlMode>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__ManualControlMode
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManualControlMode {

    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: u8,

}

impl ManualControlMode {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DISABLED: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PEDALS: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACCELERATION: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const VELOCITY: u8 = 4;

}


impl Default for ManualControlMode {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__ManualControlMode__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__ManualControlMode__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ManualControlMode {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ManualControlMode__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ManualControlMode__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ManualControlMode__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ManualControlMode {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ManualControlMode where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/ManualControlMode";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__ManualControlMode() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__ManualControlModeStatus() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__ManualControlModeStatus__init(msg: *mut ManualControlModeStatus) -> bool;
    fn autoware_adapi_v1_msgs__msg__ManualControlModeStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ManualControlModeStatus>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__ManualControlModeStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ManualControlModeStatus>);
    fn autoware_adapi_v1_msgs__msg__ManualControlModeStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ManualControlModeStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<ManualControlModeStatus>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__ManualControlModeStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManualControlModeStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: super::super::msg::rmw::ManualControlMode,

}



impl Default for ManualControlModeStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__ManualControlModeStatus__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__ManualControlModeStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ManualControlModeStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ManualControlModeStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ManualControlModeStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ManualControlModeStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ManualControlModeStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ManualControlModeStatus where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/ManualControlModeStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__ManualControlModeStatus() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat__init(msg: *mut ManualOperatorHeartbeat) -> bool;
    fn autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ManualOperatorHeartbeat>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ManualOperatorHeartbeat>);
    fn autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ManualOperatorHeartbeat>, out_seq: *mut rosidl_runtime_rs::Sequence<ManualOperatorHeartbeat>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManualOperatorHeartbeat {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ready: bool,

}



impl Default for ManualOperatorHeartbeat {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ManualOperatorHeartbeat {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ManualOperatorHeartbeat {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ManualOperatorHeartbeat where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/ManualOperatorHeartbeat";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__RouteState() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__RouteState__init(msg: *mut RouteState) -> bool;
    fn autoware_adapi_v1_msgs__msg__RouteState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RouteState>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__RouteState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RouteState>);
    fn autoware_adapi_v1_msgs__msg__RouteState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RouteState>, out_seq: *mut rosidl_runtime_rs::Sequence<RouteState>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__RouteState
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
    pub state: u16,

}

impl RouteState {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u16 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNSET: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SET: u16 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ARRIVED: u16 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CHANGING: u16 = 4;

}


impl Default for RouteState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__RouteState__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__RouteState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RouteState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RouteState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RouteState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RouteState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RouteState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RouteState where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/RouteState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__RouteState() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__Route() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__Route__init(msg: *mut Route) -> bool;
    fn autoware_adapi_v1_msgs__msg__Route__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Route>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__Route__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Route>);
    fn autoware_adapi_v1_msgs__msg__Route__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Route>, out_seq: *mut rosidl_runtime_rs::Sequence<Route>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__Route
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Route {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub data: rosidl_runtime_rs::BoundedSequence<super::super::msg::rmw::RouteData, 1>,

}



impl Default for Route {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__Route__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__Route__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Route {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__Route__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__Route__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__Route__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Route {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Route where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/Route";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__Route() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__RouteData() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__RouteData__init(msg: *mut RouteData) -> bool;
    fn autoware_adapi_v1_msgs__msg__RouteData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RouteData>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__RouteData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RouteData>);
    fn autoware_adapi_v1_msgs__msg__RouteData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RouteData>, out_seq: *mut rosidl_runtime_rs::Sequence<RouteData>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__RouteData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub segments: rosidl_runtime_rs::Sequence<super::super::msg::rmw::RouteSegment>,

}



impl Default for RouteData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__RouteData__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__RouteData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RouteData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RouteData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RouteData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RouteData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RouteData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RouteData where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/RouteData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__RouteData() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__RouteOption() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__RouteOption__init(msg: *mut RouteOption) -> bool;
    fn autoware_adapi_v1_msgs__msg__RouteOption__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RouteOption>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__RouteOption__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RouteOption>);
    fn autoware_adapi_v1_msgs__msg__RouteOption__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RouteOption>, out_seq: *mut rosidl_runtime_rs::Sequence<RouteOption>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__RouteOption
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Please refer to the following pages for details on each option.
/// https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/routing/

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteOption {

    // This member is not documented.
    #[allow(missing_docs)]
    pub allow_goal_modification: bool,

}



impl Default for RouteOption {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__RouteOption__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__RouteOption__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RouteOption {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RouteOption__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RouteOption__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RouteOption__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RouteOption {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RouteOption where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/RouteOption";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__RouteOption() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__RoutePrimitive() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__RoutePrimitive__init(msg: *mut RoutePrimitive) -> bool;
    fn autoware_adapi_v1_msgs__msg__RoutePrimitive__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RoutePrimitive>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__RoutePrimitive__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RoutePrimitive>);
    fn autoware_adapi_v1_msgs__msg__RoutePrimitive__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RoutePrimitive>, out_seq: *mut rosidl_runtime_rs::Sequence<RoutePrimitive>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__RoutePrimitive
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RoutePrimitive {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: i64,

    /// The same id may be used for each type.
    pub type_: rosidl_runtime_rs::String,

}



impl Default for RoutePrimitive {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__RoutePrimitive__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__RoutePrimitive__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RoutePrimitive {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RoutePrimitive__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RoutePrimitive__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RoutePrimitive__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RoutePrimitive {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RoutePrimitive where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/RoutePrimitive";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__RoutePrimitive() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__RouteSegment() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__RouteSegment__init(msg: *mut RouteSegment) -> bool;
    fn autoware_adapi_v1_msgs__msg__RouteSegment__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RouteSegment>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__RouteSegment__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RouteSegment>);
    fn autoware_adapi_v1_msgs__msg__RouteSegment__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RouteSegment>, out_seq: *mut rosidl_runtime_rs::Sequence<RouteSegment>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__RouteSegment
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteSegment {

    // This member is not documented.
    #[allow(missing_docs)]
    pub preferred: super::super::msg::rmw::RoutePrimitive,

    /// Does not include the preferred primitive.
    pub alternatives: rosidl_runtime_rs::Sequence<super::super::msg::rmw::RoutePrimitive>,

}



impl Default for RouteSegment {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__RouteSegment__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__RouteSegment__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RouteSegment {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RouteSegment__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RouteSegment__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RouteSegment__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RouteSegment {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RouteSegment where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/RouteSegment";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__RouteSegment() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__OperationModeState() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__OperationModeState__init(msg: *mut OperationModeState) -> bool;
    fn autoware_adapi_v1_msgs__msg__OperationModeState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<OperationModeState>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__OperationModeState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<OperationModeState>);
    fn autoware_adapi_v1_msgs__msg__OperationModeState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<OperationModeState>, out_seq: *mut rosidl_runtime_rs::Sequence<OperationModeState>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__OperationModeState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// constants for mode

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperationModeState {
    /// variables
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_autoware_control_enabled: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_in_transition: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_stop_mode_available: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_autonomous_mode_available: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_local_mode_available: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_remote_mode_available: bool,

}

impl OperationModeState {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STOP: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AUTONOMOUS: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LOCAL: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const REMOTE: u8 = 4;

}


impl Default for OperationModeState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__OperationModeState__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__OperationModeState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for OperationModeState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__OperationModeState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__OperationModeState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__OperationModeState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for OperationModeState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for OperationModeState where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/OperationModeState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__OperationModeState() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__MotionState() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__MotionState__init(msg: *mut MotionState) -> bool;
    fn autoware_adapi_v1_msgs__msg__MotionState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotionState>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__MotionState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotionState>);
    fn autoware_adapi_v1_msgs__msg__MotionState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotionState>, out_seq: *mut rosidl_runtime_rs::Sequence<MotionState>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__MotionState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotionState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: u16,

}

impl MotionState {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u16 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STOPPED: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STARTING: u16 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOVING: u16 = 3;

}


impl Default for MotionState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__MotionState__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__MotionState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotionState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MotionState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MotionState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MotionState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotionState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotionState where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/MotionState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__MotionState() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DynamicObject() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DynamicObject__init(msg: *mut DynamicObject) -> bool;
    fn autoware_adapi_v1_msgs__msg__DynamicObject__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DynamicObject>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DynamicObject__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DynamicObject>);
    fn autoware_adapi_v1_msgs__msg__DynamicObject__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DynamicObject>, out_seq: *mut rosidl_runtime_rs::Sequence<DynamicObject>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DynamicObject
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DynamicObject {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub existence_probability: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub classification: rosidl_runtime_rs::Sequence<super::super::msg::rmw::ObjectClassification>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kinematics: super::super::msg::rmw::DynamicObjectKinematics,


    // This member is not documented.
    #[allow(missing_docs)]
    pub shape: shape_msgs::msg::rmw::SolidPrimitive,

}



impl Default for DynamicObject {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DynamicObject__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DynamicObject__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DynamicObject {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DynamicObject__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DynamicObject__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DynamicObject__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DynamicObject {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DynamicObject where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DynamicObject";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DynamicObject() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DynamicObjectArray() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DynamicObjectArray__init(msg: *mut DynamicObjectArray) -> bool;
    fn autoware_adapi_v1_msgs__msg__DynamicObjectArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DynamicObjectArray>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DynamicObjectArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DynamicObjectArray>);
    fn autoware_adapi_v1_msgs__msg__DynamicObjectArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DynamicObjectArray>, out_seq: *mut rosidl_runtime_rs::Sequence<DynamicObjectArray>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DynamicObjectArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DynamicObjectArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub objects: rosidl_runtime_rs::Sequence<super::super::msg::rmw::DynamicObject>,

}



impl Default for DynamicObjectArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DynamicObjectArray__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DynamicObjectArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DynamicObjectArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DynamicObjectArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DynamicObjectArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DynamicObjectArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DynamicObjectArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DynamicObjectArray where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DynamicObjectArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DynamicObjectArray() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DynamicObjectKinematics() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DynamicObjectKinematics__init(msg: *mut DynamicObjectKinematics) -> bool;
    fn autoware_adapi_v1_msgs__msg__DynamicObjectKinematics__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DynamicObjectKinematics>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DynamicObjectKinematics__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DynamicObjectKinematics>);
    fn autoware_adapi_v1_msgs__msg__DynamicObjectKinematics__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DynamicObjectKinematics>, out_seq: *mut rosidl_runtime_rs::Sequence<DynamicObjectKinematics>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DynamicObjectKinematics
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DynamicObjectKinematics {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub twist: geometry_msgs::msg::rmw::Twist,


    // This member is not documented.
    #[allow(missing_docs)]
    pub accel: geometry_msgs::msg::rmw::Accel,


    // This member is not documented.
    #[allow(missing_docs)]
    pub predicted_paths: rosidl_runtime_rs::Sequence<super::super::msg::rmw::DynamicObjectPath>,

}



impl Default for DynamicObjectKinematics {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DynamicObjectKinematics__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DynamicObjectKinematics__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DynamicObjectKinematics {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DynamicObjectKinematics__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DynamicObjectKinematics__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DynamicObjectKinematics__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DynamicObjectKinematics {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DynamicObjectKinematics where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DynamicObjectKinematics";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DynamicObjectKinematics() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DynamicObjectPath() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DynamicObjectPath__init(msg: *mut DynamicObjectPath) -> bool;
    fn autoware_adapi_v1_msgs__msg__DynamicObjectPath__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DynamicObjectPath>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DynamicObjectPath__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DynamicObjectPath>);
    fn autoware_adapi_v1_msgs__msg__DynamicObjectPath__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DynamicObjectPath>, out_seq: *mut rosidl_runtime_rs::Sequence<DynamicObjectPath>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DynamicObjectPath
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DynamicObjectPath {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Pose>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time_step: builtin_interfaces::msg::rmw::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub confidence: f64,

}



impl Default for DynamicObjectPath {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DynamicObjectPath__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DynamicObjectPath__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DynamicObjectPath {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DynamicObjectPath__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DynamicObjectPath__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DynamicObjectPath__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DynamicObjectPath {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DynamicObjectPath where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DynamicObjectPath";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DynamicObjectPath() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__ObjectClassification() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__ObjectClassification__init(msg: *mut ObjectClassification) -> bool;
    fn autoware_adapi_v1_msgs__msg__ObjectClassification__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ObjectClassification>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__ObjectClassification__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ObjectClassification>);
    fn autoware_adapi_v1_msgs__msg__ObjectClassification__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ObjectClassification>, out_seq: *mut rosidl_runtime_rs::Sequence<ObjectClassification>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__ObjectClassification
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ObjectClassification {

    // This member is not documented.
    #[allow(missing_docs)]
    pub label: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub probability: f64,

}

impl ObjectClassification {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CAR: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TRUCK: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const BUS: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TRAILER: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOTORCYCLE: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const BICYCLE: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PEDESTRIAN: u8 = 7;

}


impl Default for ObjectClassification {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__ObjectClassification__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__ObjectClassification__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ObjectClassification {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ObjectClassification__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ObjectClassification__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__ObjectClassification__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ObjectClassification {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ObjectClassification where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/ObjectClassification";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__ObjectClassification() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__SteeringFactor() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__SteeringFactor__init(msg: *mut SteeringFactor) -> bool;
    fn autoware_adapi_v1_msgs__msg__SteeringFactor__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SteeringFactor>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__SteeringFactor__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SteeringFactor>);
    fn autoware_adapi_v1_msgs__msg__SteeringFactor__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SteeringFactor>, out_seq: *mut rosidl_runtime_rs::Sequence<SteeringFactor>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__SteeringFactor
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// constants for common use

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SteeringFactor {
    /// variables
    pub pose: [geometry_msgs::msg::rmw::Pose; 2],


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance: [f32; 2],


    // This member is not documented.
    #[allow(missing_docs)]
    pub direction: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub status: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub behavior: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sequence: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detail: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cooperation: rosidl_runtime_rs::BoundedSequence<super::super::msg::rmw::CooperationStatus, 1>,

}

impl SteeringFactor {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u16 = 0;

    /// constants for direction
    pub const LEFT: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RIGHT: u16 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STRAIGHT: u16 = 3;

    /// constants for status
    pub const APPROACHING: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TURNING: u16 = 3;

}


impl Default for SteeringFactor {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__SteeringFactor__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__SteeringFactor__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SteeringFactor {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__SteeringFactor__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__SteeringFactor__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__SteeringFactor__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SteeringFactor {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SteeringFactor where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/SteeringFactor";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__SteeringFactor() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__SteeringFactorArray() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__SteeringFactorArray__init(msg: *mut SteeringFactorArray) -> bool;
    fn autoware_adapi_v1_msgs__msg__SteeringFactorArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SteeringFactorArray>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__SteeringFactorArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SteeringFactorArray>);
    fn autoware_adapi_v1_msgs__msg__SteeringFactorArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SteeringFactorArray>, out_seq: *mut rosidl_runtime_rs::Sequence<SteeringFactorArray>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__SteeringFactorArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SteeringFactorArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub factors: rosidl_runtime_rs::Sequence<super::super::msg::rmw::SteeringFactor>,

}



impl Default for SteeringFactorArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__SteeringFactorArray__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__SteeringFactorArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SteeringFactorArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__SteeringFactorArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__SteeringFactorArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__SteeringFactorArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SteeringFactorArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SteeringFactorArray where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/SteeringFactorArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__SteeringFactorArray() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VelocityFactor() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__VelocityFactor__init(msg: *mut VelocityFactor) -> bool;
    fn autoware_adapi_v1_msgs__msg__VelocityFactor__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VelocityFactor>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__VelocityFactor__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VelocityFactor>);
    fn autoware_adapi_v1_msgs__msg__VelocityFactor__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VelocityFactor>, out_seq: *mut rosidl_runtime_rs::Sequence<VelocityFactor>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__VelocityFactor
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// constants for common use

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelocityFactor {
    /// variables
    pub pose: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub status: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub behavior: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sequence: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detail: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cooperation: rosidl_runtime_rs::BoundedSequence<super::super::msg::rmw::CooperationStatus, 1>,

}

impl VelocityFactor {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u16 = 0;

    /// constants for status
    pub const APPROACHING: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STOPPED: u16 = 2;

}


impl Default for VelocityFactor {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__VelocityFactor__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__VelocityFactor__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VelocityFactor {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VelocityFactor__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VelocityFactor__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VelocityFactor__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VelocityFactor {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VelocityFactor where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/VelocityFactor";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VelocityFactor() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VelocityFactorArray() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__VelocityFactorArray__init(msg: *mut VelocityFactorArray) -> bool;
    fn autoware_adapi_v1_msgs__msg__VelocityFactorArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VelocityFactorArray>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__VelocityFactorArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VelocityFactorArray>);
    fn autoware_adapi_v1_msgs__msg__VelocityFactorArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VelocityFactorArray>, out_seq: *mut rosidl_runtime_rs::Sequence<VelocityFactorArray>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__VelocityFactorArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelocityFactorArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub factors: rosidl_runtime_rs::Sequence<super::super::msg::rmw::VelocityFactor>,

}



impl Default for VelocityFactorArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__VelocityFactorArray__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__VelocityFactorArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VelocityFactorArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VelocityFactorArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VelocityFactorArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VelocityFactorArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VelocityFactorArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VelocityFactorArray where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/VelocityFactorArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VelocityFactorArray() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__PlanningBehavior() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__PlanningBehavior__init(msg: *mut PlanningBehavior) -> bool;
    fn autoware_adapi_v1_msgs__msg__PlanningBehavior__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlanningBehavior>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__PlanningBehavior__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlanningBehavior>);
    fn autoware_adapi_v1_msgs__msg__PlanningBehavior__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlanningBehavior>, out_seq: *mut rosidl_runtime_rs::Sequence<PlanningBehavior>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__PlanningBehavior
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// These constants are used in the behavior field of the SteeringFactor/VelocityFactor.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlanningBehavior {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}

impl PlanningBehavior {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: &'static str = "";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AVOIDANCE: &'static str = "avoidance";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CROSSWALK: &'static str = "crosswalk";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GOAL_PLANNER: &'static str = "goal-planner";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INTERSECTION: &'static str = "intersection";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LANE_CHANGE: &'static str = "lane-change";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MERGE: &'static str = "merge";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NO_DRIVABLE_LANE: &'static str = "no-drivable-lane";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NO_STOPPING_AREA: &'static str = "no-stopping-area";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const REAR_CHECK: &'static str = "rear-check";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ROUTE_OBSTACLE: &'static str = "route-obstacle";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SIDEWALK: &'static str = "sidewalk";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const START_PLANNER: &'static str = "start-planner";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STOP_SIGN: &'static str = "stop-sign";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SURROUNDING_OBSTACLE: &'static str = "surrounding-obstacle";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TRAFFIC_SIGNAL: &'static str = "traffic-signal";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const USER_DEFINED_DETECTION_AREA: &'static str = "user-defined-attention-area";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const VIRTUAL_TRAFFIC_LIGHT: &'static str = "virtual-traffic-light";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RUN_OUT: &'static str = "run-out";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ADAPTIVE_CRUISE: &'static str = "adaptive-cruise";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ROUNDABOUT: &'static str = "roundabout";

}


impl Default for PlanningBehavior {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__PlanningBehavior__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__PlanningBehavior__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlanningBehavior {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__PlanningBehavior__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__PlanningBehavior__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__PlanningBehavior__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlanningBehavior {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlanningBehavior where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/PlanningBehavior";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__PlanningBehavior() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__PlanningSequence() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__PlanningSequence__init(msg: *mut PlanningSequence) -> bool;
    fn autoware_adapi_v1_msgs__msg__PlanningSequence__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlanningSequence>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__PlanningSequence__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlanningSequence>);
    fn autoware_adapi_v1_msgs__msg__PlanningSequence__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlanningSequence>, out_seq: *mut rosidl_runtime_rs::Sequence<PlanningSequence>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__PlanningSequence
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// These constants are used in the sequence field of the SteeringFactor/VelocityFactor.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlanningSequence {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}

impl PlanningSequence {
    /// for avoidance behavior
    pub const CHANGE: &'static str = "change";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RETURN: &'static str = "return";

}


impl Default for PlanningSequence {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__PlanningSequence__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__PlanningSequence__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlanningSequence {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__PlanningSequence__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__PlanningSequence__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__PlanningSequence__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlanningSequence {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlanningSequence where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/PlanningSequence";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__PlanningSequence() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__CooperationCommand() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__CooperationCommand__init(msg: *mut CooperationCommand) -> bool;
    fn autoware_adapi_v1_msgs__msg__CooperationCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CooperationCommand>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__CooperationCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CooperationCommand>);
    fn autoware_adapi_v1_msgs__msg__CooperationCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CooperationCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<CooperationCommand>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__CooperationCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CooperationCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub uuid: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cooperator: super::super::msg::rmw::CooperationDecision,

}



impl Default for CooperationCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__CooperationCommand__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__CooperationCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CooperationCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__CooperationCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__CooperationCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__CooperationCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CooperationCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CooperationCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/CooperationCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__CooperationCommand() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__CooperationDecision() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__CooperationDecision__init(msg: *mut CooperationDecision) -> bool;
    fn autoware_adapi_v1_msgs__msg__CooperationDecision__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CooperationDecision>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__CooperationDecision__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CooperationDecision>);
    fn autoware_adapi_v1_msgs__msg__CooperationDecision__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CooperationDecision>, out_seq: *mut rosidl_runtime_rs::Sequence<CooperationDecision>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__CooperationDecision
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CooperationDecision {

    // This member is not documented.
    #[allow(missing_docs)]
    pub decision: u8,

}

impl CooperationDecision {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DEACTIVATE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACTIVATE: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AUTONOMOUS: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNDECIDED: u8 = 4;

}


impl Default for CooperationDecision {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__CooperationDecision__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__CooperationDecision__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CooperationDecision {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__CooperationDecision__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__CooperationDecision__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__CooperationDecision__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CooperationDecision {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CooperationDecision where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/CooperationDecision";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__CooperationDecision() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__CooperationPolicy() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__CooperationPolicy__init(msg: *mut CooperationPolicy) -> bool;
    fn autoware_adapi_v1_msgs__msg__CooperationPolicy__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CooperationPolicy>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__CooperationPolicy__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CooperationPolicy>);
    fn autoware_adapi_v1_msgs__msg__CooperationPolicy__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CooperationPolicy>, out_seq: *mut rosidl_runtime_rs::Sequence<CooperationPolicy>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__CooperationPolicy
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CooperationPolicy {

    // This member is not documented.
    #[allow(missing_docs)]
    pub behavior: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sequence: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub policy: u8,

}

impl CooperationPolicy {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const OPTIONAL: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const REQUIRED: u8 = 2;

}


impl Default for CooperationPolicy {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__CooperationPolicy__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__CooperationPolicy__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CooperationPolicy {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__CooperationPolicy__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__CooperationPolicy__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__CooperationPolicy__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CooperationPolicy {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CooperationPolicy where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/CooperationPolicy";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__CooperationPolicy() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__CooperationStatus() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__CooperationStatus__init(msg: *mut CooperationStatus) -> bool;
    fn autoware_adapi_v1_msgs__msg__CooperationStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CooperationStatus>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__CooperationStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CooperationStatus>);
    fn autoware_adapi_v1_msgs__msg__CooperationStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CooperationStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<CooperationStatus>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__CooperationStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CooperationStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub uuid: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub autonomous: super::super::msg::rmw::CooperationDecision,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cooperator: super::super::msg::rmw::CooperationDecision,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cancellable: bool,

}



impl Default for CooperationStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__CooperationStatus__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__CooperationStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CooperationStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__CooperationStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__CooperationStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__CooperationStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CooperationStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CooperationStatus where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/CooperationStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__CooperationStatus() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__RtiState() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__RtiState__init(msg: *mut RtiState) -> bool;
    fn autoware_adapi_v1_msgs__msg__RtiState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RtiState>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__RtiState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RtiState>);
    fn autoware_adapi_v1_msgs__msg__RtiState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RtiState>, out_seq: *mut rosidl_runtime_rs::Sequence<RtiState>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__RtiState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RtiState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub request: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for RtiState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__RtiState__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__RtiState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RtiState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RtiState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RtiState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__RtiState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RtiState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RtiState where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/RtiState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__RtiState() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__MrmState() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__MrmState__init(msg: *mut MrmState) -> bool;
    fn autoware_adapi_v1_msgs__msg__MrmState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MrmState>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__MrmState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MrmState>);
    fn autoware_adapi_v1_msgs__msg__MrmState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MrmState>, out_seq: *mut rosidl_runtime_rs::Sequence<MrmState>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__MrmState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MrmState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub behavior: u16,

}

impl MrmState {
    /// For common use
    pub const UNKNOWN: u16 = 0;

    /// For state
    pub const NORMAL: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MRM_OPERATING: u16 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MRM_SUCCEEDED: u16 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MRM_FAILED: u16 = 4;

    /// For behavior. Deprecated: use description API.
    /// https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/fail-safe/
    pub const NONE: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const EMERGENCY_STOP: u16 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const COMFORTABLE_STOP: u16 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PULL_OVER: u16 = 4;

}


impl Default for MrmState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__MrmState__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__MrmState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MrmState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MrmState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MrmState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MrmState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MrmState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MrmState where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/MrmState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__MrmState() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__MrmDescription() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__MrmDescription__init(msg: *mut MrmDescription) -> bool;
    fn autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MrmDescription>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MrmDescription>);
    fn autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MrmDescription>, out_seq: *mut rosidl_runtime_rs::Sequence<MrmDescription>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__MrmDescription
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MrmDescription {

    // This member is not documented.
    #[allow(missing_docs)]
    pub behavior: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub name: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub description: rosidl_runtime_rs::String,

}



impl Default for MrmDescription {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__MrmDescription__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__MrmDescription__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MrmDescription {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MrmDescription__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MrmDescription {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MrmDescription where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/MrmDescription";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__MrmDescription() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__MrmRequest() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__MrmRequest__init(msg: *mut MrmRequest) -> bool;
    fn autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MrmRequest>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MrmRequest>);
    fn autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MrmRequest>, out_seq: *mut rosidl_runtime_rs::Sequence<MrmRequest>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__MrmRequest
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MrmRequest {
    /// The identifier of the request sender.
    pub sender: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub strategy: u16,

}

impl MrmRequest {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u16 = 0;

    /// Cancel the MRM request.
    pub const CANCEL: u16 = 1;

    /// Delegate the selection of MRM behavior to Autoware.
    pub const DELEGATE: u16 = 2;

}


impl Default for MrmRequest {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__MrmRequest__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__MrmRequest__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MrmRequest {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MrmRequest__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MrmRequest {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MrmRequest where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/MrmRequest";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__MrmRequest() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__MrmRequestList() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__MrmRequestList__init(msg: *mut MrmRequestList) -> bool;
    fn autoware_adapi_v1_msgs__msg__MrmRequestList__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MrmRequestList>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__MrmRequestList__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MrmRequestList>);
    fn autoware_adapi_v1_msgs__msg__MrmRequestList__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MrmRequestList>, out_seq: *mut rosidl_runtime_rs::Sequence<MrmRequestList>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__MrmRequestList
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MrmRequestList {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub requests: rosidl_runtime_rs::Sequence<super::super::msg::rmw::MrmRequest>,

}



impl Default for MrmRequestList {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__MrmRequestList__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__MrmRequestList__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MrmRequestList {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MrmRequestList__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MrmRequestList__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__MrmRequestList__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MrmRequestList {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MrmRequestList where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/MrmRequestList";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__MrmRequestList() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__Heartbeat() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__Heartbeat__init(msg: *mut Heartbeat) -> bool;
    fn autoware_adapi_v1_msgs__msg__Heartbeat__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Heartbeat>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__Heartbeat__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Heartbeat>);
    fn autoware_adapi_v1_msgs__msg__Heartbeat__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Heartbeat>, out_seq: *mut rosidl_runtime_rs::Sequence<Heartbeat>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__Heartbeat
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Timestamp in Autoware for delay checking.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Heartbeat {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

    /// Sequence number for order verification, wraps at 65535.
    pub seq: u16,

}



impl Default for Heartbeat {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__Heartbeat__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__Heartbeat__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Heartbeat {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__Heartbeat__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__Heartbeat__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__Heartbeat__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Heartbeat {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Heartbeat where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/Heartbeat";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__Heartbeat() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagGraphStruct() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DiagGraphStruct__init(msg: *mut DiagGraphStruct) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagGraphStruct__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DiagGraphStruct>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagGraphStruct__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DiagGraphStruct>);
    fn autoware_adapi_v1_msgs__msg__DiagGraphStruct__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DiagGraphStruct>, out_seq: *mut rosidl_runtime_rs::Sequence<DiagGraphStruct>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DiagGraphStruct
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagGraphStruct {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub nodes: rosidl_runtime_rs::Sequence<super::super::msg::rmw::DiagNodeStruct>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub diags: rosidl_runtime_rs::Sequence<super::super::msg::rmw::DiagLeafStruct>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub links: rosidl_runtime_rs::Sequence<super::super::msg::rmw::DiagLinkStruct>,

}



impl Default for DiagGraphStruct {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DiagGraphStruct__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DiagGraphStruct__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DiagGraphStruct {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagGraphStruct__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagGraphStruct__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagGraphStruct__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DiagGraphStruct {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DiagGraphStruct where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DiagGraphStruct";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagGraphStruct() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagGraphStatus() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DiagGraphStatus__init(msg: *mut DiagGraphStatus) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagGraphStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DiagGraphStatus>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagGraphStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DiagGraphStatus>);
    fn autoware_adapi_v1_msgs__msg__DiagGraphStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DiagGraphStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<DiagGraphStatus>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DiagGraphStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagGraphStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub nodes: rosidl_runtime_rs::Sequence<super::super::msg::rmw::DiagNodeStatus>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub diags: rosidl_runtime_rs::Sequence<super::super::msg::rmw::DiagLeafStatus>,

}



impl Default for DiagGraphStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DiagGraphStatus__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DiagGraphStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DiagGraphStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagGraphStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagGraphStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagGraphStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DiagGraphStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DiagGraphStatus where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DiagGraphStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagGraphStatus() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagLeafStruct() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DiagLeafStruct__init(msg: *mut DiagLeafStruct) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DiagLeafStruct>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DiagLeafStruct>);
    fn autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DiagLeafStruct>, out_seq: *mut rosidl_runtime_rs::Sequence<DiagLeafStruct>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DiagLeafStruct
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagLeafStruct {

    // This member is not documented.
    #[allow(missing_docs)]
    pub parent: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub name: rosidl_runtime_rs::String,

}



impl Default for DiagLeafStruct {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DiagLeafStruct__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DiagLeafStruct__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DiagLeafStruct {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DiagLeafStruct {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DiagLeafStruct where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DiagLeafStruct";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagLeafStruct() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagLeafStatus() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DiagLeafStatus__init(msg: *mut DiagLeafStatus) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagLeafStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DiagLeafStatus>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagLeafStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DiagLeafStatus>);
    fn autoware_adapi_v1_msgs__msg__DiagLeafStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DiagLeafStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<DiagLeafStatus>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DiagLeafStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// The level of diagnostic_msgs/msg/DiagnosticStatus.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagLeafStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub level: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub input_level: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub hardware_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub values: rosidl_runtime_rs::Sequence<super::super::msg::rmw::KvString>,

}



impl Default for DiagLeafStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DiagLeafStatus__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DiagLeafStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DiagLeafStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagLeafStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagLeafStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagLeafStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DiagLeafStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DiagLeafStatus where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DiagLeafStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagLeafStatus() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagNodeStruct() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DiagNodeStruct__init(msg: *mut DiagNodeStruct) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DiagNodeStruct>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DiagNodeStruct>);
    fn autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DiagNodeStruct>, out_seq: *mut rosidl_runtime_rs::Sequence<DiagNodeStruct>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DiagNodeStruct
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagNodeStruct {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: rosidl_runtime_rs::String,

}



impl Default for DiagNodeStruct {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DiagNodeStruct__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DiagNodeStruct__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DiagNodeStruct {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DiagNodeStruct {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DiagNodeStruct where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DiagNodeStruct";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagNodeStruct() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagNodeStatus() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DiagNodeStatus__init(msg: *mut DiagNodeStatus) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DiagNodeStatus>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DiagNodeStatus>);
    fn autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DiagNodeStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<DiagNodeStatus>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DiagNodeStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// The level of diagnostic_msgs/msg/DiagnosticStatus.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagNodeStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub level: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub input_level: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub latch_level: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_dependent: bool,

}



impl Default for DiagNodeStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DiagNodeStatus__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DiagNodeStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DiagNodeStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagNodeStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DiagNodeStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DiagNodeStatus where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DiagNodeStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagNodeStatus() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagLinkStruct() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DiagLinkStruct__init(msg: *mut DiagLinkStruct) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DiagLinkStruct>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DiagLinkStruct>);
    fn autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DiagLinkStruct>, out_seq: *mut rosidl_runtime_rs::Sequence<DiagLinkStruct>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DiagLinkStruct
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// The index of nodes in the graph struct message.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagLinkStruct {

    // This member is not documented.
    #[allow(missing_docs)]
    pub parent: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub child: u32,

}



impl Default for DiagLinkStruct {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DiagLinkStruct__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DiagLinkStruct__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DiagLinkStruct {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DiagLinkStruct {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DiagLinkStruct where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DiagLinkStruct";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DiagLinkStruct() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DoorCommand() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DoorCommand__init(msg: *mut DoorCommand) -> bool;
    fn autoware_adapi_v1_msgs__msg__DoorCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DoorCommand>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DoorCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DoorCommand>);
    fn autoware_adapi_v1_msgs__msg__DoorCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DoorCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<DoorCommand>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DoorCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DoorCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub index: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub command: u8,

}

impl DoorCommand {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const OPEN: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLOSE: u8 = 2;

}


impl Default for DoorCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DoorCommand__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DoorCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DoorCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DoorCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DoorCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DoorCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DoorCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DoorCommand where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DoorCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DoorCommand() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DoorLayout() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DoorLayout__init(msg: *mut DoorLayout) -> bool;
    fn autoware_adapi_v1_msgs__msg__DoorLayout__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DoorLayout>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DoorLayout__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DoorLayout>);
    fn autoware_adapi_v1_msgs__msg__DoorLayout__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DoorLayout>, out_seq: *mut rosidl_runtime_rs::Sequence<DoorLayout>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DoorLayout
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DoorLayout {

    // This member is not documented.
    #[allow(missing_docs)]
    pub roles: rosidl_runtime_rs::Sequence<u8>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub description: rosidl_runtime_rs::String,

}

impl DoorLayout {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GET_ON: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GET_OFF: u8 = 2;

}


impl Default for DoorLayout {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DoorLayout__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DoorLayout__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DoorLayout {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DoorLayout__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DoorLayout__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DoorLayout__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DoorLayout {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DoorLayout where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DoorLayout";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DoorLayout() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DoorStatus() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DoorStatus__init(msg: *mut DoorStatus) -> bool;
    fn autoware_adapi_v1_msgs__msg__DoorStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DoorStatus>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DoorStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DoorStatus>);
    fn autoware_adapi_v1_msgs__msg__DoorStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DoorStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<DoorStatus>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DoorStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DoorStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: u8,

}

impl DoorStatus {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NOT_AVAILABLE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const OPENED: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLOSED: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const OPENING: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLOSING: u8 = 5;

}


impl Default for DoorStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DoorStatus__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DoorStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DoorStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DoorStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DoorStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DoorStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DoorStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DoorStatus where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DoorStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DoorStatus() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DoorStatusArray() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__DoorStatusArray__init(msg: *mut DoorStatusArray) -> bool;
    fn autoware_adapi_v1_msgs__msg__DoorStatusArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DoorStatusArray>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__DoorStatusArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DoorStatusArray>);
    fn autoware_adapi_v1_msgs__msg__DoorStatusArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DoorStatusArray>, out_seq: *mut rosidl_runtime_rs::Sequence<DoorStatusArray>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__DoorStatusArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DoorStatusArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub doors: rosidl_runtime_rs::Sequence<super::super::msg::rmw::DoorStatus>,

}



impl Default for DoorStatusArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__DoorStatusArray__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__DoorStatusArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DoorStatusArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DoorStatusArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DoorStatusArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__DoorStatusArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DoorStatusArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DoorStatusArray where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/DoorStatusArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__DoorStatusArray() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__Gear() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__Gear__init(msg: *mut Gear) -> bool;
    fn autoware_adapi_v1_msgs__msg__Gear__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Gear>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__Gear__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Gear>);
    fn autoware_adapi_v1_msgs__msg__Gear__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Gear>, out_seq: *mut rosidl_runtime_rs::Sequence<Gear>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__Gear
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// constants

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Gear {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: u8,

}

impl Gear {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NEUTRAL: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DRIVE: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const REVERSE: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PARK: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LOW: u8 = 5;

}


impl Default for Gear {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__Gear__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__Gear__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Gear {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__Gear__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__Gear__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__Gear__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Gear {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Gear where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/Gear";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__Gear() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__HazardLights() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__HazardLights__init(msg: *mut HazardLights) -> bool;
    fn autoware_adapi_v1_msgs__msg__HazardLights__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<HazardLights>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__HazardLights__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<HazardLights>);
    fn autoware_adapi_v1_msgs__msg__HazardLights__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<HazardLights>, out_seq: *mut rosidl_runtime_rs::Sequence<HazardLights>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__HazardLights
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// constants

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HazardLights {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: u8,

}

impl HazardLights {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DISABLE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ENABLE: u8 = 2;

}


impl Default for HazardLights {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__HazardLights__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__HazardLights__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for HazardLights {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__HazardLights__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__HazardLights__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__HazardLights__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for HazardLights {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for HazardLights where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/HazardLights";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__HazardLights() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__TurnIndicators() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__TurnIndicators__init(msg: *mut TurnIndicators) -> bool;
    fn autoware_adapi_v1_msgs__msg__TurnIndicators__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TurnIndicators>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__TurnIndicators__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TurnIndicators>);
    fn autoware_adapi_v1_msgs__msg__TurnIndicators__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TurnIndicators>, out_seq: *mut rosidl_runtime_rs::Sequence<TurnIndicators>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__TurnIndicators
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// constants

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TurnIndicators {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: u8,

}

impl TurnIndicators {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DISABLE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LEFT: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RIGHT: u8 = 3;

}


impl Default for TurnIndicators {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__TurnIndicators__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__TurnIndicators__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TurnIndicators {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__TurnIndicators__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__TurnIndicators__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__TurnIndicators__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TurnIndicators {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TurnIndicators where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/TurnIndicators";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__TurnIndicators() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VehicleMetrics() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__VehicleMetrics__init(msg: *mut VehicleMetrics) -> bool;
    fn autoware_adapi_v1_msgs__msg__VehicleMetrics__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VehicleMetrics>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__VehicleMetrics__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VehicleMetrics>);
    fn autoware_adapi_v1_msgs__msg__VehicleMetrics__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VehicleMetrics>, out_seq: *mut rosidl_runtime_rs::Sequence<VehicleMetrics>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__VehicleMetrics
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleMetrics {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

    /// The remaining vehicle fuel or battery. Ratio with the maximum as 1.0.
    pub energy: f32,

}



impl Default for VehicleMetrics {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__VehicleMetrics__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__VehicleMetrics__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VehicleMetrics {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleMetrics__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleMetrics__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleMetrics__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VehicleMetrics {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VehicleMetrics where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/VehicleMetrics";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VehicleMetrics() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VehicleStatus() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__VehicleStatus__init(msg: *mut VehicleStatus) -> bool;
    fn autoware_adapi_v1_msgs__msg__VehicleStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VehicleStatus>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__VehicleStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VehicleStatus>);
    fn autoware_adapi_v1_msgs__msg__VehicleStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VehicleStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<VehicleStatus>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__VehicleStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub gear: super::super::msg::rmw::Gear,


    // This member is not documented.
    #[allow(missing_docs)]
    pub turn_indicators: super::super::msg::rmw::TurnIndicators,


    // This member is not documented.
    #[allow(missing_docs)]
    pub hazard_lights: super::super::msg::rmw::HazardLights,


    // This member is not documented.
    #[allow(missing_docs)]
    pub steering_tire_angle: f64,

}



impl Default for VehicleStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__VehicleStatus__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__VehicleStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VehicleStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VehicleStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VehicleStatus where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/VehicleStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VehicleStatus() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VehicleDimensions() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__VehicleDimensions__init(msg: *mut VehicleDimensions) -> bool;
    fn autoware_adapi_v1_msgs__msg__VehicleDimensions__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VehicleDimensions>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__VehicleDimensions__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VehicleDimensions>);
    fn autoware_adapi_v1_msgs__msg__VehicleDimensions__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VehicleDimensions>, out_seq: *mut rosidl_runtime_rs::Sequence<VehicleDimensions>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__VehicleDimensions
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleDimensions {

    // This member is not documented.
    #[allow(missing_docs)]
    pub wheel_radius: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub wheel_width: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub wheel_base: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub wheel_tread: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub front_overhang: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rear_overhang: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub left_overhang: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_overhang: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub height: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub footprint: geometry_msgs::msg::rmw::Polygon,

}



impl Default for VehicleDimensions {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__VehicleDimensions__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__VehicleDimensions__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VehicleDimensions {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleDimensions__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleDimensions__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleDimensions__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VehicleDimensions {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VehicleDimensions where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/VehicleDimensions";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VehicleDimensions() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VehicleSpecs() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__VehicleSpecs__init(msg: *mut VehicleSpecs) -> bool;
    fn autoware_adapi_v1_msgs__msg__VehicleSpecs__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VehicleSpecs>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__VehicleSpecs__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VehicleSpecs>);
    fn autoware_adapi_v1_msgs__msg__VehicleSpecs__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VehicleSpecs>, out_seq: *mut rosidl_runtime_rs::Sequence<VehicleSpecs>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__VehicleSpecs
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleSpecs {

    // This member is not documented.
    #[allow(missing_docs)]
    pub max_steering_tire_angle: f32,

}



impl Default for VehicleSpecs {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__VehicleSpecs__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__VehicleSpecs__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VehicleSpecs {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleSpecs__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleSpecs__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleSpecs__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VehicleSpecs {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VehicleSpecs where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/VehicleSpecs";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VehicleSpecs() }
  }
}


#[link(name = "autoware_adapi_v1_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VehicleKinematics() -> *const std::ffi::c_void;
}

#[link(name = "autoware_adapi_v1_msgs__rosidl_generator_c")]
extern "C" {
    fn autoware_adapi_v1_msgs__msg__VehicleKinematics__init(msg: *mut VehicleKinematics) -> bool;
    fn autoware_adapi_v1_msgs__msg__VehicleKinematics__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VehicleKinematics>, size: usize) -> bool;
    fn autoware_adapi_v1_msgs__msg__VehicleKinematics__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VehicleKinematics>);
    fn autoware_adapi_v1_msgs__msg__VehicleKinematics__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VehicleKinematics>, out_seq: *mut rosidl_runtime_rs::Sequence<VehicleKinematics>) -> bool;
}

// Corresponds to autoware_adapi_v1_msgs__msg__VehicleKinematics
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Geographic point, using the WGS 84 reference ellipsoid.
/// This data will be invalid If Autoware does not provide projection information between geographic coordinates and local coordinates.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleKinematics {

    // This member is not documented.
    #[allow(missing_docs)]
    pub geographic_pose: geographic_msgs::msg::rmw::GeoPointStamped,

    /// Local coordinate from the autoware
    pub pose: geometry_msgs::msg::rmw::PoseWithCovarianceStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub twist: geometry_msgs::msg::rmw::TwistWithCovarianceStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub accel: geometry_msgs::msg::rmw::AccelWithCovarianceStamped,

}



impl Default for VehicleKinematics {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !autoware_adapi_v1_msgs__msg__VehicleKinematics__init(&mut msg as *mut _) {
        panic!("Call to autoware_adapi_v1_msgs__msg__VehicleKinematics__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VehicleKinematics {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleKinematics__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleKinematics__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { autoware_adapi_v1_msgs__msg__VehicleKinematics__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VehicleKinematics {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VehicleKinematics where Self: Sized {
  const TYPE_NAME: &'static str = "autoware_adapi_v1_msgs/msg/VehicleKinematics";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__autoware_adapi_v1_msgs__msg__VehicleKinematics() }
  }
}


