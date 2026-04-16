#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to autoware_adapi_v1_msgs__msg__ResponseStatus
/// error code

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ResponseStatus {
    /// variables
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub code: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ResponseStatus::default())
  }
}

impl rosidl_runtime_rs::Message for ResponseStatus {
  type RmwMsg = super::msg::rmw::ResponseStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        code: msg.code,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      code: msg.code,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      code: msg.code,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__KvString

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct KvString {

    // This member is not documented.
    #[allow(missing_docs)]
    pub key: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub value: std::string::String,

}



impl Default for KvString {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::KvString::default())
  }
}

impl rosidl_runtime_rs::Message for KvString {
  type RmwMsg = super::msg::rmw::KvString;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        key: msg.key.as_str().into(),
        value: msg.value.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        key: msg.key.as_str().into(),
        value: msg.value.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      key: msg.key.to_string(),
      value: msg.value.to_string(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__LocalizationInitializationState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LocalizationInitializationState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LocalizationInitializationState::default())
  }
}

impl rosidl_runtime_rs::Message for LocalizationInitializationState {
  type RmwMsg = super::msg::rmw::LocalizationInitializationState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        state: msg.state,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      state: msg.state,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      state: msg.state,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__PedalsCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PedalsCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub throttle: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub brake: f32,

}



impl Default for PedalsCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PedalsCommand::default())
  }
}

impl rosidl_runtime_rs::Message for PedalsCommand {
  type RmwMsg = super::msg::rmw::PedalsCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        throttle: msg.throttle,
        brake: msg.brake,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      throttle: msg.throttle,
      brake: msg.brake,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      throttle: msg.throttle,
      brake: msg.brake,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__AccelerationCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AccelerationCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub acceleration: f32,

}



impl Default for AccelerationCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AccelerationCommand::default())
  }
}

impl rosidl_runtime_rs::Message for AccelerationCommand {
  type RmwMsg = super::msg::rmw::AccelerationCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        acceleration: msg.acceleration,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      acceleration: msg.acceleration,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      acceleration: msg.acceleration,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__VelocityCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelocityCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: f32,

}



impl Default for VelocityCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VelocityCommand::default())
  }
}

impl rosidl_runtime_rs::Message for VelocityCommand {
  type RmwMsg = super::msg::rmw::VelocityCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        velocity: msg.velocity,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      velocity: msg.velocity,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      velocity: msg.velocity,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__SteeringCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SteeringCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub steering_tire_angle: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub steering_tire_velocity: f32,

}



impl Default for SteeringCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SteeringCommand::default())
  }
}

impl rosidl_runtime_rs::Message for SteeringCommand {
  type RmwMsg = super::msg::rmw::SteeringCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        steering_tire_angle: msg.steering_tire_angle,
        steering_tire_velocity: msg.steering_tire_velocity,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      steering_tire_angle: msg.steering_tire_angle,
      steering_tire_velocity: msg.steering_tire_velocity,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      steering_tire_angle: msg.steering_tire_angle,
      steering_tire_velocity: msg.steering_tire_velocity,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__GearCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GearCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub command: super::msg::Gear,

}



impl Default for GearCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::GearCommand::default())
  }
}

impl rosidl_runtime_rs::Message for GearCommand {
  type RmwMsg = super::msg::rmw::GearCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        command: super::msg::Gear::into_rmw_message(std::borrow::Cow::Owned(msg.command)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        command: super::msg::Gear::into_rmw_message(std::borrow::Cow::Borrowed(&msg.command)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      command: super::msg::Gear::from_rmw_message(msg.command),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TurnIndicatorsCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub command: super::msg::TurnIndicators,

}



impl Default for TurnIndicatorsCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TurnIndicatorsCommand::default())
  }
}

impl rosidl_runtime_rs::Message for TurnIndicatorsCommand {
  type RmwMsg = super::msg::rmw::TurnIndicatorsCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        command: super::msg::TurnIndicators::into_rmw_message(std::borrow::Cow::Owned(msg.command)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        command: super::msg::TurnIndicators::into_rmw_message(std::borrow::Cow::Borrowed(&msg.command)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      command: super::msg::TurnIndicators::from_rmw_message(msg.command),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__HazardLightsCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HazardLightsCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub command: super::msg::HazardLights,

}



impl Default for HazardLightsCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::HazardLightsCommand::default())
  }
}

impl rosidl_runtime_rs::Message for HazardLightsCommand {
  type RmwMsg = super::msg::rmw::HazardLightsCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        command: super::msg::HazardLights::into_rmw_message(std::borrow::Cow::Owned(msg.command)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        command: super::msg::HazardLights::into_rmw_message(std::borrow::Cow::Borrowed(&msg.command)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      command: super::msg::HazardLights::from_rmw_message(msg.command),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__ManualControlMode

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ManualControlMode::default())
  }
}

impl rosidl_runtime_rs::Message for ManualControlMode {
  type RmwMsg = super::msg::rmw::ManualControlMode;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        mode: msg.mode,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      mode: msg.mode,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      mode: msg.mode,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__ManualControlModeStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManualControlModeStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: super::msg::ManualControlMode,

}



impl Default for ManualControlModeStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ManualControlModeStatus::default())
  }
}

impl rosidl_runtime_rs::Message for ManualControlModeStatus {
  type RmwMsg = super::msg::rmw::ManualControlModeStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        mode: super::msg::ManualControlMode::into_rmw_message(std::borrow::Cow::Owned(msg.mode)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        mode: super::msg::ManualControlMode::into_rmw_message(std::borrow::Cow::Borrowed(&msg.mode)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      mode: super::msg::ManualControlMode::from_rmw_message(msg.mode),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__ManualOperatorHeartbeat

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManualOperatorHeartbeat {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ready: bool,

}



impl Default for ManualOperatorHeartbeat {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ManualOperatorHeartbeat::default())
  }
}

impl rosidl_runtime_rs::Message for ManualOperatorHeartbeat {
  type RmwMsg = super::msg::rmw::ManualOperatorHeartbeat;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        ready: msg.ready,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      ready: msg.ready,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      ready: msg.ready,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__RouteState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RouteState::default())
  }
}

impl rosidl_runtime_rs::Message for RouteState {
  type RmwMsg = super::msg::rmw::RouteState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        state: msg.state,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      state: msg.state,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      state: msg.state,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__Route

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Route {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub data: rosidl_runtime_rs::BoundedSequence<super::msg::rmw::RouteData, 1>,

}



impl Default for Route {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Route::default())
  }
}

impl rosidl_runtime_rs::Message for Route {
  type RmwMsg = super::msg::rmw::Route;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        data: msg.data,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        data: msg.data.clone(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      data: msg.data,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__RouteData

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub segments: Vec<super::msg::RouteSegment>,

}



impl Default for RouteData {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RouteData::default())
  }
}

impl rosidl_runtime_rs::Message for RouteData {
  type RmwMsg = super::msg::rmw::RouteData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        start: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.start)).into_owned(),
        goal: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
        segments: msg.segments
          .into_iter()
          .map(|elem| super::msg::RouteSegment::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        start: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start)).into_owned(),
        goal: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
        segments: msg.segments
          .iter()
          .map(|elem| super::msg::RouteSegment::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      start: geometry_msgs::msg::Pose::from_rmw_message(msg.start),
      goal: geometry_msgs::msg::Pose::from_rmw_message(msg.goal),
      segments: msg.segments
          .into_iter()
          .map(super::msg::RouteSegment::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__RouteOption
/// Please refer to the following pages for details on each option.
/// https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/routing/

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteOption {

    // This member is not documented.
    #[allow(missing_docs)]
    pub allow_goal_modification: bool,

}



impl Default for RouteOption {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RouteOption::default())
  }
}

impl rosidl_runtime_rs::Message for RouteOption {
  type RmwMsg = super::msg::rmw::RouteOption;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        allow_goal_modification: msg.allow_goal_modification,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      allow_goal_modification: msg.allow_goal_modification,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      allow_goal_modification: msg.allow_goal_modification,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__RoutePrimitive

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RoutePrimitive {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: i64,

    /// The same id may be used for each type.
    pub type_: std::string::String,

}



impl Default for RoutePrimitive {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RoutePrimitive::default())
  }
}

impl rosidl_runtime_rs::Message for RoutePrimitive {
  type RmwMsg = super::msg::rmw::RoutePrimitive;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        type_: msg.type_.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
        type_: msg.type_.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      type_: msg.type_.to_string(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__RouteSegment

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteSegment {

    // This member is not documented.
    #[allow(missing_docs)]
    pub preferred: super::msg::RoutePrimitive,

    /// Does not include the preferred primitive.
    pub alternatives: Vec<super::msg::RoutePrimitive>,

}



impl Default for RouteSegment {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RouteSegment::default())
  }
}

impl rosidl_runtime_rs::Message for RouteSegment {
  type RmwMsg = super::msg::rmw::RouteSegment;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        preferred: super::msg::RoutePrimitive::into_rmw_message(std::borrow::Cow::Owned(msg.preferred)).into_owned(),
        alternatives: msg.alternatives
          .into_iter()
          .map(|elem| super::msg::RoutePrimitive::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        preferred: super::msg::RoutePrimitive::into_rmw_message(std::borrow::Cow::Borrowed(&msg.preferred)).into_owned(),
        alternatives: msg.alternatives
          .iter()
          .map(|elem| super::msg::RoutePrimitive::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      preferred: super::msg::RoutePrimitive::from_rmw_message(msg.preferred),
      alternatives: msg.alternatives
          .into_iter()
          .map(super::msg::RoutePrimitive::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__OperationModeState
/// constants for mode

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperationModeState {
    /// variables
    pub stamp: builtin_interfaces::msg::Time,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::OperationModeState::default())
  }
}

impl rosidl_runtime_rs::Message for OperationModeState {
  type RmwMsg = super::msg::rmw::OperationModeState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        mode: msg.mode,
        is_autoware_control_enabled: msg.is_autoware_control_enabled,
        is_in_transition: msg.is_in_transition,
        is_stop_mode_available: msg.is_stop_mode_available,
        is_autonomous_mode_available: msg.is_autonomous_mode_available,
        is_local_mode_available: msg.is_local_mode_available,
        is_remote_mode_available: msg.is_remote_mode_available,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      mode: msg.mode,
      is_autoware_control_enabled: msg.is_autoware_control_enabled,
      is_in_transition: msg.is_in_transition,
      is_stop_mode_available: msg.is_stop_mode_available,
      is_autonomous_mode_available: msg.is_autonomous_mode_available,
      is_local_mode_available: msg.is_local_mode_available,
      is_remote_mode_available: msg.is_remote_mode_available,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      mode: msg.mode,
      is_autoware_control_enabled: msg.is_autoware_control_enabled,
      is_in_transition: msg.is_in_transition,
      is_stop_mode_available: msg.is_stop_mode_available,
      is_autonomous_mode_available: msg.is_autonomous_mode_available,
      is_local_mode_available: msg.is_local_mode_available,
      is_remote_mode_available: msg.is_remote_mode_available,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__MotionState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotionState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MotionState::default())
  }
}

impl rosidl_runtime_rs::Message for MotionState {
  type RmwMsg = super::msg::rmw::MotionState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        state: msg.state,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      state: msg.state,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      state: msg.state,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DynamicObject

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DynamicObject {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub existence_probability: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub classification: Vec<super::msg::ObjectClassification>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kinematics: super::msg::DynamicObjectKinematics,


    // This member is not documented.
    #[allow(missing_docs)]
    pub shape: shape_msgs::msg::SolidPrimitive,

}



impl Default for DynamicObject {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DynamicObject::default())
  }
}

impl rosidl_runtime_rs::Message for DynamicObject {
  type RmwMsg = super::msg::rmw::DynamicObject;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.id)).into_owned(),
        existence_probability: msg.existence_probability,
        classification: msg.classification
          .into_iter()
          .map(|elem| super::msg::ObjectClassification::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        kinematics: super::msg::DynamicObjectKinematics::into_rmw_message(std::borrow::Cow::Owned(msg.kinematics)).into_owned(),
        shape: shape_msgs::msg::SolidPrimitive::into_rmw_message(std::borrow::Cow::Owned(msg.shape)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.id)).into_owned(),
      existence_probability: msg.existence_probability,
        classification: msg.classification
          .iter()
          .map(|elem| super::msg::ObjectClassification::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        kinematics: super::msg::DynamicObjectKinematics::into_rmw_message(std::borrow::Cow::Borrowed(&msg.kinematics)).into_owned(),
        shape: shape_msgs::msg::SolidPrimitive::into_rmw_message(std::borrow::Cow::Borrowed(&msg.shape)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.id),
      existence_probability: msg.existence_probability,
      classification: msg.classification
          .into_iter()
          .map(super::msg::ObjectClassification::from_rmw_message)
          .collect(),
      kinematics: super::msg::DynamicObjectKinematics::from_rmw_message(msg.kinematics),
      shape: shape_msgs::msg::SolidPrimitive::from_rmw_message(msg.shape),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DynamicObjectArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DynamicObjectArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub objects: Vec<super::msg::DynamicObject>,

}



impl Default for DynamicObjectArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DynamicObjectArray::default())
  }
}

impl rosidl_runtime_rs::Message for DynamicObjectArray {
  type RmwMsg = super::msg::rmw::DynamicObjectArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        objects: msg.objects
          .into_iter()
          .map(|elem| super::msg::DynamicObject::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        objects: msg.objects
          .iter()
          .map(|elem| super::msg::DynamicObject::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      objects: msg.objects
          .into_iter()
          .map(super::msg::DynamicObject::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DynamicObjectKinematics

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DynamicObjectKinematics {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub twist: geometry_msgs::msg::Twist,


    // This member is not documented.
    #[allow(missing_docs)]
    pub accel: geometry_msgs::msg::Accel,


    // This member is not documented.
    #[allow(missing_docs)]
    pub predicted_paths: Vec<super::msg::DynamicObjectPath>,

}



impl Default for DynamicObjectKinematics {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DynamicObjectKinematics::default())
  }
}

impl rosidl_runtime_rs::Message for DynamicObjectKinematics {
  type RmwMsg = super::msg::rmw::DynamicObjectKinematics;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        twist: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(msg.twist)).into_owned(),
        accel: geometry_msgs::msg::Accel::into_rmw_message(std::borrow::Cow::Owned(msg.accel)).into_owned(),
        predicted_paths: msg.predicted_paths
          .into_iter()
          .map(|elem| super::msg::DynamicObjectPath::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
        twist: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(&msg.twist)).into_owned(),
        accel: geometry_msgs::msg::Accel::into_rmw_message(std::borrow::Cow::Borrowed(&msg.accel)).into_owned(),
        predicted_paths: msg.predicted_paths
          .iter()
          .map(|elem| super::msg::DynamicObjectPath::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
      twist: geometry_msgs::msg::Twist::from_rmw_message(msg.twist),
      accel: geometry_msgs::msg::Accel::from_rmw_message(msg.accel),
      predicted_paths: msg.predicted_paths
          .into_iter()
          .map(super::msg::DynamicObjectPath::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DynamicObjectPath

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DynamicObjectPath {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: Vec<geometry_msgs::msg::Pose>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time_step: builtin_interfaces::msg::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub confidence: f64,

}



impl Default for DynamicObjectPath {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DynamicObjectPath::default())
  }
}

impl rosidl_runtime_rs::Message for DynamicObjectPath {
  type RmwMsg = super::msg::rmw::DynamicObjectPath;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path
          .into_iter()
          .map(|elem| geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        time_step: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_step)).into_owned(),
        confidence: msg.confidence,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path
          .iter()
          .map(|elem| geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        time_step: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_step)).into_owned(),
      confidence: msg.confidence,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: msg.path
          .into_iter()
          .map(geometry_msgs::msg::Pose::from_rmw_message)
          .collect(),
      time_step: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_step),
      confidence: msg.confidence,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__ObjectClassification

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ObjectClassification::default())
  }
}

impl rosidl_runtime_rs::Message for ObjectClassification {
  type RmwMsg = super::msg::rmw::ObjectClassification;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        label: msg.label,
        probability: msg.probability,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      label: msg.label,
      probability: msg.probability,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      label: msg.label,
      probability: msg.probability,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__SteeringFactor
/// constants for common use

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SteeringFactor {
    /// variables
    pub pose: [geometry_msgs::msg::Pose; 2],


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
    pub behavior: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sequence: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detail: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cooperation: rosidl_runtime_rs::BoundedSequence<super::msg::rmw::CooperationStatus, 1>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SteeringFactor::default())
  }
}

impl rosidl_runtime_rs::Message for SteeringFactor {
  type RmwMsg = super::msg::rmw::SteeringFactor;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: msg.pose
          .map(|elem| geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned()),
        distance: msg.distance,
        direction: msg.direction,
        status: msg.status,
        behavior: msg.behavior.as_str().into(),
        sequence: msg.sequence.as_str().into(),
        detail: msg.detail.as_str().into(),
        cooperation: msg.cooperation,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: msg.pose
          .iter()
          .map(|elem| geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect::<Vec<_>>()
          .try_into()
          .unwrap(),
        distance: msg.distance,
      direction: msg.direction,
      status: msg.status,
        behavior: msg.behavior.as_str().into(),
        sequence: msg.sequence.as_str().into(),
        detail: msg.detail.as_str().into(),
        cooperation: msg.cooperation.clone(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: msg.pose
        .map(geometry_msgs::msg::Pose::from_rmw_message),
      distance: msg.distance,
      direction: msg.direction,
      status: msg.status,
      behavior: msg.behavior.to_string(),
      sequence: msg.sequence.to_string(),
      detail: msg.detail.to_string(),
      cooperation: msg.cooperation,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__SteeringFactorArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SteeringFactorArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub factors: Vec<super::msg::SteeringFactor>,

}



impl Default for SteeringFactorArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SteeringFactorArray::default())
  }
}

impl rosidl_runtime_rs::Message for SteeringFactorArray {
  type RmwMsg = super::msg::rmw::SteeringFactorArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        factors: msg.factors
          .into_iter()
          .map(|elem| super::msg::SteeringFactor::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        factors: msg.factors
          .iter()
          .map(|elem| super::msg::SteeringFactor::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      factors: msg.factors
          .into_iter()
          .map(super::msg::SteeringFactor::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__VelocityFactor
/// constants for common use

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelocityFactor {
    /// variables
    pub pose: geometry_msgs::msg::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub status: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub behavior: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sequence: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detail: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cooperation: rosidl_runtime_rs::BoundedSequence<super::msg::rmw::CooperationStatus, 1>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VelocityFactor::default())
  }
}

impl rosidl_runtime_rs::Message for VelocityFactor {
  type RmwMsg = super::msg::rmw::VelocityFactor;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        distance: msg.distance,
        status: msg.status,
        behavior: msg.behavior.as_str().into(),
        sequence: msg.sequence.as_str().into(),
        detail: msg.detail.as_str().into(),
        cooperation: msg.cooperation,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      distance: msg.distance,
      status: msg.status,
        behavior: msg.behavior.as_str().into(),
        sequence: msg.sequence.as_str().into(),
        detail: msg.detail.as_str().into(),
        cooperation: msg.cooperation.clone(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
      distance: msg.distance,
      status: msg.status,
      behavior: msg.behavior.to_string(),
      sequence: msg.sequence.to_string(),
      detail: msg.detail.to_string(),
      cooperation: msg.cooperation,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__VelocityFactorArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelocityFactorArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub factors: Vec<super::msg::VelocityFactor>,

}



impl Default for VelocityFactorArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VelocityFactorArray::default())
  }
}

impl rosidl_runtime_rs::Message for VelocityFactorArray {
  type RmwMsg = super::msg::rmw::VelocityFactorArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        factors: msg.factors
          .into_iter()
          .map(|elem| super::msg::VelocityFactor::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        factors: msg.factors
          .iter()
          .map(|elem| super::msg::VelocityFactor::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      factors: msg.factors
          .into_iter()
          .map(super::msg::VelocityFactor::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__PlanningBehavior
/// These constants are used in the behavior field of the SteeringFactor/VelocityFactor.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PlanningBehavior::default())
  }
}

impl rosidl_runtime_rs::Message for PlanningBehavior {
  type RmwMsg = super::msg::rmw::PlanningBehavior;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__PlanningSequence
/// These constants are used in the sequence field of the SteeringFactor/VelocityFactor.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PlanningSequence::default())
  }
}

impl rosidl_runtime_rs::Message for PlanningSequence {
  type RmwMsg = super::msg::rmw::PlanningSequence;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__CooperationCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CooperationCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub uuid: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cooperator: super::msg::CooperationDecision,

}



impl Default for CooperationCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CooperationCommand::default())
  }
}

impl rosidl_runtime_rs::Message for CooperationCommand {
  type RmwMsg = super::msg::rmw::CooperationCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        uuid: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.uuid)).into_owned(),
        cooperator: super::msg::CooperationDecision::into_rmw_message(std::borrow::Cow::Owned(msg.cooperator)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        uuid: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.uuid)).into_owned(),
        cooperator: super::msg::CooperationDecision::into_rmw_message(std::borrow::Cow::Borrowed(&msg.cooperator)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      uuid: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.uuid),
      cooperator: super::msg::CooperationDecision::from_rmw_message(msg.cooperator),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__CooperationDecision

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CooperationDecision::default())
  }
}

impl rosidl_runtime_rs::Message for CooperationDecision {
  type RmwMsg = super::msg::rmw::CooperationDecision;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        decision: msg.decision,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      decision: msg.decision,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      decision: msg.decision,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__CooperationPolicy

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CooperationPolicy {

    // This member is not documented.
    #[allow(missing_docs)]
    pub behavior: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sequence: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CooperationPolicy::default())
  }
}

impl rosidl_runtime_rs::Message for CooperationPolicy {
  type RmwMsg = super::msg::rmw::CooperationPolicy;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        behavior: msg.behavior.as_str().into(),
        sequence: msg.sequence.as_str().into(),
        policy: msg.policy,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        behavior: msg.behavior.as_str().into(),
        sequence: msg.sequence.as_str().into(),
      policy: msg.policy,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      behavior: msg.behavior.to_string(),
      sequence: msg.sequence.to_string(),
      policy: msg.policy,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__CooperationStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CooperationStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub uuid: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub autonomous: super::msg::CooperationDecision,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cooperator: super::msg::CooperationDecision,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cancellable: bool,

}



impl Default for CooperationStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CooperationStatus::default())
  }
}

impl rosidl_runtime_rs::Message for CooperationStatus {
  type RmwMsg = super::msg::rmw::CooperationStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        uuid: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.uuid)).into_owned(),
        autonomous: super::msg::CooperationDecision::into_rmw_message(std::borrow::Cow::Owned(msg.autonomous)).into_owned(),
        cooperator: super::msg::CooperationDecision::into_rmw_message(std::borrow::Cow::Owned(msg.cooperator)).into_owned(),
        cancellable: msg.cancellable,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        uuid: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.uuid)).into_owned(),
        autonomous: super::msg::CooperationDecision::into_rmw_message(std::borrow::Cow::Borrowed(&msg.autonomous)).into_owned(),
        cooperator: super::msg::CooperationDecision::into_rmw_message(std::borrow::Cow::Borrowed(&msg.cooperator)).into_owned(),
      cancellable: msg.cancellable,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      uuid: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.uuid),
      autonomous: super::msg::CooperationDecision::from_rmw_message(msg.autonomous),
      cooperator: super::msg::CooperationDecision::from_rmw_message(msg.cooperator),
      cancellable: msg.cancellable,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__RtiState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RtiState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub request: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for RtiState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RtiState::default())
  }
}

impl rosidl_runtime_rs::Message for RtiState {
  type RmwMsg = super::msg::rmw::RtiState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        request: msg.request,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      request: msg.request,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      request: msg.request,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__MrmState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MrmState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MrmState::default())
  }
}

impl rosidl_runtime_rs::Message for MrmState {
  type RmwMsg = super::msg::rmw::MrmState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        state: msg.state,
        behavior: msg.behavior,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      state: msg.state,
      behavior: msg.behavior,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      state: msg.state,
      behavior: msg.behavior,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__MrmDescription

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MrmDescription {

    // This member is not documented.
    #[allow(missing_docs)]
    pub behavior: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub name: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub description: std::string::String,

}



impl Default for MrmDescription {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MrmDescription::default())
  }
}

impl rosidl_runtime_rs::Message for MrmDescription {
  type RmwMsg = super::msg::rmw::MrmDescription;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        behavior: msg.behavior,
        name: msg.name.as_str().into(),
        description: msg.description.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      behavior: msg.behavior,
        name: msg.name.as_str().into(),
        description: msg.description.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      behavior: msg.behavior,
      name: msg.name.to_string(),
      description: msg.description.to_string(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__MrmRequest

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MrmRequest {
    /// The identifier of the request sender.
    pub sender: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MrmRequest::default())
  }
}

impl rosidl_runtime_rs::Message for MrmRequest {
  type RmwMsg = super::msg::rmw::MrmRequest;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        sender: msg.sender.as_str().into(),
        strategy: msg.strategy,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        sender: msg.sender.as_str().into(),
      strategy: msg.strategy,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      sender: msg.sender.to_string(),
      strategy: msg.strategy,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__MrmRequestList

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MrmRequestList {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub requests: Vec<super::msg::MrmRequest>,

}



impl Default for MrmRequestList {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MrmRequestList::default())
  }
}

impl rosidl_runtime_rs::Message for MrmRequestList {
  type RmwMsg = super::msg::rmw::MrmRequestList;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        requests: msg.requests
          .into_iter()
          .map(|elem| super::msg::MrmRequest::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        requests: msg.requests
          .iter()
          .map(|elem| super::msg::MrmRequest::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      requests: msg.requests
          .into_iter()
          .map(super::msg::MrmRequest::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__Heartbeat
/// Timestamp in Autoware for delay checking.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Heartbeat {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

    /// Sequence number for order verification, wraps at 65535.
    pub seq: u16,

}



impl Default for Heartbeat {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Heartbeat::default())
  }
}

impl rosidl_runtime_rs::Message for Heartbeat {
  type RmwMsg = super::msg::rmw::Heartbeat;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        seq: msg.seq,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      seq: msg.seq,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      seq: msg.seq,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DiagGraphStruct

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagGraphStruct {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub nodes: Vec<super::msg::DiagNodeStruct>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub diags: Vec<super::msg::DiagLeafStruct>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub links: Vec<super::msg::DiagLinkStruct>,

}



impl Default for DiagGraphStruct {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DiagGraphStruct::default())
  }
}

impl rosidl_runtime_rs::Message for DiagGraphStruct {
  type RmwMsg = super::msg::rmw::DiagGraphStruct;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        id: msg.id.as_str().into(),
        nodes: msg.nodes
          .into_iter()
          .map(|elem| super::msg::DiagNodeStruct::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        diags: msg.diags
          .into_iter()
          .map(|elem| super::msg::DiagLeafStruct::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        links: msg.links
          .into_iter()
          .map(|elem| super::msg::DiagLinkStruct::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        id: msg.id.as_str().into(),
        nodes: msg.nodes
          .iter()
          .map(|elem| super::msg::DiagNodeStruct::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        diags: msg.diags
          .iter()
          .map(|elem| super::msg::DiagLeafStruct::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        links: msg.links
          .iter()
          .map(|elem| super::msg::DiagLinkStruct::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      id: msg.id.to_string(),
      nodes: msg.nodes
          .into_iter()
          .map(super::msg::DiagNodeStruct::from_rmw_message)
          .collect(),
      diags: msg.diags
          .into_iter()
          .map(super::msg::DiagLeafStruct::from_rmw_message)
          .collect(),
      links: msg.links
          .into_iter()
          .map(super::msg::DiagLinkStruct::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DiagGraphStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagGraphStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub nodes: Vec<super::msg::DiagNodeStatus>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub diags: Vec<super::msg::DiagLeafStatus>,

}



impl Default for DiagGraphStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DiagGraphStatus::default())
  }
}

impl rosidl_runtime_rs::Message for DiagGraphStatus {
  type RmwMsg = super::msg::rmw::DiagGraphStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        id: msg.id.as_str().into(),
        nodes: msg.nodes
          .into_iter()
          .map(|elem| super::msg::DiagNodeStatus::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        diags: msg.diags
          .into_iter()
          .map(|elem| super::msg::DiagLeafStatus::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        id: msg.id.as_str().into(),
        nodes: msg.nodes
          .iter()
          .map(|elem| super::msg::DiagNodeStatus::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        diags: msg.diags
          .iter()
          .map(|elem| super::msg::DiagLeafStatus::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      id: msg.id.to_string(),
      nodes: msg.nodes
          .into_iter()
          .map(super::msg::DiagNodeStatus::from_rmw_message)
          .collect(),
      diags: msg.diags
          .into_iter()
          .map(super::msg::DiagLeafStatus::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DiagLeafStruct

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagLeafStruct {

    // This member is not documented.
    #[allow(missing_docs)]
    pub parent: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub name: std::string::String,

}



impl Default for DiagLeafStruct {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DiagLeafStruct::default())
  }
}

impl rosidl_runtime_rs::Message for DiagLeafStruct {
  type RmwMsg = super::msg::rmw::DiagLeafStruct;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        parent: msg.parent,
        name: msg.name.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      parent: msg.parent,
        name: msg.name.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      parent: msg.parent,
      name: msg.name.to_string(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DiagLeafStatus
/// The level of diagnostic_msgs/msg/DiagnosticStatus.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub message: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub hardware_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub values: Vec<super::msg::KvString>,

}



impl Default for DiagLeafStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DiagLeafStatus::default())
  }
}

impl rosidl_runtime_rs::Message for DiagLeafStatus {
  type RmwMsg = super::msg::rmw::DiagLeafStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        level: msg.level,
        input_level: msg.input_level,
        message: msg.message.as_str().into(),
        hardware_id: msg.hardware_id.as_str().into(),
        values: msg.values
          .into_iter()
          .map(|elem| super::msg::KvString::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      level: msg.level,
      input_level: msg.input_level,
        message: msg.message.as_str().into(),
        hardware_id: msg.hardware_id.as_str().into(),
        values: msg.values
          .iter()
          .map(|elem| super::msg::KvString::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      level: msg.level,
      input_level: msg.input_level,
      message: msg.message.to_string(),
      hardware_id: msg.hardware_id.to_string(),
      values: msg.values
          .into_iter()
          .map(super::msg::KvString::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DiagNodeStruct

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagNodeStruct {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: std::string::String,

}



impl Default for DiagNodeStruct {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DiagNodeStruct::default())
  }
}

impl rosidl_runtime_rs::Message for DiagNodeStruct {
  type RmwMsg = super::msg::rmw::DiagNodeStruct;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: msg.path.to_string(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DiagNodeStatus
/// The level of diagnostic_msgs/msg/DiagnosticStatus.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DiagNodeStatus::default())
  }
}

impl rosidl_runtime_rs::Message for DiagNodeStatus {
  type RmwMsg = super::msg::rmw::DiagNodeStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        level: msg.level,
        input_level: msg.input_level,
        latch_level: msg.latch_level,
        is_dependent: msg.is_dependent,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      level: msg.level,
      input_level: msg.input_level,
      latch_level: msg.latch_level,
      is_dependent: msg.is_dependent,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      level: msg.level,
      input_level: msg.input_level,
      latch_level: msg.latch_level,
      is_dependent: msg.is_dependent,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DiagLinkStruct
/// The index of nodes in the graph struct message.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DiagLinkStruct::default())
  }
}

impl rosidl_runtime_rs::Message for DiagLinkStruct {
  type RmwMsg = super::msg::rmw::DiagLinkStruct;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        parent: msg.parent,
        child: msg.child,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      parent: msg.parent,
      child: msg.child,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      parent: msg.parent,
      child: msg.child,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DoorCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DoorCommand::default())
  }
}

impl rosidl_runtime_rs::Message for DoorCommand {
  type RmwMsg = super::msg::rmw::DoorCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        index: msg.index,
        command: msg.command,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      index: msg.index,
      command: msg.command,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      index: msg.index,
      command: msg.command,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DoorLayout

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DoorLayout {

    // This member is not documented.
    #[allow(missing_docs)]
    pub roles: Vec<u8>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub description: std::string::String,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DoorLayout::default())
  }
}

impl rosidl_runtime_rs::Message for DoorLayout {
  type RmwMsg = super::msg::rmw::DoorLayout;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        roles: msg.roles.into(),
        description: msg.description.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        roles: msg.roles.as_slice().into(),
        description: msg.description.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      roles: msg.roles
          .into_iter()
          .collect(),
      description: msg.description.to_string(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DoorStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DoorStatus::default())
  }
}

impl rosidl_runtime_rs::Message for DoorStatus {
  type RmwMsg = super::msg::rmw::DoorStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__DoorStatusArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DoorStatusArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub doors: Vec<super::msg::DoorStatus>,

}



impl Default for DoorStatusArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DoorStatusArray::default())
  }
}

impl rosidl_runtime_rs::Message for DoorStatusArray {
  type RmwMsg = super::msg::rmw::DoorStatusArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        doors: msg.doors
          .into_iter()
          .map(|elem| super::msg::DoorStatus::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        doors: msg.doors
          .iter()
          .map(|elem| super::msg::DoorStatus::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      doors: msg.doors
          .into_iter()
          .map(super::msg::DoorStatus::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__Gear
/// constants

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Gear::default())
  }
}

impl rosidl_runtime_rs::Message for Gear {
  type RmwMsg = super::msg::rmw::Gear;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__HazardLights
/// constants

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::HazardLights::default())
  }
}

impl rosidl_runtime_rs::Message for HazardLights {
  type RmwMsg = super::msg::rmw::HazardLights;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__TurnIndicators
/// constants

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TurnIndicators::default())
  }
}

impl rosidl_runtime_rs::Message for TurnIndicators {
  type RmwMsg = super::msg::rmw::TurnIndicators;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__VehicleMetrics

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleMetrics {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

    /// The remaining vehicle fuel or battery. Ratio with the maximum as 1.0.
    pub energy: f32,

}



impl Default for VehicleMetrics {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VehicleMetrics::default())
  }
}

impl rosidl_runtime_rs::Message for VehicleMetrics {
  type RmwMsg = super::msg::rmw::VehicleMetrics;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        energy: msg.energy,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      energy: msg.energy,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      energy: msg.energy,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__VehicleStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub gear: super::msg::Gear,


    // This member is not documented.
    #[allow(missing_docs)]
    pub turn_indicators: super::msg::TurnIndicators,


    // This member is not documented.
    #[allow(missing_docs)]
    pub hazard_lights: super::msg::HazardLights,


    // This member is not documented.
    #[allow(missing_docs)]
    pub steering_tire_angle: f64,

}



impl Default for VehicleStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VehicleStatus::default())
  }
}

impl rosidl_runtime_rs::Message for VehicleStatus {
  type RmwMsg = super::msg::rmw::VehicleStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        gear: super::msg::Gear::into_rmw_message(std::borrow::Cow::Owned(msg.gear)).into_owned(),
        turn_indicators: super::msg::TurnIndicators::into_rmw_message(std::borrow::Cow::Owned(msg.turn_indicators)).into_owned(),
        hazard_lights: super::msg::HazardLights::into_rmw_message(std::borrow::Cow::Owned(msg.hazard_lights)).into_owned(),
        steering_tire_angle: msg.steering_tire_angle,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        gear: super::msg::Gear::into_rmw_message(std::borrow::Cow::Borrowed(&msg.gear)).into_owned(),
        turn_indicators: super::msg::TurnIndicators::into_rmw_message(std::borrow::Cow::Borrowed(&msg.turn_indicators)).into_owned(),
        hazard_lights: super::msg::HazardLights::into_rmw_message(std::borrow::Cow::Borrowed(&msg.hazard_lights)).into_owned(),
      steering_tire_angle: msg.steering_tire_angle,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      gear: super::msg::Gear::from_rmw_message(msg.gear),
      turn_indicators: super::msg::TurnIndicators::from_rmw_message(msg.turn_indicators),
      hazard_lights: super::msg::HazardLights::from_rmw_message(msg.hazard_lights),
      steering_tire_angle: msg.steering_tire_angle,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__VehicleDimensions

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub footprint: geometry_msgs::msg::Polygon,

}



impl Default for VehicleDimensions {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VehicleDimensions::default())
  }
}

impl rosidl_runtime_rs::Message for VehicleDimensions {
  type RmwMsg = super::msg::rmw::VehicleDimensions;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        wheel_radius: msg.wheel_radius,
        wheel_width: msg.wheel_width,
        wheel_base: msg.wheel_base,
        wheel_tread: msg.wheel_tread,
        front_overhang: msg.front_overhang,
        rear_overhang: msg.rear_overhang,
        left_overhang: msg.left_overhang,
        right_overhang: msg.right_overhang,
        height: msg.height,
        footprint: geometry_msgs::msg::Polygon::into_rmw_message(std::borrow::Cow::Owned(msg.footprint)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      wheel_radius: msg.wheel_radius,
      wheel_width: msg.wheel_width,
      wheel_base: msg.wheel_base,
      wheel_tread: msg.wheel_tread,
      front_overhang: msg.front_overhang,
      rear_overhang: msg.rear_overhang,
      left_overhang: msg.left_overhang,
      right_overhang: msg.right_overhang,
      height: msg.height,
        footprint: geometry_msgs::msg::Polygon::into_rmw_message(std::borrow::Cow::Borrowed(&msg.footprint)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      wheel_radius: msg.wheel_radius,
      wheel_width: msg.wheel_width,
      wheel_base: msg.wheel_base,
      wheel_tread: msg.wheel_tread,
      front_overhang: msg.front_overhang,
      rear_overhang: msg.rear_overhang,
      left_overhang: msg.left_overhang,
      right_overhang: msg.right_overhang,
      height: msg.height,
      footprint: geometry_msgs::msg::Polygon::from_rmw_message(msg.footprint),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__VehicleSpecs

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleSpecs {

    // This member is not documented.
    #[allow(missing_docs)]
    pub max_steering_tire_angle: f32,

}



impl Default for VehicleSpecs {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VehicleSpecs::default())
  }
}

impl rosidl_runtime_rs::Message for VehicleSpecs {
  type RmwMsg = super::msg::rmw::VehicleSpecs;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        max_steering_tire_angle: msg.max_steering_tire_angle,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      max_steering_tire_angle: msg.max_steering_tire_angle,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      max_steering_tire_angle: msg.max_steering_tire_angle,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__msg__VehicleKinematics
/// Geographic point, using the WGS 84 reference ellipsoid.
/// This data will be invalid If Autoware does not provide projection information between geographic coordinates and local coordinates.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleKinematics {

    // This member is not documented.
    #[allow(missing_docs)]
    pub geographic_pose: geographic_msgs::msg::GeoPointStamped,

    /// Local coordinate from the autoware
    pub pose: geometry_msgs::msg::PoseWithCovarianceStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub twist: geometry_msgs::msg::TwistWithCovarianceStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub accel: geometry_msgs::msg::AccelWithCovarianceStamped,

}



impl Default for VehicleKinematics {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VehicleKinematics::default())
  }
}

impl rosidl_runtime_rs::Message for VehicleKinematics {
  type RmwMsg = super::msg::rmw::VehicleKinematics;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        geographic_pose: geographic_msgs::msg::GeoPointStamped::into_rmw_message(std::borrow::Cow::Owned(msg.geographic_pose)).into_owned(),
        pose: geometry_msgs::msg::PoseWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        twist: geometry_msgs::msg::TwistWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Owned(msg.twist)).into_owned(),
        accel: geometry_msgs::msg::AccelWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Owned(msg.accel)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        geographic_pose: geographic_msgs::msg::GeoPointStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.geographic_pose)).into_owned(),
        pose: geometry_msgs::msg::PoseWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
        twist: geometry_msgs::msg::TwistWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.twist)).into_owned(),
        accel: geometry_msgs::msg::AccelWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.accel)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      geographic_pose: geographic_msgs::msg::GeoPointStamped::from_rmw_message(msg.geographic_pose),
      pose: geometry_msgs::msg::PoseWithCovarianceStamped::from_rmw_message(msg.pose),
      twist: geometry_msgs::msg::TwistWithCovarianceStamped::from_rmw_message(msg.twist),
      accel: geometry_msgs::msg::AccelWithCovarianceStamped::from_rmw_message(msg.accel),
    }
  }
}


