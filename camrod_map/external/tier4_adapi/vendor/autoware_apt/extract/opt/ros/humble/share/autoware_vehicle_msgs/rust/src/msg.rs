#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to autoware_vehicle_msgs__msg__ControlModeReport

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ControlModeReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ControlModeReport::default())
  }
}

impl rosidl_runtime_rs::Message for ControlModeReport {
  type RmwMsg = super::msg::rmw::ControlModeReport;

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


// Corresponds to autoware_vehicle_msgs__msg__Engage

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Engage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub engage: bool,

}



impl Default for Engage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Engage::default())
  }
}

impl rosidl_runtime_rs::Message for Engage {
  type RmwMsg = super::msg::rmw::Engage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        engage: msg.engage,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      engage: msg.engage,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      engage: msg.engage,
    }
  }
}


// Corresponds to autoware_vehicle_msgs__msg__GearCommand

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::GearCommand::default())
  }
}

impl rosidl_runtime_rs::Message for GearCommand {
  type RmwMsg = super::msg::rmw::GearCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        command: msg.command,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      command: msg.command,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      command: msg.command,
    }
  }
}


// Corresponds to autoware_vehicle_msgs__msg__GearReport

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GearReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::GearReport::default())
  }
}

impl rosidl_runtime_rs::Message for GearReport {
  type RmwMsg = super::msg::rmw::GearReport;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        report: msg.report,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      report: msg.report,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      report: msg.report,
    }
  }
}


// Corresponds to autoware_vehicle_msgs__msg__HazardLightsCommand

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::HazardLightsCommand::default())
  }
}

impl rosidl_runtime_rs::Message for HazardLightsCommand {
  type RmwMsg = super::msg::rmw::HazardLightsCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        command: msg.command,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      command: msg.command,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      command: msg.command,
    }
  }
}


// Corresponds to autoware_vehicle_msgs__msg__HazardLightsReport

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HazardLightsReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::HazardLightsReport::default())
  }
}

impl rosidl_runtime_rs::Message for HazardLightsReport {
  type RmwMsg = super::msg::rmw::HazardLightsReport;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        report: msg.report,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      report: msg.report,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      report: msg.report,
    }
  }
}


// Corresponds to autoware_vehicle_msgs__msg__SteeringReport

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SteeringReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub steering_tire_angle: f32,

}



impl Default for SteeringReport {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SteeringReport::default())
  }
}

impl rosidl_runtime_rs::Message for SteeringReport {
  type RmwMsg = super::msg::rmw::SteeringReport;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        steering_tire_angle: msg.steering_tire_angle,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      steering_tire_angle: msg.steering_tire_angle,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      steering_tire_angle: msg.steering_tire_angle,
    }
  }
}


// Corresponds to autoware_vehicle_msgs__msg__TurnIndicatorsCommand

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TurnIndicatorsCommand::default())
  }
}

impl rosidl_runtime_rs::Message for TurnIndicatorsCommand {
  type RmwMsg = super::msg::rmw::TurnIndicatorsCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        command: msg.command,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      command: msg.command,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      command: msg.command,
    }
  }
}


// Corresponds to autoware_vehicle_msgs__msg__TurnIndicatorsReport

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TurnIndicatorsReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TurnIndicatorsReport::default())
  }
}

impl rosidl_runtime_rs::Message for TurnIndicatorsReport {
  type RmwMsg = super::msg::rmw::TurnIndicatorsReport;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        report: msg.report,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      report: msg.report,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      report: msg.report,
    }
  }
}


// Corresponds to autoware_vehicle_msgs__msg__VelocityReport

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelocityReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VelocityReport::default())
  }
}

impl rosidl_runtime_rs::Message for VelocityReport {
  type RmwMsg = super::msg::rmw::VelocityReport;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        longitudinal_velocity: msg.longitudinal_velocity,
        lateral_velocity: msg.lateral_velocity,
        heading_rate: msg.heading_rate,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      longitudinal_velocity: msg.longitudinal_velocity,
      lateral_velocity: msg.lateral_velocity,
      heading_rate: msg.heading_rate,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      longitudinal_velocity: msg.longitudinal_velocity,
      lateral_velocity: msg.lateral_velocity,
      heading_rate: msg.heading_rate,
    }
  }
}


