#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to autoware_planning_msgs__msg__LaneletPrimitive

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaneletPrimitive {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: i64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub primitive_type: std::string::String,

}



impl Default for LaneletPrimitive {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LaneletPrimitive::default())
  }
}

impl rosidl_runtime_rs::Message for LaneletPrimitive {
  type RmwMsg = super::msg::rmw::LaneletPrimitive;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        primitive_type: msg.primitive_type.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
        primitive_type: msg.primitive_type.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      primitive_type: msg.primitive_type.to_string(),
    }
  }
}


// Corresponds to autoware_planning_msgs__msg__LaneletRoute

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaneletRoute {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start_pose: geometry_msgs::msg::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_pose: geometry_msgs::msg::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub segments: Vec<super::msg::LaneletSegment>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub uuid: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub allow_modification: bool,

}



impl Default for LaneletRoute {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LaneletRoute::default())
  }
}

impl rosidl_runtime_rs::Message for LaneletRoute {
  type RmwMsg = super::msg::rmw::LaneletRoute;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        start_pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.start_pose)).into_owned(),
        goal_pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.goal_pose)).into_owned(),
        segments: msg.segments
          .into_iter()
          .map(|elem| super::msg::LaneletSegment::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        uuid: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.uuid)).into_owned(),
        allow_modification: msg.allow_modification,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        start_pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start_pose)).into_owned(),
        goal_pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_pose)).into_owned(),
        segments: msg.segments
          .iter()
          .map(|elem| super::msg::LaneletSegment::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        uuid: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.uuid)).into_owned(),
      allow_modification: msg.allow_modification,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      start_pose: geometry_msgs::msg::Pose::from_rmw_message(msg.start_pose),
      goal_pose: geometry_msgs::msg::Pose::from_rmw_message(msg.goal_pose),
      segments: msg.segments
          .into_iter()
          .map(super::msg::LaneletSegment::from_rmw_message)
          .collect(),
      uuid: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.uuid),
      allow_modification: msg.allow_modification,
    }
  }
}


// Corresponds to autoware_planning_msgs__msg__LaneletSegment

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaneletSegment {

    // This member is not documented.
    #[allow(missing_docs)]
    pub preferred_primitive: super::msg::LaneletPrimitive,


    // This member is not documented.
    #[allow(missing_docs)]
    pub primitives: Vec<super::msg::LaneletPrimitive>,

}



impl Default for LaneletSegment {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LaneletSegment::default())
  }
}

impl rosidl_runtime_rs::Message for LaneletSegment {
  type RmwMsg = super::msg::rmw::LaneletSegment;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        preferred_primitive: super::msg::LaneletPrimitive::into_rmw_message(std::borrow::Cow::Owned(msg.preferred_primitive)).into_owned(),
        primitives: msg.primitives
          .into_iter()
          .map(|elem| super::msg::LaneletPrimitive::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        preferred_primitive: super::msg::LaneletPrimitive::into_rmw_message(std::borrow::Cow::Borrowed(&msg.preferred_primitive)).into_owned(),
        primitives: msg.primitives
          .iter()
          .map(|elem| super::msg::LaneletPrimitive::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      preferred_primitive: super::msg::LaneletPrimitive::from_rmw_message(msg.preferred_primitive),
      primitives: msg.primitives
          .into_iter()
          .map(super::msg::LaneletPrimitive::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_planning_msgs__msg__PoseWithUuidStamped

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PoseWithUuidStamped {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub uuid: unique_identifier_msgs::msg::UUID,

}



impl Default for PoseWithUuidStamped {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PoseWithUuidStamped::default())
  }
}

impl rosidl_runtime_rs::Message for PoseWithUuidStamped {
  type RmwMsg = super::msg::rmw::PoseWithUuidStamped;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        uuid: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.uuid)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
        uuid: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.uuid)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
      uuid: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.uuid),
    }
  }
}


// Corresponds to autoware_planning_msgs__msg__Trajectory

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Trajectory {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub points: Vec<super::msg::TrajectoryPoint>,

}



impl Default for Trajectory {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Trajectory::default())
  }
}

impl rosidl_runtime_rs::Message for Trajectory {
  type RmwMsg = super::msg::rmw::Trajectory;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        points: msg.points
          .into_iter()
          .map(|elem| super::msg::TrajectoryPoint::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        points: msg.points
          .iter()
          .map(|elem| super::msg::TrajectoryPoint::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      points: msg.points
          .into_iter()
          .map(super::msg::TrajectoryPoint::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_planning_msgs__msg__TrajectoryPoint

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrajectoryPoint {

    // This member is not documented.
    #[allow(missing_docs)]
    pub time_from_start: builtin_interfaces::msg::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::Pose,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TrajectoryPoint::default())
  }
}

impl rosidl_runtime_rs::Message for TrajectoryPoint {
  type RmwMsg = super::msg::rmw::TrajectoryPoint;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_from_start: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_from_start)).into_owned(),
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        longitudinal_velocity_mps: msg.longitudinal_velocity_mps,
        lateral_velocity_mps: msg.lateral_velocity_mps,
        acceleration_mps2: msg.acceleration_mps2,
        heading_rate_rps: msg.heading_rate_rps,
        front_wheel_angle_rad: msg.front_wheel_angle_rad,
        rear_wheel_angle_rad: msg.rear_wheel_angle_rad,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_from_start: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_from_start)).into_owned(),
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      longitudinal_velocity_mps: msg.longitudinal_velocity_mps,
      lateral_velocity_mps: msg.lateral_velocity_mps,
      acceleration_mps2: msg.acceleration_mps2,
      heading_rate_rps: msg.heading_rate_rps,
      front_wheel_angle_rad: msg.front_wheel_angle_rad,
      rear_wheel_angle_rad: msg.rear_wheel_angle_rad,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      time_from_start: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_from_start),
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
      longitudinal_velocity_mps: msg.longitudinal_velocity_mps,
      lateral_velocity_mps: msg.lateral_velocity_mps,
      acceleration_mps2: msg.acceleration_mps2,
      heading_rate_rps: msg.heading_rate_rps,
      front_wheel_angle_rad: msg.front_wheel_angle_rad,
      rear_wheel_angle_rad: msg.rear_wheel_angle_rad,
    }
  }
}


// Corresponds to autoware_planning_msgs__msg__Path

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Path {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub points: Vec<super::msg::PathPoint>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub left_bound: Vec<geometry_msgs::msg::Point>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_bound: Vec<geometry_msgs::msg::Point>,

}



impl Default for Path {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Path::default())
  }
}

impl rosidl_runtime_rs::Message for Path {
  type RmwMsg = super::msg::rmw::Path;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        points: msg.points
          .into_iter()
          .map(|elem| super::msg::PathPoint::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        left_bound: msg.left_bound
          .into_iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        right_bound: msg.right_bound
          .into_iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        points: msg.points
          .iter()
          .map(|elem| super::msg::PathPoint::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        left_bound: msg.left_bound
          .iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        right_bound: msg.right_bound
          .iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      points: msg.points
          .into_iter()
          .map(super::msg::PathPoint::from_rmw_message)
          .collect(),
      left_bound: msg.left_bound
          .into_iter()
          .map(geometry_msgs::msg::Point::from_rmw_message)
          .collect(),
      right_bound: msg.right_bound
          .into_iter()
          .map(geometry_msgs::msg::Point::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_planning_msgs__msg__PathPoint

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PathPoint {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::Pose,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PathPoint::default())
  }
}

impl rosidl_runtime_rs::Message for PathPoint {
  type RmwMsg = super::msg::rmw::PathPoint;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        longitudinal_velocity_mps: msg.longitudinal_velocity_mps,
        lateral_velocity_mps: msg.lateral_velocity_mps,
        heading_rate_rps: msg.heading_rate_rps,
        is_final: msg.is_final,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      longitudinal_velocity_mps: msg.longitudinal_velocity_mps,
      lateral_velocity_mps: msg.lateral_velocity_mps,
      heading_rate_rps: msg.heading_rate_rps,
      is_final: msg.is_final,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
      longitudinal_velocity_mps: msg.longitudinal_velocity_mps,
      lateral_velocity_mps: msg.lateral_velocity_mps,
      heading_rate_rps: msg.heading_rate_rps,
      is_final: msg.is_final,
    }
  }
}


// Corresponds to autoware_planning_msgs__msg__RouteState

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


