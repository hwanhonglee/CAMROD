#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to autoware_adapi_v1_msgs__srv__InitializeLocalization_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct InitializeLocalization_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: rosidl_runtime_rs::BoundedSequence<geometry_msgs::msg::rmw::PoseWithCovarianceStamped, 1>,

}



impl Default for InitializeLocalization_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::InitializeLocalization_Request::default())
  }
}

impl rosidl_runtime_rs::Message for InitializeLocalization_Request {
  type RmwMsg = super::srv::rmw::InitializeLocalization_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: msg.pose,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: msg.pose.clone(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: msg.pose,
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__InitializeLocalization_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct InitializeLocalization_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::InitializeLocalization_Response::default())
  }
}

impl rosidl_runtime_rs::Message for InitializeLocalization_Response {
  type RmwMsg = super::srv::rmw::InitializeLocalization_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__ListManualControlMode_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ListManualControlMode_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ListManualControlMode_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ListManualControlMode_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ListManualControlMode_Request {
  type RmwMsg = super::srv::rmw::ListManualControlMode_Request;

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


// Corresponds to autoware_adapi_v1_msgs__srv__ListManualControlMode_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ListManualControlMode_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,


    // This member is not documented.
    #[allow(missing_docs)]
    pub modes: Vec<super::msg::ManualControlMode>,

}



impl Default for ListManualControlMode_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ListManualControlMode_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ListManualControlMode_Response {
  type RmwMsg = super::srv::rmw::ListManualControlMode_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
        modes: msg.modes
          .into_iter()
          .map(|elem| super::msg::ManualControlMode::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
        modes: msg.modes
          .iter()
          .map(|elem| super::msg::ManualControlMode::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
      modes: msg.modes
          .into_iter()
          .map(super::msg::ManualControlMode::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SelectManualControlMode_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SelectManualControlMode_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: super::msg::ManualControlMode,

}



impl Default for SelectManualControlMode_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SelectManualControlMode_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SelectManualControlMode_Request {
  type RmwMsg = super::srv::rmw::SelectManualControlMode_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        mode: super::msg::ManualControlMode::into_rmw_message(std::borrow::Cow::Owned(msg.mode)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        mode: super::msg::ManualControlMode::into_rmw_message(std::borrow::Cow::Borrowed(&msg.mode)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      mode: super::msg::ManualControlMode::from_rmw_message(msg.mode),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SelectManualControlMode_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SelectManualControlMode_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

}



impl Default for SelectManualControlMode_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SelectManualControlMode_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SelectManualControlMode_Response {
  type RmwMsg = super::srv::rmw::SelectManualControlMode_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__ClearRoute_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ClearRoute_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ClearRoute_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ClearRoute_Request {
  type RmwMsg = super::srv::rmw::ClearRoute_Request;

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


// Corresponds to autoware_adapi_v1_msgs__srv__ClearRoute_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

}



impl Default for ClearRoute_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ClearRoute_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ClearRoute_Response {
  type RmwMsg = super::srv::rmw::ClearRoute_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SetRoute_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub option: super::msg::RouteOption,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub segments: Vec<super::msg::RouteSegment>,

}



impl Default for SetRoute_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetRoute_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetRoute_Request {
  type RmwMsg = super::srv::rmw::SetRoute_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        option: super::msg::RouteOption::into_rmw_message(std::borrow::Cow::Owned(msg.option)).into_owned(),
        goal: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
        segments: msg.segments
          .into_iter()
          .map(|elem| super::msg::RouteSegment::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        option: super::msg::RouteOption::into_rmw_message(std::borrow::Cow::Borrowed(&msg.option)).into_owned(),
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
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      option: super::msg::RouteOption::from_rmw_message(msg.option),
      goal: geometry_msgs::msg::Pose::from_rmw_message(msg.goal),
      segments: msg.segments
          .into_iter()
          .map(super::msg::RouteSegment::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SetRoute_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetRoute_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetRoute_Response {
  type RmwMsg = super::srv::rmw::SetRoute_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SetRoutePoints_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetRoutePoints_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub option: super::msg::RouteOption,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub waypoints: Vec<geometry_msgs::msg::Pose>,

}



impl Default for SetRoutePoints_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetRoutePoints_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetRoutePoints_Request {
  type RmwMsg = super::srv::rmw::SetRoutePoints_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        option: super::msg::RouteOption::into_rmw_message(std::borrow::Cow::Owned(msg.option)).into_owned(),
        goal: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
        waypoints: msg.waypoints
          .into_iter()
          .map(|elem| geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        option: super::msg::RouteOption::into_rmw_message(std::borrow::Cow::Borrowed(&msg.option)).into_owned(),
        goal: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
        waypoints: msg.waypoints
          .iter()
          .map(|elem| geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      option: super::msg::RouteOption::from_rmw_message(msg.option),
      goal: geometry_msgs::msg::Pose::from_rmw_message(msg.goal),
      waypoints: msg.waypoints
          .into_iter()
          .map(geometry_msgs::msg::Pose::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SetRoutePoints_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetRoutePoints_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetRoutePoints_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetRoutePoints_Response {
  type RmwMsg = super::srv::rmw::SetRoutePoints_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__ChangeOperationMode_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChangeOperationMode_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ChangeOperationMode_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ChangeOperationMode_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ChangeOperationMode_Request {
  type RmwMsg = super::srv::rmw::ChangeOperationMode_Request;

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


// Corresponds to autoware_adapi_v1_msgs__srv__ChangeOperationMode_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChangeOperationMode_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ChangeOperationMode_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ChangeOperationMode_Response {
  type RmwMsg = super::srv::rmw::ChangeOperationMode_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__AcceptStart_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AcceptStart_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for AcceptStart_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::AcceptStart_Request::default())
  }
}

impl rosidl_runtime_rs::Message for AcceptStart_Request {
  type RmwMsg = super::srv::rmw::AcceptStart_Request;

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


// Corresponds to autoware_adapi_v1_msgs__srv__AcceptStart_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AcceptStart_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

}

impl AcceptStart_Response {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ERROR_NOT_STARTING: u16 = 1;

}


impl Default for AcceptStart_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::AcceptStart_Response::default())
  }
}

impl rosidl_runtime_rs::Message for AcceptStart_Response {
  type RmwMsg = super::srv::rmw::AcceptStart_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SetCooperationCommands_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCooperationCommands_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub commands: Vec<super::msg::CooperationCommand>,

}



impl Default for SetCooperationCommands_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetCooperationCommands_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetCooperationCommands_Request {
  type RmwMsg = super::srv::rmw::SetCooperationCommands_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        commands: msg.commands
          .into_iter()
          .map(|elem| super::msg::CooperationCommand::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        commands: msg.commands
          .iter()
          .map(|elem| super::msg::CooperationCommand::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      commands: msg.commands
          .into_iter()
          .map(super::msg::CooperationCommand::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SetCooperationCommands_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCooperationCommands_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

}



impl Default for SetCooperationCommands_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetCooperationCommands_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetCooperationCommands_Response {
  type RmwMsg = super::srv::rmw::SetCooperationCommands_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCooperationPolicies_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub policies: Vec<super::msg::CooperationPolicy>,

}



impl Default for SetCooperationPolicies_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetCooperationPolicies_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetCooperationPolicies_Request {
  type RmwMsg = super::srv::rmw::SetCooperationPolicies_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        policies: msg.policies
          .into_iter()
          .map(|elem| super::msg::CooperationPolicy::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        policies: msg.policies
          .iter()
          .map(|elem| super::msg::CooperationPolicy::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      policies: msg.policies
          .into_iter()
          .map(super::msg::CooperationPolicy::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SetCooperationPolicies_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCooperationPolicies_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

}



impl Default for SetCooperationPolicies_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetCooperationPolicies_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetCooperationPolicies_Response {
  type RmwMsg = super::srv::rmw::SetCooperationPolicies_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCooperationPolicies_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetCooperationPolicies_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetCooperationPolicies_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetCooperationPolicies_Request {
  type RmwMsg = super::srv::rmw::GetCooperationPolicies_Request;

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


// Corresponds to autoware_adapi_v1_msgs__srv__GetCooperationPolicies_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCooperationPolicies_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,


    // This member is not documented.
    #[allow(missing_docs)]
    pub policies: Vec<super::msg::CooperationPolicy>,

}



impl Default for GetCooperationPolicies_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetCooperationPolicies_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetCooperationPolicies_Response {
  type RmwMsg = super::srv::rmw::GetCooperationPolicies_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
        policies: msg.policies
          .into_iter()
          .map(|elem| super::msg::CooperationPolicy::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
        policies: msg.policies
          .iter()
          .map(|elem| super::msg::CooperationPolicy::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
      policies: msg.policies
          .into_iter()
          .map(super::msg::CooperationPolicy::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SendMrmRequest_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SendMrmRequest_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub request: super::msg::MrmRequest,

}



impl Default for SendMrmRequest_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SendMrmRequest_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SendMrmRequest_Request {
  type RmwMsg = super::srv::rmw::SendMrmRequest_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        request: super::msg::MrmRequest::into_rmw_message(std::borrow::Cow::Owned(msg.request)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        request: super::msg::MrmRequest::into_rmw_message(std::borrow::Cow::Borrowed(&msg.request)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      request: super::msg::MrmRequest::from_rmw_message(msg.request),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SendMrmRequest_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SendMrmRequest_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

}



impl Default for SendMrmRequest_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SendMrmRequest_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SendMrmRequest_Response {
  type RmwMsg = super::srv::rmw::SendMrmRequest_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__ListMrmDescription_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ListMrmDescription_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ListMrmDescription_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ListMrmDescription_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ListMrmDescription_Request {
  type RmwMsg = super::srv::rmw::ListMrmDescription_Request;

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


// Corresponds to autoware_adapi_v1_msgs__srv__ListMrmDescription_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ListMrmDescription_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub descriptions: Vec<super::msg::MrmDescription>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

}



impl Default for ListMrmDescription_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ListMrmDescription_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ListMrmDescription_Response {
  type RmwMsg = super::srv::rmw::ListMrmDescription_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        descriptions: msg.descriptions
          .into_iter()
          .map(|elem| super::msg::MrmDescription::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        descriptions: msg.descriptions
          .iter()
          .map(|elem| super::msg::MrmDescription::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      descriptions: msg.descriptions
          .into_iter()
          .map(super::msg::MrmDescription::from_rmw_message)
          .collect(),
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__ResetDiagGraph_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ResetDiagGraph_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ResetDiagGraph_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ResetDiagGraph_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ResetDiagGraph_Request {
  type RmwMsg = super::srv::rmw::ResetDiagGraph_Request;

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


// Corresponds to autoware_adapi_v1_msgs__srv__ResetDiagGraph_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ResetDiagGraph_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

}



impl Default for ResetDiagGraph_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ResetDiagGraph_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ResetDiagGraph_Response {
  type RmwMsg = super::srv::rmw::ResetDiagGraph_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SetDoorCommand_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetDoorCommand_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub doors: Vec<super::msg::DoorCommand>,

}



impl Default for SetDoorCommand_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetDoorCommand_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetDoorCommand_Request {
  type RmwMsg = super::srv::rmw::SetDoorCommand_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        doors: msg.doors
          .into_iter()
          .map(|elem| super::msg::DoorCommand::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        doors: msg.doors
          .iter()
          .map(|elem| super::msg::DoorCommand::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      doors: msg.doors
          .into_iter()
          .map(super::msg::DoorCommand::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__SetDoorCommand_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetDoorCommand_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,

}



impl Default for SetDoorCommand_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetDoorCommand_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetDoorCommand_Response {
  type RmwMsg = super::srv::rmw::SetDoorCommand_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__GetDoorLayout_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetDoorLayout_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetDoorLayout_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetDoorLayout_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetDoorLayout_Request {
  type RmwMsg = super::srv::rmw::GetDoorLayout_Request;

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


// Corresponds to autoware_adapi_v1_msgs__srv__GetDoorLayout_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetDoorLayout_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,


    // This member is not documented.
    #[allow(missing_docs)]
    pub doors: Vec<super::msg::DoorLayout>,

}



impl Default for GetDoorLayout_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetDoorLayout_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetDoorLayout_Response {
  type RmwMsg = super::srv::rmw::GetDoorLayout_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
        doors: msg.doors
          .into_iter()
          .map(|elem| super::msg::DoorLayout::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
        doors: msg.doors
          .iter()
          .map(|elem| super::msg::DoorLayout::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
      doors: msg.doors
          .into_iter()
          .map(super::msg::DoorLayout::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetVehicleDimensions_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetVehicleDimensions_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetVehicleDimensions_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetVehicleDimensions_Request {
  type RmwMsg = super::srv::rmw::GetVehicleDimensions_Request;

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


// Corresponds to autoware_adapi_v1_msgs__srv__GetVehicleDimensions_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetVehicleDimensions_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dimensions: super::msg::VehicleDimensions,

}



impl Default for GetVehicleDimensions_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetVehicleDimensions_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetVehicleDimensions_Response {
  type RmwMsg = super::srv::rmw::GetVehicleDimensions_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
        dimensions: super::msg::VehicleDimensions::into_rmw_message(std::borrow::Cow::Owned(msg.dimensions)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
        dimensions: super::msg::VehicleDimensions::into_rmw_message(std::borrow::Cow::Borrowed(&msg.dimensions)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
      dimensions: super::msg::VehicleDimensions::from_rmw_message(msg.dimensions),
    }
  }
}


// Corresponds to autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetVehicleSpecs_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetVehicleSpecs_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetVehicleSpecs_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetVehicleSpecs_Request {
  type RmwMsg = super::srv::rmw::GetVehicleSpecs_Request;

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


// Corresponds to autoware_adapi_v1_msgs__srv__GetVehicleSpecs_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetVehicleSpecs_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: super::msg::ResponseStatus,


    // This member is not documented.
    #[allow(missing_docs)]
    pub specs: super::msg::VehicleSpecs,

}



impl Default for GetVehicleSpecs_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetVehicleSpecs_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetVehicleSpecs_Response {
  type RmwMsg = super::srv::rmw::GetVehicleSpecs_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
        specs: super::msg::VehicleSpecs::into_rmw_message(std::borrow::Cow::Owned(msg.specs)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: super::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
        specs: super::msg::VehicleSpecs::into_rmw_message(std::borrow::Cow::Borrowed(&msg.specs)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: super::msg::ResponseStatus::from_rmw_message(msg.status),
      specs: super::msg::VehicleSpecs::from_rmw_message(msg.specs),
    }
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


