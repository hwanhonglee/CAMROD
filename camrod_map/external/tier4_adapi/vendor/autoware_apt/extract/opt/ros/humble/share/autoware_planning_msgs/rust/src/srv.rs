#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to autoware_planning_msgs__srv__ClearRoute_Request

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


// Corresponds to autoware_planning_msgs__srv__ClearRoute_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: autoware_common_msgs::msg::ResponseStatus,

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
        status: autoware_common_msgs::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: autoware_common_msgs::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: autoware_common_msgs::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_planning_msgs__srv__SetLaneletRoute_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetLaneletRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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



impl Default for SetLaneletRoute_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetLaneletRoute_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetLaneletRoute_Request {
  type RmwMsg = super::srv::rmw::SetLaneletRoute_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
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


// Corresponds to autoware_planning_msgs__srv__SetLaneletRoute_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetLaneletRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: autoware_common_msgs::msg::ResponseStatus,

}



impl Default for SetLaneletRoute_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetLaneletRoute_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetLaneletRoute_Response {
  type RmwMsg = super::srv::rmw::SetLaneletRoute_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: autoware_common_msgs::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: autoware_common_msgs::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: autoware_common_msgs::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_planning_msgs__srv__SetPreferredPrimitive_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPreferredPrimitive_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub preferred_primitives: Vec<super::msg::LaneletPrimitive>,

    /// reset flag for preferred primitives in route
    /// If set to true, this signals to mission_planner that the preferred-primitives have been reverted to those of the original path
    pub reset: bool,

    /// ID of the route that will be modified.
    pub uuid: unique_identifier_msgs::msg::UUID,

}



impl Default for SetPreferredPrimitive_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetPreferredPrimitive_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetPreferredPrimitive_Request {
  type RmwMsg = super::srv::rmw::SetPreferredPrimitive_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        preferred_primitives: msg.preferred_primitives
          .into_iter()
          .map(|elem| super::msg::LaneletPrimitive::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        reset: msg.reset,
        uuid: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.uuid)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        preferred_primitives: msg.preferred_primitives
          .iter()
          .map(|elem| super::msg::LaneletPrimitive::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      reset: msg.reset,
        uuid: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.uuid)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      preferred_primitives: msg.preferred_primitives
          .into_iter()
          .map(super::msg::LaneletPrimitive::from_rmw_message)
          .collect(),
      reset: msg.reset,
      uuid: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.uuid),
    }
  }
}


// Corresponds to autoware_planning_msgs__srv__SetPreferredPrimitive_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPreferredPrimitive_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: autoware_common_msgs::msg::ResponseStatus,

}



impl Default for SetPreferredPrimitive_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetPreferredPrimitive_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetPreferredPrimitive_Response {
  type RmwMsg = super::srv::rmw::SetPreferredPrimitive_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: autoware_common_msgs::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: autoware_common_msgs::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: autoware_common_msgs::msg::ResponseStatus::from_rmw_message(msg.status),
    }
  }
}


// Corresponds to autoware_planning_msgs__srv__SetWaypointRoute_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetWaypointRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_pose: geometry_msgs::msg::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub waypoints: Vec<geometry_msgs::msg::Pose>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub uuid: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub allow_modification: bool,

}



impl Default for SetWaypointRoute_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetWaypointRoute_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetWaypointRoute_Request {
  type RmwMsg = super::srv::rmw::SetWaypointRoute_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        goal_pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.goal_pose)).into_owned(),
        waypoints: msg.waypoints
          .into_iter()
          .map(|elem| geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        uuid: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.uuid)).into_owned(),
        allow_modification: msg.allow_modification,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        goal_pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_pose)).into_owned(),
        waypoints: msg.waypoints
          .iter()
          .map(|elem| geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        uuid: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.uuid)).into_owned(),
      allow_modification: msg.allow_modification,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      goal_pose: geometry_msgs::msg::Pose::from_rmw_message(msg.goal_pose),
      waypoints: msg.waypoints
          .into_iter()
          .map(geometry_msgs::msg::Pose::from_rmw_message)
          .collect(),
      uuid: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.uuid),
      allow_modification: msg.allow_modification,
    }
  }
}


// Corresponds to autoware_planning_msgs__srv__SetWaypointRoute_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetWaypointRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: autoware_common_msgs::msg::ResponseStatus,

}



impl Default for SetWaypointRoute_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetWaypointRoute_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetWaypointRoute_Response {
  type RmwMsg = super::srv::rmw::SetWaypointRoute_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: autoware_common_msgs::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: autoware_common_msgs::msg::ResponseStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: autoware_common_msgs::msg::ResponseStatus::from_rmw_message(msg.status),
    }
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


