#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to autoware_common_msgs__msg__ResponseStatus
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


