#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to explore_lite_msgs__msg__ExploreStatus
/// Exploration status constants
/// EXPLORATION_STARTED: Exploration has begun (published on initialization)
/// EXPLORATION_IN_PROGRESS: Exploration resumed after pause
/// EXPLORATION_PAUSED: Exploration manually stopped via /explore/resume topic
/// EXPLORATION_COMPLETE: No frontiers remaining, exploration finished
/// RETURNING_TO_ORIGIN: Robot navigating back to initial pose (if return_to_init: true)
/// RETURNED_TO_ORIGIN: Robot successfully reached initial pose

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExploreStatus {
    /// Current exploration status
    pub status: std::string::String,

}

impl ExploreStatus {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const EXPLORATION_STARTED: &'static str = "exploration_started";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const EXPLORATION_IN_PROGRESS: &'static str = "exploration_in_progress";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const EXPLORATION_PAUSED: &'static str = "exploration_paused";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const EXPLORATION_COMPLETE: &'static str = "exploration_complete";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RETURNING_TO_ORIGIN: &'static str = "returning_to_origin";


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RETURNED_TO_ORIGIN: &'static str = "returned_to_origin";

}


impl Default for ExploreStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ExploreStatus::default())
  }
}

impl rosidl_runtime_rs::Message for ExploreStatus {
  type RmwMsg = super::msg::rmw::ExploreStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status.to_string(),
    }
  }
}


