#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "explore_lite_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__explore_lite_msgs__msg__ExploreStatus() -> *const std::ffi::c_void;
}

#[link(name = "explore_lite_msgs__rosidl_generator_c")]
extern "C" {
    fn explore_lite_msgs__msg__ExploreStatus__init(msg: *mut ExploreStatus) -> bool;
    fn explore_lite_msgs__msg__ExploreStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExploreStatus>, size: usize) -> bool;
    fn explore_lite_msgs__msg__ExploreStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExploreStatus>);
    fn explore_lite_msgs__msg__ExploreStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExploreStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<ExploreStatus>) -> bool;
}

// Corresponds to explore_lite_msgs__msg__ExploreStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Exploration status constants
/// EXPLORATION_STARTED: Exploration has begun (published on initialization)
/// EXPLORATION_IN_PROGRESS: Exploration resumed after pause
/// EXPLORATION_PAUSED: Exploration manually stopped via /explore/resume topic
/// EXPLORATION_COMPLETE: No frontiers remaining, exploration finished
/// RETURNING_TO_ORIGIN: Robot navigating back to initial pose (if return_to_init: true)
/// RETURNED_TO_ORIGIN: Robot successfully reached initial pose

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExploreStatus {
    /// Current exploration status
    pub status: rosidl_runtime_rs::String,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !explore_lite_msgs__msg__ExploreStatus__init(&mut msg as *mut _) {
        panic!("Call to explore_lite_msgs__msg__ExploreStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExploreStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { explore_lite_msgs__msg__ExploreStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { explore_lite_msgs__msg__ExploreStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { explore_lite_msgs__msg__ExploreStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExploreStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExploreStatus where Self: Sized {
  const TYPE_NAME: &'static str = "explore_lite_msgs/msg/ExploreStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__explore_lite_msgs__msg__ExploreStatus() }
  }
}


