// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from xarm_msgs:msg/MoveVelocity.idl
// generated code does not contain a copyright notice
#include "xarm_msgs/msg/detail/move_velocity__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "xarm_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "xarm_msgs/msg/detail/move_velocity__struct.h"
#include "xarm_msgs/msg/detail/move_velocity__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // speeds
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // speeds

// forward declare type support functions


using _MoveVelocity__ros_msg_type = xarm_msgs__msg__MoveVelocity;

static bool _MoveVelocity__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MoveVelocity__ros_msg_type * ros_message = static_cast<const _MoveVelocity__ros_msg_type *>(untyped_ros_message);
  // Field name: speeds
  {
    size_t size = ros_message->speeds.size;
    auto array_ptr = ros_message->speeds.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: is_sync
  {
    cdr << (ros_message->is_sync ? true : false);
  }

  // Field name: is_tool_coord
  {
    cdr << (ros_message->is_tool_coord ? true : false);
  }

  // Field name: duration
  {
    cdr << ros_message->duration;
  }

  return true;
}

static bool _MoveVelocity__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MoveVelocity__ros_msg_type * ros_message = static_cast<_MoveVelocity__ros_msg_type *>(untyped_ros_message);
  // Field name: speeds
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->speeds.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->speeds);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->speeds, size)) {
      fprintf(stderr, "failed to create array for field 'speeds'");
      return false;
    }
    auto array_ptr = ros_message->speeds.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: is_sync
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_sync = tmp ? true : false;
  }

  // Field name: is_tool_coord
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_tool_coord = tmp ? true : false;
  }

  // Field name: duration
  {
    cdr >> ros_message->duration;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t get_serialized_size_xarm_msgs__msg__MoveVelocity(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MoveVelocity__ros_msg_type * ros_message = static_cast<const _MoveVelocity__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name speeds
  {
    size_t array_size = ros_message->speeds.size;
    auto array_ptr = ros_message->speeds.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_sync
  {
    size_t item_size = sizeof(ros_message->is_sync);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_tool_coord
  {
    size_t item_size = sizeof(ros_message->is_tool_coord);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name duration
  {
    size_t item_size = sizeof(ros_message->duration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MoveVelocity__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_xarm_msgs__msg__MoveVelocity(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t max_serialized_size_xarm_msgs__msg__MoveVelocity(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: speeds
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: is_sync
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_tool_coord
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: duration
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = xarm_msgs__msg__MoveVelocity;
    is_plain =
      (
      offsetof(DataType, duration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MoveVelocity__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_xarm_msgs__msg__MoveVelocity(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MoveVelocity = {
  "xarm_msgs::msg",
  "MoveVelocity",
  _MoveVelocity__cdr_serialize,
  _MoveVelocity__cdr_deserialize,
  _MoveVelocity__get_serialized_size,
  _MoveVelocity__max_serialized_size
};

static rosidl_message_type_support_t _MoveVelocity__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MoveVelocity,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, msg, MoveVelocity)() {
  return &_MoveVelocity__type_support;
}

#if defined(__cplusplus)
}
#endif
