// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from xarm_msgs:msg/MoveVelocity.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__MSG__DETAIL__MOVE_VELOCITY__FUNCTIONS_H_
#define XARM_MSGS__MSG__DETAIL__MOVE_VELOCITY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "xarm_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "xarm_msgs/msg/detail/move_velocity__struct.h"

/// Initialize msg/MoveVelocity message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * xarm_msgs__msg__MoveVelocity
 * )) before or use
 * xarm_msgs__msg__MoveVelocity__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
bool
xarm_msgs__msg__MoveVelocity__init(xarm_msgs__msg__MoveVelocity * msg);

/// Finalize msg/MoveVelocity message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
void
xarm_msgs__msg__MoveVelocity__fini(xarm_msgs__msg__MoveVelocity * msg);

/// Create msg/MoveVelocity message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * xarm_msgs__msg__MoveVelocity__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
xarm_msgs__msg__MoveVelocity *
xarm_msgs__msg__MoveVelocity__create();

/// Destroy msg/MoveVelocity message.
/**
 * It calls
 * xarm_msgs__msg__MoveVelocity__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
void
xarm_msgs__msg__MoveVelocity__destroy(xarm_msgs__msg__MoveVelocity * msg);

/// Check for msg/MoveVelocity message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
bool
xarm_msgs__msg__MoveVelocity__are_equal(const xarm_msgs__msg__MoveVelocity * lhs, const xarm_msgs__msg__MoveVelocity * rhs);

/// Copy a msg/MoveVelocity message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
bool
xarm_msgs__msg__MoveVelocity__copy(
  const xarm_msgs__msg__MoveVelocity * input,
  xarm_msgs__msg__MoveVelocity * output);

/// Initialize array of msg/MoveVelocity messages.
/**
 * It allocates the memory for the number of elements and calls
 * xarm_msgs__msg__MoveVelocity__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
bool
xarm_msgs__msg__MoveVelocity__Sequence__init(xarm_msgs__msg__MoveVelocity__Sequence * array, size_t size);

/// Finalize array of msg/MoveVelocity messages.
/**
 * It calls
 * xarm_msgs__msg__MoveVelocity__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
void
xarm_msgs__msg__MoveVelocity__Sequence__fini(xarm_msgs__msg__MoveVelocity__Sequence * array);

/// Create array of msg/MoveVelocity messages.
/**
 * It allocates the memory for the array and calls
 * xarm_msgs__msg__MoveVelocity__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
xarm_msgs__msg__MoveVelocity__Sequence *
xarm_msgs__msg__MoveVelocity__Sequence__create(size_t size);

/// Destroy array of msg/MoveVelocity messages.
/**
 * It calls
 * xarm_msgs__msg__MoveVelocity__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
void
xarm_msgs__msg__MoveVelocity__Sequence__destroy(xarm_msgs__msg__MoveVelocity__Sequence * array);

/// Check for msg/MoveVelocity message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
bool
xarm_msgs__msg__MoveVelocity__Sequence__are_equal(const xarm_msgs__msg__MoveVelocity__Sequence * lhs, const xarm_msgs__msg__MoveVelocity__Sequence * rhs);

/// Copy an array of msg/MoveVelocity messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
bool
xarm_msgs__msg__MoveVelocity__Sequence__copy(
  const xarm_msgs__msg__MoveVelocity__Sequence * input,
  xarm_msgs__msg__MoveVelocity__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // XARM_MSGS__MSG__DETAIL__MOVE_VELOCITY__FUNCTIONS_H_
