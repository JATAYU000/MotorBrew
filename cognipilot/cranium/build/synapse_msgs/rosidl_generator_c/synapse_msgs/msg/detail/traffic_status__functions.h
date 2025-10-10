// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from synapse_msgs:msg/TrafficStatus.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__TRAFFIC_STATUS__FUNCTIONS_H_
#define SYNAPSE_MSGS__MSG__DETAIL__TRAFFIC_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "synapse_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "synapse_msgs/msg/detail/traffic_status__struct.h"

/// Initialize msg/TrafficStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * synapse_msgs__msg__TrafficStatus
 * )) before or use
 * synapse_msgs__msg__TrafficStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__TrafficStatus__init(synapse_msgs__msg__TrafficStatus * msg);

/// Finalize msg/TrafficStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
void
synapse_msgs__msg__TrafficStatus__fini(synapse_msgs__msg__TrafficStatus * msg);

/// Create msg/TrafficStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * synapse_msgs__msg__TrafficStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
synapse_msgs__msg__TrafficStatus *
synapse_msgs__msg__TrafficStatus__create();

/// Destroy msg/TrafficStatus message.
/**
 * It calls
 * synapse_msgs__msg__TrafficStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
void
synapse_msgs__msg__TrafficStatus__destroy(synapse_msgs__msg__TrafficStatus * msg);

/// Check for msg/TrafficStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__TrafficStatus__are_equal(const synapse_msgs__msg__TrafficStatus * lhs, const synapse_msgs__msg__TrafficStatus * rhs);

/// Copy a msg/TrafficStatus message.
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
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__TrafficStatus__copy(
  const synapse_msgs__msg__TrafficStatus * input,
  synapse_msgs__msg__TrafficStatus * output);

/// Initialize array of msg/TrafficStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * synapse_msgs__msg__TrafficStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__TrafficStatus__Sequence__init(synapse_msgs__msg__TrafficStatus__Sequence * array, size_t size);

/// Finalize array of msg/TrafficStatus messages.
/**
 * It calls
 * synapse_msgs__msg__TrafficStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
void
synapse_msgs__msg__TrafficStatus__Sequence__fini(synapse_msgs__msg__TrafficStatus__Sequence * array);

/// Create array of msg/TrafficStatus messages.
/**
 * It allocates the memory for the array and calls
 * synapse_msgs__msg__TrafficStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
synapse_msgs__msg__TrafficStatus__Sequence *
synapse_msgs__msg__TrafficStatus__Sequence__create(size_t size);

/// Destroy array of msg/TrafficStatus messages.
/**
 * It calls
 * synapse_msgs__msg__TrafficStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
void
synapse_msgs__msg__TrafficStatus__Sequence__destroy(synapse_msgs__msg__TrafficStatus__Sequence * array);

/// Check for msg/TrafficStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__TrafficStatus__Sequence__are_equal(const synapse_msgs__msg__TrafficStatus__Sequence * lhs, const synapse_msgs__msg__TrafficStatus__Sequence * rhs);

/// Copy an array of msg/TrafficStatus messages.
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
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__TrafficStatus__Sequence__copy(
  const synapse_msgs__msg__TrafficStatus__Sequence * input,
  synapse_msgs__msg__TrafficStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SYNAPSE_MSGS__MSG__DETAIL__TRAFFIC_STATUS__FUNCTIONS_H_
