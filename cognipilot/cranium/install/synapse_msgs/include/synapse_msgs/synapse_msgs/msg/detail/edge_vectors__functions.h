// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from synapse_msgs:msg/EdgeVectors.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__FUNCTIONS_H_
#define SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "synapse_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "synapse_msgs/msg/detail/edge_vectors__struct.h"

/// Initialize msg/EdgeVectors message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * synapse_msgs__msg__EdgeVectors
 * )) before or use
 * synapse_msgs__msg__EdgeVectors__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__EdgeVectors__init(synapse_msgs__msg__EdgeVectors * msg);

/// Finalize msg/EdgeVectors message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
void
synapse_msgs__msg__EdgeVectors__fini(synapse_msgs__msg__EdgeVectors * msg);

/// Create msg/EdgeVectors message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * synapse_msgs__msg__EdgeVectors__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
synapse_msgs__msg__EdgeVectors *
synapse_msgs__msg__EdgeVectors__create();

/// Destroy msg/EdgeVectors message.
/**
 * It calls
 * synapse_msgs__msg__EdgeVectors__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
void
synapse_msgs__msg__EdgeVectors__destroy(synapse_msgs__msg__EdgeVectors * msg);

/// Check for msg/EdgeVectors message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__EdgeVectors__are_equal(const synapse_msgs__msg__EdgeVectors * lhs, const synapse_msgs__msg__EdgeVectors * rhs);

/// Copy a msg/EdgeVectors message.
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
synapse_msgs__msg__EdgeVectors__copy(
  const synapse_msgs__msg__EdgeVectors * input,
  synapse_msgs__msg__EdgeVectors * output);

/// Initialize array of msg/EdgeVectors messages.
/**
 * It allocates the memory for the number of elements and calls
 * synapse_msgs__msg__EdgeVectors__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__EdgeVectors__Sequence__init(synapse_msgs__msg__EdgeVectors__Sequence * array, size_t size);

/// Finalize array of msg/EdgeVectors messages.
/**
 * It calls
 * synapse_msgs__msg__EdgeVectors__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
void
synapse_msgs__msg__EdgeVectors__Sequence__fini(synapse_msgs__msg__EdgeVectors__Sequence * array);

/// Create array of msg/EdgeVectors messages.
/**
 * It allocates the memory for the array and calls
 * synapse_msgs__msg__EdgeVectors__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
synapse_msgs__msg__EdgeVectors__Sequence *
synapse_msgs__msg__EdgeVectors__Sequence__create(size_t size);

/// Destroy array of msg/EdgeVectors messages.
/**
 * It calls
 * synapse_msgs__msg__EdgeVectors__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
void
synapse_msgs__msg__EdgeVectors__Sequence__destroy(synapse_msgs__msg__EdgeVectors__Sequence * array);

/// Check for msg/EdgeVectors message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__EdgeVectors__Sequence__are_equal(const synapse_msgs__msg__EdgeVectors__Sequence * lhs, const synapse_msgs__msg__EdgeVectors__Sequence * rhs);

/// Copy an array of msg/EdgeVectors messages.
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
synapse_msgs__msg__EdgeVectors__Sequence__copy(
  const synapse_msgs__msg__EdgeVectors__Sequence * input,
  synapse_msgs__msg__EdgeVectors__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SYNAPSE_MSGS__MSG__DETAIL__EDGE_VECTORS__FUNCTIONS_H_
