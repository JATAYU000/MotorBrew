// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from synapse_msgs:msg/WarehouseShelf.idl
// generated code does not contain a copyright notice

#ifndef SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__FUNCTIONS_H_
#define SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "synapse_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "synapse_msgs/msg/detail/warehouse_shelf__struct.h"

/// Initialize msg/WarehouseShelf message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * synapse_msgs__msg__WarehouseShelf
 * )) before or use
 * synapse_msgs__msg__WarehouseShelf__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__WarehouseShelf__init(synapse_msgs__msg__WarehouseShelf * msg);

/// Finalize msg/WarehouseShelf message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
void
synapse_msgs__msg__WarehouseShelf__fini(synapse_msgs__msg__WarehouseShelf * msg);

/// Create msg/WarehouseShelf message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * synapse_msgs__msg__WarehouseShelf__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
synapse_msgs__msg__WarehouseShelf *
synapse_msgs__msg__WarehouseShelf__create();

/// Destroy msg/WarehouseShelf message.
/**
 * It calls
 * synapse_msgs__msg__WarehouseShelf__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
void
synapse_msgs__msg__WarehouseShelf__destroy(synapse_msgs__msg__WarehouseShelf * msg);

/// Check for msg/WarehouseShelf message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__WarehouseShelf__are_equal(const synapse_msgs__msg__WarehouseShelf * lhs, const synapse_msgs__msg__WarehouseShelf * rhs);

/// Copy a msg/WarehouseShelf message.
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
synapse_msgs__msg__WarehouseShelf__copy(
  const synapse_msgs__msg__WarehouseShelf * input,
  synapse_msgs__msg__WarehouseShelf * output);

/// Initialize array of msg/WarehouseShelf messages.
/**
 * It allocates the memory for the number of elements and calls
 * synapse_msgs__msg__WarehouseShelf__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__WarehouseShelf__Sequence__init(synapse_msgs__msg__WarehouseShelf__Sequence * array, size_t size);

/// Finalize array of msg/WarehouseShelf messages.
/**
 * It calls
 * synapse_msgs__msg__WarehouseShelf__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
void
synapse_msgs__msg__WarehouseShelf__Sequence__fini(synapse_msgs__msg__WarehouseShelf__Sequence * array);

/// Create array of msg/WarehouseShelf messages.
/**
 * It allocates the memory for the array and calls
 * synapse_msgs__msg__WarehouseShelf__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
synapse_msgs__msg__WarehouseShelf__Sequence *
synapse_msgs__msg__WarehouseShelf__Sequence__create(size_t size);

/// Destroy array of msg/WarehouseShelf messages.
/**
 * It calls
 * synapse_msgs__msg__WarehouseShelf__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
void
synapse_msgs__msg__WarehouseShelf__Sequence__destroy(synapse_msgs__msg__WarehouseShelf__Sequence * array);

/// Check for msg/WarehouseShelf message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_synapse_msgs
bool
synapse_msgs__msg__WarehouseShelf__Sequence__are_equal(const synapse_msgs__msg__WarehouseShelf__Sequence * lhs, const synapse_msgs__msg__WarehouseShelf__Sequence * rhs);

/// Copy an array of msg/WarehouseShelf messages.
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
synapse_msgs__msg__WarehouseShelf__Sequence__copy(
  const synapse_msgs__msg__WarehouseShelf__Sequence * input,
  synapse_msgs__msg__WarehouseShelf__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SYNAPSE_MSGS__MSG__DETAIL__WAREHOUSE_SHELF__FUNCTIONS_H_
