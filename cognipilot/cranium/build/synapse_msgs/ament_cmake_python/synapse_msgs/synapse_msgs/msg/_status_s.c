// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from synapse_msgs:msg/Status.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "synapse_msgs/msg/detail/status__struct.h"
#include "synapse_msgs/msg/detail/status__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool synapse_msgs__msg__status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[32];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("synapse_msgs.msg._status.Status", full_classname_dest, 31) == 0);
  }
  synapse_msgs__msg__Status * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // arming
    PyObject * field = PyObject_GetAttrString(_pymsg, "arming");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->arming = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // joy
    PyObject * field = PyObject_GetAttrString(_pymsg, "joy");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->joy = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // fuel
    PyObject * field = PyObject_GetAttrString(_pymsg, "fuel");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fuel = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // safety
    PyObject * field = PyObject_GetAttrString(_pymsg, "safety");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->safety = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // synapse
    PyObject * field = PyObject_GetAttrString(_pymsg, "synapse");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->synapse = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // status_message
    PyObject * field = PyObject_GetAttrString(_pymsg, "status_message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->status_message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // fuel_percentage
    PyObject * field = PyObject_GetAttrString(_pymsg, "fuel_percentage");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fuel_percentage = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // power
    PyObject * field = PyObject_GetAttrString(_pymsg, "power");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->power = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // request_seq
    PyObject * field = PyObject_GetAttrString(_pymsg, "request_seq");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->request_seq = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // request_rejected
    PyObject * field = PyObject_GetAttrString(_pymsg, "request_rejected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->request_rejected = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * synapse_msgs__msg__status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Status */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("synapse_msgs.msg._status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Status");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  synapse_msgs__msg__Status * ros_message = (synapse_msgs__msg__Status *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // arming
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->arming);
    {
      int rc = PyObject_SetAttrString(_pymessage, "arming", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // joy
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->joy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "joy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fuel
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->fuel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fuel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // safety
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->safety);
    {
      int rc = PyObject_SetAttrString(_pymessage, "safety", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // synapse
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->synapse);
    {
      int rc = PyObject_SetAttrString(_pymessage, "synapse", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // status_message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->status_message.data,
      strlen(ros_message->status_message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "status_message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fuel_percentage
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->fuel_percentage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fuel_percentage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // power
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->power);
    {
      int rc = PyObject_SetAttrString(_pymessage, "power", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // request_seq
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->request_seq);
    {
      int rc = PyObject_SetAttrString(_pymessage, "request_seq", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // request_rejected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->request_rejected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "request_rejected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
