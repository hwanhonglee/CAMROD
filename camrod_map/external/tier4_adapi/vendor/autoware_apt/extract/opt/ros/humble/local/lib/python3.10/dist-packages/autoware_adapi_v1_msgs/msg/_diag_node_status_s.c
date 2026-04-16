// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from autoware_adapi_v1_msgs:msg/DiagNodeStatus.idl
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
#include "autoware_adapi_v1_msgs/msg/detail/diag_node_status__struct.h"
#include "autoware_adapi_v1_msgs/msg/detail/diag_node_status__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool autoware_adapi_v1_msgs__msg__diag_node_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[60];
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
    assert(strncmp("autoware_adapi_v1_msgs.msg._diag_node_status.DiagNodeStatus", full_classname_dest, 59) == 0);
  }
  autoware_adapi_v1_msgs__msg__DiagNodeStatus * ros_message = _ros_message;
  {  // level
    PyObject * field = PyObject_GetAttrString(_pymsg, "level");
    if (!field) {
      return false;
    }
    assert(PyBytes_Check(field));
    ros_message->level = PyBytes_AS_STRING(field)[0];
    Py_DECREF(field);
  }
  {  // input_level
    PyObject * field = PyObject_GetAttrString(_pymsg, "input_level");
    if (!field) {
      return false;
    }
    assert(PyBytes_Check(field));
    ros_message->input_level = PyBytes_AS_STRING(field)[0];
    Py_DECREF(field);
  }
  {  // latch_level
    PyObject * field = PyObject_GetAttrString(_pymsg, "latch_level");
    if (!field) {
      return false;
    }
    assert(PyBytes_Check(field));
    ros_message->latch_level = PyBytes_AS_STRING(field)[0];
    Py_DECREF(field);
  }
  {  // is_dependent
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_dependent");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_dependent = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * autoware_adapi_v1_msgs__msg__diag_node_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DiagNodeStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("autoware_adapi_v1_msgs.msg._diag_node_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DiagNodeStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  autoware_adapi_v1_msgs__msg__DiagNodeStatus * ros_message = (autoware_adapi_v1_msgs__msg__DiagNodeStatus *)raw_ros_message;
  {  // level
    PyObject * field = NULL;
    field = PyBytes_FromStringAndSize((const char *)&ros_message->level, 1);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "level", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // input_level
    PyObject * field = NULL;
    field = PyBytes_FromStringAndSize((const char *)&ros_message->input_level, 1);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "input_level", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // latch_level
    PyObject * field = NULL;
    field = PyBytes_FromStringAndSize((const char *)&ros_message->latch_level, 1);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "latch_level", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_dependent
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_dependent ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_dependent", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
