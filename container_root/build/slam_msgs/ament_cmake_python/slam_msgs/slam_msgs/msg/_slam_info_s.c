// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from slam_msgs:msg/SlamInfo.idl
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
#include "slam_msgs/msg/detail/slam_info__struct.h"
#include "slam_msgs/msg/detail/slam_info__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool slam_msgs__msg__slam_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[34];
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
    assert(strncmp("slam_msgs.msg._slam_info.SlamInfo", full_classname_dest, 33) == 0);
  }
  slam_msgs__msg__SlamInfo * ros_message = _ros_message;
  {  // num_maps
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_maps");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_maps = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // num_keyframes_in_current_map
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_keyframes_in_current_map");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_keyframes_in_current_map = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // tracking_frequency
    PyObject * field = PyObject_GetAttrString(_pymsg, "tracking_frequency");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tracking_frequency = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * slam_msgs__msg__slam_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SlamInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("slam_msgs.msg._slam_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SlamInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  slam_msgs__msg__SlamInfo * ros_message = (slam_msgs__msg__SlamInfo *)raw_ros_message;
  {  // num_maps
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->num_maps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_maps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_keyframes_in_current_map
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->num_keyframes_in_current_map);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_keyframes_in_current_map", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tracking_frequency
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tracking_frequency);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tracking_frequency", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
