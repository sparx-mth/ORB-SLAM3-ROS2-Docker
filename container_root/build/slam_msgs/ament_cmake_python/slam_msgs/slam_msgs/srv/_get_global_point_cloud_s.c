// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from slam_msgs:srv/GetGlobalPointCloud.idl
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
#include "slam_msgs/srv/detail/get_global_point_cloud__struct.h"
#include "slam_msgs/srv/detail/get_global_point_cloud__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool slam_msgs__srv__get_global_point_cloud__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[66];
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
    assert(strncmp("slam_msgs.srv._get_global_point_cloud.GetGlobalPointCloud_Request", full_classname_dest, 65) == 0);
  }
  slam_msgs__srv__GetGlobalPointCloud_Request * ros_message = _ros_message;
  {  // global_voxel_resolution
    PyObject * field = PyObject_GetAttrString(_pymsg, "global_voxel_resolution");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->global_voxel_resolution = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // local_voxel_resolution
    PyObject * field = PyObject_GetAttrString(_pymsg, "local_voxel_resolution");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->local_voxel_resolution = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_thresh_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_thresh_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_thresh_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // get_grayscale
    PyObject * field = PyObject_GetAttrString(_pymsg, "get_grayscale");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->get_grayscale = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * slam_msgs__srv__get_global_point_cloud__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetGlobalPointCloud_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("slam_msgs.srv._get_global_point_cloud");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetGlobalPointCloud_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  slam_msgs__srv__GetGlobalPointCloud_Request * ros_message = (slam_msgs__srv__GetGlobalPointCloud_Request *)raw_ros_message;
  {  // global_voxel_resolution
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->global_voxel_resolution);
    {
      int rc = PyObject_SetAttrString(_pymessage, "global_voxel_resolution", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // local_voxel_resolution
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->local_voxel_resolution);
    {
      int rc = PyObject_SetAttrString(_pymessage, "local_voxel_resolution", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_thresh_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_thresh_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_thresh_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // get_grayscale
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->get_grayscale ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "get_grayscale", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "slam_msgs/srv/detail/get_global_point_cloud__struct.h"
// already included above
// #include "slam_msgs/srv/detail/get_global_point_cloud__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool slam_msgs__srv__get_global_point_cloud__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[67];
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
    assert(strncmp("slam_msgs.srv._get_global_point_cloud.GetGlobalPointCloud_Response", full_classname_dest, 66) == 0);
  }
  slam_msgs__srv__GetGlobalPointCloud_Response * ros_message = _ros_message;
  {  // response
    PyObject * field = PyObject_GetAttrString(_pymsg, "response");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->response = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * slam_msgs__srv__get_global_point_cloud__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetGlobalPointCloud_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("slam_msgs.srv._get_global_point_cloud");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetGlobalPointCloud_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  slam_msgs__srv__GetGlobalPointCloud_Response * ros_message = (slam_msgs__srv__GetGlobalPointCloud_Response *)raw_ros_message;
  {  // response
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->response ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "response", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
