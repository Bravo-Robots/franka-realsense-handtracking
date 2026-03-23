// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from hand_interfaces:msg/Pinch.idl
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
#include "hand_interfaces/msg/detail/pinch__struct.h"
#include "hand_interfaces/msg/detail/pinch__functions.h"

bool hand_interfaces__msg__finger_data__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * hand_interfaces__msg__finger_data__convert_to_py(void * raw_ros_message);
bool hand_interfaces__msg__finger_data__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * hand_interfaces__msg__finger_data__convert_to_py(void * raw_ros_message);
bool hand_interfaces__msg__finger_data__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * hand_interfaces__msg__finger_data__convert_to_py(void * raw_ros_message);
bool hand_interfaces__msg__finger_data__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * hand_interfaces__msg__finger_data__convert_to_py(void * raw_ros_message);
bool hand_interfaces__msg__finger_data__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * hand_interfaces__msg__finger_data__convert_to_py(void * raw_ros_message);
bool hand_interfaces__msg__finger_data__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * hand_interfaces__msg__finger_data__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool hand_interfaces__msg__pinch__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[33];
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
    assert(strncmp("hand_interfaces.msg._pinch.Pinch", full_classname_dest, 32) == 0);
  }
  hand_interfaces__msg__Pinch * ros_message = _ros_message;
  {  // wrist
    PyObject * field = PyObject_GetAttrString(_pymsg, "wrist");
    if (!field) {
      return false;
    }
    if (!hand_interfaces__msg__finger_data__convert_from_py(field, &ros_message->wrist)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // thumb
    PyObject * field = PyObject_GetAttrString(_pymsg, "thumb");
    if (!field) {
      return false;
    }
    if (!hand_interfaces__msg__finger_data__convert_from_py(field, &ros_message->thumb)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // index
    PyObject * field = PyObject_GetAttrString(_pymsg, "index");
    if (!field) {
      return false;
    }
    if (!hand_interfaces__msg__finger_data__convert_from_py(field, &ros_message->index)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // middle
    PyObject * field = PyObject_GetAttrString(_pymsg, "middle");
    if (!field) {
      return false;
    }
    if (!hand_interfaces__msg__finger_data__convert_from_py(field, &ros_message->middle)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // ring
    PyObject * field = PyObject_GetAttrString(_pymsg, "ring");
    if (!field) {
      return false;
    }
    if (!hand_interfaces__msg__finger_data__convert_from_py(field, &ros_message->ring)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // pinky
    PyObject * field = PyObject_GetAttrString(_pymsg, "pinky");
    if (!field) {
      return false;
    }
    if (!hand_interfaces__msg__finger_data__convert_from_py(field, &ros_message->pinky)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * hand_interfaces__msg__pinch__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Pinch */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("hand_interfaces.msg._pinch");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Pinch");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  hand_interfaces__msg__Pinch * ros_message = (hand_interfaces__msg__Pinch *)raw_ros_message;
  {  // wrist
    PyObject * field = NULL;
    field = hand_interfaces__msg__finger_data__convert_to_py(&ros_message->wrist);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "wrist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thumb
    PyObject * field = NULL;
    field = hand_interfaces__msg__finger_data__convert_to_py(&ros_message->thumb);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "thumb", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // index
    PyObject * field = NULL;
    field = hand_interfaces__msg__finger_data__convert_to_py(&ros_message->index);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "index", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // middle
    PyObject * field = NULL;
    field = hand_interfaces__msg__finger_data__convert_to_py(&ros_message->middle);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "middle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ring
    PyObject * field = NULL;
    field = hand_interfaces__msg__finger_data__convert_to_py(&ros_message->ring);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "ring", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pinky
    PyObject * field = NULL;
    field = hand_interfaces__msg__finger_data__convert_to_py(&ros_message->pinky);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pinky", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
