// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from as2_msgs:action/SwarmFlocking.idl
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
#include "as2_msgs/action/detail/swarm_flocking__struct.h"
#include "as2_msgs/action/detail/swarm_flocking__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

// Nested array functions includes
#include "as2_msgs/msg/detail/pose_with_id__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose_stamped__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose_stamped__convert_to_py(void * raw_ros_message);
bool as2_msgs__msg__pose_with_id__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * as2_msgs__msg__pose_with_id__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool as2_msgs__action__swarm_flocking__goal__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
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
    assert(strncmp("as2_msgs.action._swarm_flocking.SwarmFlocking_Goal", full_classname_dest, 50) == 0);
  }
  as2_msgs__action__SwarmFlocking_Goal * ros_message = _ros_message;
  {  // virtual_centroid
    PyObject * field = PyObject_GetAttrString(_pymsg, "virtual_centroid");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose_stamped__convert_from_py(field, &ros_message->virtual_centroid)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // swarm_formation
    PyObject * field = PyObject_GetAttrString(_pymsg, "swarm_formation");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'swarm_formation'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!as2_msgs__msg__PoseWithID__Sequence__init(&(ros_message->swarm_formation), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create as2_msgs__msg__PoseWithID__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    as2_msgs__msg__PoseWithID * dest = ros_message->swarm_formation.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!as2_msgs__msg__pose_with_id__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // drones_namespace
    PyObject * field = PyObject_GetAttrString(_pymsg, "drones_namespace");
    if (!field) {
      return false;
    }
    {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'drones_namespace'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = PySequence_Size(field);
      if (-1 == size) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      if (!rosidl_runtime_c__String__Sequence__init(&(ros_message->drones_namespace), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create String__Sequence ros_message");
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      rosidl_runtime_c__String * dest = ros_message->drones_namespace.data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyUnicode_Check(item));
        PyObject * encoded_item = PyUnicode_AsUTF8String(item);
        if (!encoded_item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        rosidl_runtime_c__String__assign(&dest[i], PyBytes_AS_STRING(encoded_item));
        Py_DECREF(encoded_item);
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * as2_msgs__action__swarm_flocking__goal__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SwarmFlocking_Goal */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("as2_msgs.action._swarm_flocking");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SwarmFlocking_Goal");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  as2_msgs__action__SwarmFlocking_Goal * ros_message = (as2_msgs__action__SwarmFlocking_Goal *)raw_ros_message;
  {  // virtual_centroid
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose_stamped__convert_to_py(&ros_message->virtual_centroid);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "virtual_centroid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // swarm_formation
    PyObject * field = NULL;
    size_t size = ros_message->swarm_formation.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    as2_msgs__msg__PoseWithID * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->swarm_formation.data[i]);
      PyObject * pyitem = as2_msgs__msg__pose_with_id__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "swarm_formation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // drones_namespace
    PyObject * field = NULL;
    size_t size = ros_message->drones_namespace.size;
    rosidl_runtime_c__String * src = ros_message->drones_namespace.data;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
      PyObject * decoded_item = PyUnicode_DecodeUTF8(src[i].data, strlen(src[i].data), "replace");
      if (!decoded_item) {
        return NULL;
      }
      int rc = PyList_SetItem(field, i, decoded_item);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "drones_namespace", field);
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
// #include "as2_msgs/action/detail/swarm_flocking__struct.h"
// already included above
// #include "as2_msgs/action/detail/swarm_flocking__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool as2_msgs__action__swarm_flocking__result__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[53];
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
    assert(strncmp("as2_msgs.action._swarm_flocking.SwarmFlocking_Result", full_classname_dest, 52) == 0);
  }
  as2_msgs__action__SwarmFlocking_Result * ros_message = _ros_message;
  {  // swarm_success
    PyObject * field = PyObject_GetAttrString(_pymsg, "swarm_success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->swarm_success = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * as2_msgs__action__swarm_flocking__result__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SwarmFlocking_Result */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("as2_msgs.action._swarm_flocking");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SwarmFlocking_Result");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  as2_msgs__action__SwarmFlocking_Result * ros_message = (as2_msgs__action__SwarmFlocking_Result *)raw_ros_message;
  {  // swarm_success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->swarm_success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "swarm_success", field);
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
// #include "as2_msgs/action/detail/swarm_flocking__struct.h"
// already included above
// #include "as2_msgs/action/detail/swarm_flocking__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool as2_msgs__action__swarm_flocking__feedback__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[55];
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
    assert(strncmp("as2_msgs.action._swarm_flocking.SwarmFlocking_Feedback", full_classname_dest, 54) == 0);
  }
  as2_msgs__action__SwarmFlocking_Feedback * ros_message = _ros_message;
  {  // swarm_pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "swarm_pose");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->swarm_pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * as2_msgs__action__swarm_flocking__feedback__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SwarmFlocking_Feedback */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("as2_msgs.action._swarm_flocking");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SwarmFlocking_Feedback");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  as2_msgs__action__SwarmFlocking_Feedback * ros_message = (as2_msgs__action__SwarmFlocking_Feedback *)raw_ros_message;
  {  // swarm_pose
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->swarm_pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "swarm_pose", field);
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
// #include "as2_msgs/action/detail/swarm_flocking__struct.h"
// already included above
// #include "as2_msgs/action/detail/swarm_flocking__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool unique_identifier_msgs__msg__uuid__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * unique_identifier_msgs__msg__uuid__convert_to_py(void * raw_ros_message);
bool as2_msgs__action__swarm_flocking__goal__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * as2_msgs__action__swarm_flocking__goal__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool as2_msgs__action__swarm_flocking__send_goal__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[63];
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
    assert(strncmp("as2_msgs.action._swarm_flocking.SwarmFlocking_SendGoal_Request", full_classname_dest, 62) == 0);
  }
  as2_msgs__action__SwarmFlocking_SendGoal_Request * ros_message = _ros_message;
  {  // goal_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_id");
    if (!field) {
      return false;
    }
    if (!unique_identifier_msgs__msg__uuid__convert_from_py(field, &ros_message->goal_id)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // goal
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal");
    if (!field) {
      return false;
    }
    if (!as2_msgs__action__swarm_flocking__goal__convert_from_py(field, &ros_message->goal)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * as2_msgs__action__swarm_flocking__send_goal__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SwarmFlocking_SendGoal_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("as2_msgs.action._swarm_flocking");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SwarmFlocking_SendGoal_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  as2_msgs__action__SwarmFlocking_SendGoal_Request * ros_message = (as2_msgs__action__SwarmFlocking_SendGoal_Request *)raw_ros_message;
  {  // goal_id
    PyObject * field = NULL;
    field = unique_identifier_msgs__msg__uuid__convert_to_py(&ros_message->goal_id);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // goal
    PyObject * field = NULL;
    field = as2_msgs__action__swarm_flocking__goal__convert_to_py(&ros_message->goal);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal", field);
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
// #include "as2_msgs/action/detail/swarm_flocking__struct.h"
// already included above
// #include "as2_msgs/action/detail/swarm_flocking__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool as2_msgs__action__swarm_flocking__send_goal__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[64];
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
    assert(strncmp("as2_msgs.action._swarm_flocking.SwarmFlocking_SendGoal_Response", full_classname_dest, 63) == 0);
  }
  as2_msgs__action__SwarmFlocking_SendGoal_Response * ros_message = _ros_message;
  {  // accepted
    PyObject * field = PyObject_GetAttrString(_pymsg, "accepted");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->accepted = (Py_True == field);
    Py_DECREF(field);
  }
  {  // stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "stamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->stamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * as2_msgs__action__swarm_flocking__send_goal__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SwarmFlocking_SendGoal_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("as2_msgs.action._swarm_flocking");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SwarmFlocking_SendGoal_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  as2_msgs__action__SwarmFlocking_SendGoal_Response * ros_message = (as2_msgs__action__SwarmFlocking_SendGoal_Response *)raw_ros_message;
  {  // accepted
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->accepted ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "accepted", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->stamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "stamp", field);
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
// #include "as2_msgs/action/detail/swarm_flocking__struct.h"
// already included above
// #include "as2_msgs/action/detail/swarm_flocking__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool unique_identifier_msgs__msg__uuid__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * unique_identifier_msgs__msg__uuid__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool as2_msgs__action__swarm_flocking__get_result__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[64];
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
    assert(strncmp("as2_msgs.action._swarm_flocking.SwarmFlocking_GetResult_Request", full_classname_dest, 63) == 0);
  }
  as2_msgs__action__SwarmFlocking_GetResult_Request * ros_message = _ros_message;
  {  // goal_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_id");
    if (!field) {
      return false;
    }
    if (!unique_identifier_msgs__msg__uuid__convert_from_py(field, &ros_message->goal_id)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * as2_msgs__action__swarm_flocking__get_result__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SwarmFlocking_GetResult_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("as2_msgs.action._swarm_flocking");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SwarmFlocking_GetResult_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  as2_msgs__action__SwarmFlocking_GetResult_Request * ros_message = (as2_msgs__action__SwarmFlocking_GetResult_Request *)raw_ros_message;
  {  // goal_id
    PyObject * field = NULL;
    field = unique_identifier_msgs__msg__uuid__convert_to_py(&ros_message->goal_id);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_id", field);
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
// #include "as2_msgs/action/detail/swarm_flocking__struct.h"
// already included above
// #include "as2_msgs/action/detail/swarm_flocking__functions.h"

bool as2_msgs__action__swarm_flocking__result__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * as2_msgs__action__swarm_flocking__result__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool as2_msgs__action__swarm_flocking__get_result__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[65];
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
    assert(strncmp("as2_msgs.action._swarm_flocking.SwarmFlocking_GetResult_Response", full_classname_dest, 64) == 0);
  }
  as2_msgs__action__SwarmFlocking_GetResult_Response * ros_message = _ros_message;
  {  // status
    PyObject * field = PyObject_GetAttrString(_pymsg, "status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->status = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // result
    PyObject * field = PyObject_GetAttrString(_pymsg, "result");
    if (!field) {
      return false;
    }
    if (!as2_msgs__action__swarm_flocking__result__convert_from_py(field, &ros_message->result)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * as2_msgs__action__swarm_flocking__get_result__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SwarmFlocking_GetResult_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("as2_msgs.action._swarm_flocking");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SwarmFlocking_GetResult_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  as2_msgs__action__SwarmFlocking_GetResult_Response * ros_message = (as2_msgs__action__SwarmFlocking_GetResult_Response *)raw_ros_message;
  {  // status
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // result
    PyObject * field = NULL;
    field = as2_msgs__action__swarm_flocking__result__convert_to_py(&ros_message->result);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "result", field);
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
// #include "as2_msgs/action/detail/swarm_flocking__struct.h"
// already included above
// #include "as2_msgs/action/detail/swarm_flocking__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool unique_identifier_msgs__msg__uuid__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * unique_identifier_msgs__msg__uuid__convert_to_py(void * raw_ros_message);
bool as2_msgs__action__swarm_flocking__feedback__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * as2_msgs__action__swarm_flocking__feedback__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool as2_msgs__action__swarm_flocking__feedback_message__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[62];
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
    assert(strncmp("as2_msgs.action._swarm_flocking.SwarmFlocking_FeedbackMessage", full_classname_dest, 61) == 0);
  }
  as2_msgs__action__SwarmFlocking_FeedbackMessage * ros_message = _ros_message;
  {  // goal_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_id");
    if (!field) {
      return false;
    }
    if (!unique_identifier_msgs__msg__uuid__convert_from_py(field, &ros_message->goal_id)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // feedback
    PyObject * field = PyObject_GetAttrString(_pymsg, "feedback");
    if (!field) {
      return false;
    }
    if (!as2_msgs__action__swarm_flocking__feedback__convert_from_py(field, &ros_message->feedback)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * as2_msgs__action__swarm_flocking__feedback_message__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SwarmFlocking_FeedbackMessage */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("as2_msgs.action._swarm_flocking");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SwarmFlocking_FeedbackMessage");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  as2_msgs__action__SwarmFlocking_FeedbackMessage * ros_message = (as2_msgs__action__SwarmFlocking_FeedbackMessage *)raw_ros_message;
  {  // goal_id
    PyObject * field = NULL;
    field = unique_identifier_msgs__msg__uuid__convert_to_py(&ros_message->goal_id);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // feedback
    PyObject * field = NULL;
    field = as2_msgs__action__swarm_flocking__feedback__convert_to_py(&ros_message->feedback);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "feedback", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
