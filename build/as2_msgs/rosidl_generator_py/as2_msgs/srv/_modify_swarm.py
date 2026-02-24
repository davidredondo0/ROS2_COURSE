# generated from rosidl_generator_py/resource/_idl.py.em
# with input from as2_msgs:srv/ModifySwarm.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ModifySwarm_Request(type):
    """Metaclass of message 'ModifySwarm_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('as2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'as2_msgs.srv.ModifySwarm_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__modify_swarm__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__modify_swarm__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__modify_swarm__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__modify_swarm__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__modify_swarm__request

            from as2_msgs.msg import PoseWithID
            if PoseWithID.__class__._TYPE_SUPPORT is None:
                PoseWithID.__class__.__import_type_support__()

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ModifySwarm_Request(metaclass=Metaclass_ModifySwarm_Request):
    """Message class 'ModifySwarm_Request'."""

    __slots__ = [
        '_detach_drone',
        '_new_drone',
        '_new_virtual_centroid_ref',
        '_virtual_centroid',
        '_swarm_formation',
    ]

    _fields_and_field_types = {
        'detach_drone': 'boolean',
        'new_drone': 'boolean',
        'new_virtual_centroid_ref': 'boolean',
        'virtual_centroid': 'geometry_msgs/PoseStamped',
        'swarm_formation': 'sequence<as2_msgs/PoseWithID>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['as2_msgs', 'msg'], 'PoseWithID')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.detach_drone = kwargs.get('detach_drone', bool())
        self.new_drone = kwargs.get('new_drone', bool())
        self.new_virtual_centroid_ref = kwargs.get('new_virtual_centroid_ref', bool())
        from geometry_msgs.msg import PoseStamped
        self.virtual_centroid = kwargs.get('virtual_centroid', PoseStamped())
        self.swarm_formation = kwargs.get('swarm_formation', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.detach_drone != other.detach_drone:
            return False
        if self.new_drone != other.new_drone:
            return False
        if self.new_virtual_centroid_ref != other.new_virtual_centroid_ref:
            return False
        if self.virtual_centroid != other.virtual_centroid:
            return False
        if self.swarm_formation != other.swarm_formation:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def detach_drone(self):
        """Message field 'detach_drone'."""
        return self._detach_drone

    @detach_drone.setter
    def detach_drone(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'detach_drone' field must be of type 'bool'"
        self._detach_drone = value

    @builtins.property
    def new_drone(self):
        """Message field 'new_drone'."""
        return self._new_drone

    @new_drone.setter
    def new_drone(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'new_drone' field must be of type 'bool'"
        self._new_drone = value

    @builtins.property
    def new_virtual_centroid_ref(self):
        """Message field 'new_virtual_centroid_ref'."""
        return self._new_virtual_centroid_ref

    @new_virtual_centroid_ref.setter
    def new_virtual_centroid_ref(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'new_virtual_centroid_ref' field must be of type 'bool'"
        self._new_virtual_centroid_ref = value

    @builtins.property
    def virtual_centroid(self):
        """Message field 'virtual_centroid'."""
        return self._virtual_centroid

    @virtual_centroid.setter
    def virtual_centroid(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'virtual_centroid' field must be a sub message of type 'PoseStamped'"
        self._virtual_centroid = value

    @builtins.property
    def swarm_formation(self):
        """Message field 'swarm_formation'."""
        return self._swarm_formation

    @swarm_formation.setter
    def swarm_formation(self, value):
        if __debug__:
            from as2_msgs.msg import PoseWithID
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, PoseWithID) for v in value) and
                 True), \
                "The 'swarm_formation' field must be a set or sequence and each value of type 'PoseWithID'"
        self._swarm_formation = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ModifySwarm_Response(type):
    """Metaclass of message 'ModifySwarm_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('as2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'as2_msgs.srv.ModifySwarm_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__modify_swarm__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__modify_swarm__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__modify_swarm__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__modify_swarm__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__modify_swarm__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ModifySwarm_Response(metaclass=Metaclass_ModifySwarm_Response):
    """Message class 'ModifySwarm_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_ModifySwarm(type):
    """Metaclass of service 'ModifySwarm'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('as2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'as2_msgs.srv.ModifySwarm')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__modify_swarm

            from as2_msgs.srv import _modify_swarm
            if _modify_swarm.Metaclass_ModifySwarm_Request._TYPE_SUPPORT is None:
                _modify_swarm.Metaclass_ModifySwarm_Request.__import_type_support__()
            if _modify_swarm.Metaclass_ModifySwarm_Response._TYPE_SUPPORT is None:
                _modify_swarm.Metaclass_ModifySwarm_Response.__import_type_support__()


class ModifySwarm(metaclass=Metaclass_ModifySwarm):
    from as2_msgs.srv._modify_swarm import ModifySwarm_Request as Request
    from as2_msgs.srv._modify_swarm import ModifySwarm_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
