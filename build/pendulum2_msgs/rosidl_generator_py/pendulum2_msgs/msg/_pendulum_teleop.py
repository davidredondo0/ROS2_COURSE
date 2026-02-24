# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pendulum2_msgs:msg/PendulumTeleop.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PendulumTeleop(type):
    """Metaclass of message 'PendulumTeleop'."""

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
            module = import_type_support('pendulum2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pendulum2_msgs.msg.PendulumTeleop')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pendulum_teleop
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pendulum_teleop
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pendulum_teleop
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pendulum_teleop
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pendulum_teleop

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PendulumTeleop(metaclass=Metaclass_PendulumTeleop):
    """Message class 'PendulumTeleop'."""

    __slots__ = [
        '_pole_angle',
        '_pole_velocity',
        '_cart_position',
        '_cart_velocity',
    ]

    _fields_and_field_types = {
        'pole_angle': 'double',
        'pole_velocity': 'double',
        'cart_position': 'double',
        'cart_velocity': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.pole_angle = kwargs.get('pole_angle', float())
        self.pole_velocity = kwargs.get('pole_velocity', float())
        self.cart_position = kwargs.get('cart_position', float())
        self.cart_velocity = kwargs.get('cart_velocity', float())

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
        if self.pole_angle != other.pole_angle:
            return False
        if self.pole_velocity != other.pole_velocity:
            return False
        if self.cart_position != other.cart_position:
            return False
        if self.cart_velocity != other.cart_velocity:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pole_angle(self):
        """Message field 'pole_angle'."""
        return self._pole_angle

    @pole_angle.setter
    def pole_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pole_angle' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pole_angle' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pole_angle = value

    @builtins.property
    def pole_velocity(self):
        """Message field 'pole_velocity'."""
        return self._pole_velocity

    @pole_velocity.setter
    def pole_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pole_velocity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pole_velocity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pole_velocity = value

    @builtins.property
    def cart_position(self):
        """Message field 'cart_position'."""
        return self._cart_position

    @cart_position.setter
    def cart_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cart_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'cart_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._cart_position = value

    @builtins.property
    def cart_velocity(self):
        """Message field 'cart_velocity'."""
        return self._cart_velocity

    @cart_velocity.setter
    def cart_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cart_velocity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'cart_velocity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._cart_velocity = value
