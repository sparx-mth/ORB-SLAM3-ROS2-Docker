# generated from rosidl_generator_py/resource/_idl.py.em
# with input from slam_msgs:msg/SlamInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SlamInfo(type):
    """Metaclass of message 'SlamInfo'."""

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
            module = import_type_support('slam_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'slam_msgs.msg.SlamInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__slam_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__slam_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__slam_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__slam_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__slam_info

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SlamInfo(metaclass=Metaclass_SlamInfo):
    """Message class 'SlamInfo'."""

    __slots__ = [
        '_num_maps',
        '_num_keyframes_in_current_map',
        '_tracking_frequency',
    ]

    _fields_and_field_types = {
        'num_maps': 'int32',
        'num_keyframes_in_current_map': 'int32',
        'tracking_frequency': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.num_maps = kwargs.get('num_maps', int())
        self.num_keyframes_in_current_map = kwargs.get('num_keyframes_in_current_map', int())
        self.tracking_frequency = kwargs.get('tracking_frequency', float())

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
        if self.num_maps != other.num_maps:
            return False
        if self.num_keyframes_in_current_map != other.num_keyframes_in_current_map:
            return False
        if self.tracking_frequency != other.tracking_frequency:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def num_maps(self):
        """Message field 'num_maps'."""
        return self._num_maps

    @num_maps.setter
    def num_maps(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_maps' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'num_maps' field must be an integer in [-2147483648, 2147483647]"
        self._num_maps = value

    @builtins.property
    def num_keyframes_in_current_map(self):
        """Message field 'num_keyframes_in_current_map'."""
        return self._num_keyframes_in_current_map

    @num_keyframes_in_current_map.setter
    def num_keyframes_in_current_map(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_keyframes_in_current_map' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'num_keyframes_in_current_map' field must be an integer in [-2147483648, 2147483647]"
        self._num_keyframes_in_current_map = value

    @builtins.property
    def tracking_frequency(self):
        """Message field 'tracking_frequency'."""
        return self._tracking_frequency

    @tracking_frequency.setter
    def tracking_frequency(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tracking_frequency' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'tracking_frequency' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._tracking_frequency = value
