# generated from rosidl_generator_py/resource/_idl.py.em
# with input from slam_msgs:srv/GetGlobalPointCloud.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetGlobalPointCloud_Request(type):
    """Metaclass of message 'GetGlobalPointCloud_Request'."""

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
                'slam_msgs.srv.GetGlobalPointCloud_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_global_point_cloud__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_global_point_cloud__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_global_point_cloud__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_global_point_cloud__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_global_point_cloud__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'GLOBAL_VOXEL_RESOLUTION__DEFAULT': 0.3,
            'LOCAL_VOXEL_RESOLUTION__DEFAULT': 0.3,
            'Z_THRESH_MAX__DEFAULT': 2.0,
            'GET_GRAYSCALE__DEFAULT': True,
        }

    @property
    def GLOBAL_VOXEL_RESOLUTION__DEFAULT(cls):
        """Return default value for message field 'global_voxel_resolution'."""
        return 0.3

    @property
    def LOCAL_VOXEL_RESOLUTION__DEFAULT(cls):
        """Return default value for message field 'local_voxel_resolution'."""
        return 0.3

    @property
    def Z_THRESH_MAX__DEFAULT(cls):
        """Return default value for message field 'z_thresh_max'."""
        return 2.0

    @property
    def GET_GRAYSCALE__DEFAULT(cls):
        """Return default value for message field 'get_grayscale'."""
        return True


class GetGlobalPointCloud_Request(metaclass=Metaclass_GetGlobalPointCloud_Request):
    """Message class 'GetGlobalPointCloud_Request'."""

    __slots__ = [
        '_global_voxel_resolution',
        '_local_voxel_resolution',
        '_z_thresh_max',
        '_get_grayscale',
    ]

    _fields_and_field_types = {
        'global_voxel_resolution': 'float',
        'local_voxel_resolution': 'float',
        'z_thresh_max': 'float',
        'get_grayscale': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.global_voxel_resolution = kwargs.get(
            'global_voxel_resolution', GetGlobalPointCloud_Request.GLOBAL_VOXEL_RESOLUTION__DEFAULT)
        self.local_voxel_resolution = kwargs.get(
            'local_voxel_resolution', GetGlobalPointCloud_Request.LOCAL_VOXEL_RESOLUTION__DEFAULT)
        self.z_thresh_max = kwargs.get(
            'z_thresh_max', GetGlobalPointCloud_Request.Z_THRESH_MAX__DEFAULT)
        self.get_grayscale = kwargs.get(
            'get_grayscale', GetGlobalPointCloud_Request.GET_GRAYSCALE__DEFAULT)

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
        if self.global_voxel_resolution != other.global_voxel_resolution:
            return False
        if self.local_voxel_resolution != other.local_voxel_resolution:
            return False
        if self.z_thresh_max != other.z_thresh_max:
            return False
        if self.get_grayscale != other.get_grayscale:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def global_voxel_resolution(self):
        """Message field 'global_voxel_resolution'."""
        return self._global_voxel_resolution

    @global_voxel_resolution.setter
    def global_voxel_resolution(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'global_voxel_resolution' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'global_voxel_resolution' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._global_voxel_resolution = value

    @builtins.property
    def local_voxel_resolution(self):
        """Message field 'local_voxel_resolution'."""
        return self._local_voxel_resolution

    @local_voxel_resolution.setter
    def local_voxel_resolution(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'local_voxel_resolution' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'local_voxel_resolution' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._local_voxel_resolution = value

    @builtins.property
    def z_thresh_max(self):
        """Message field 'z_thresh_max'."""
        return self._z_thresh_max

    @z_thresh_max.setter
    def z_thresh_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_thresh_max' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_thresh_max' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_thresh_max = value

    @builtins.property
    def get_grayscale(self):
        """Message field 'get_grayscale'."""
        return self._get_grayscale

    @get_grayscale.setter
    def get_grayscale(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'get_grayscale' field must be of type 'bool'"
        self._get_grayscale = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetGlobalPointCloud_Response(type):
    """Metaclass of message 'GetGlobalPointCloud_Response'."""

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
                'slam_msgs.srv.GetGlobalPointCloud_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_global_point_cloud__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_global_point_cloud__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_global_point_cloud__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_global_point_cloud__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_global_point_cloud__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetGlobalPointCloud_Response(metaclass=Metaclass_GetGlobalPointCloud_Response):
    """Message class 'GetGlobalPointCloud_Response'."""

    __slots__ = [
        '_response',
    ]

    _fields_and_field_types = {
        'response': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.response = kwargs.get('response', bool())

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
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'response' field must be of type 'bool'"
        self._response = value


class Metaclass_GetGlobalPointCloud(type):
    """Metaclass of service 'GetGlobalPointCloud'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('slam_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'slam_msgs.srv.GetGlobalPointCloud')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_global_point_cloud

            from slam_msgs.srv import _get_global_point_cloud
            if _get_global_point_cloud.Metaclass_GetGlobalPointCloud_Request._TYPE_SUPPORT is None:
                _get_global_point_cloud.Metaclass_GetGlobalPointCloud_Request.__import_type_support__()
            if _get_global_point_cloud.Metaclass_GetGlobalPointCloud_Response._TYPE_SUPPORT is None:
                _get_global_point_cloud.Metaclass_GetGlobalPointCloud_Response.__import_type_support__()


class GetGlobalPointCloud(metaclass=Metaclass_GetGlobalPointCloud):
    from slam_msgs.srv._get_global_point_cloud import GetGlobalPointCloud_Request as Request
    from slam_msgs.srv._get_global_point_cloud import GetGlobalPointCloud_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
