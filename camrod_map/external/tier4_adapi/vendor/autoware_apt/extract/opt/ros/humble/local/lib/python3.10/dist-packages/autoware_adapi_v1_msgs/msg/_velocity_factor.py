# generated from rosidl_generator_py/resource/_idl.py.em
# with input from autoware_adapi_v1_msgs:msg/VelocityFactor.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VelocityFactor(type):
    """Metaclass of message 'VelocityFactor'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'UNKNOWN': 0,
        'APPROACHING': 1,
        'STOPPED': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('autoware_adapi_v1_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'autoware_adapi_v1_msgs.msg.VelocityFactor')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__velocity_factor
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__velocity_factor
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__velocity_factor
            cls._TYPE_SUPPORT = module.type_support_msg__msg__velocity_factor
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__velocity_factor

            from autoware_adapi_v1_msgs.msg import CooperationStatus
            if CooperationStatus.__class__._TYPE_SUPPORT is None:
                CooperationStatus.__class__.__import_type_support__()

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'UNKNOWN': cls.__constants['UNKNOWN'],
            'APPROACHING': cls.__constants['APPROACHING'],
            'STOPPED': cls.__constants['STOPPED'],
        }

    @property
    def UNKNOWN(self):
        """Message constant 'UNKNOWN'."""
        return Metaclass_VelocityFactor.__constants['UNKNOWN']

    @property
    def APPROACHING(self):
        """Message constant 'APPROACHING'."""
        return Metaclass_VelocityFactor.__constants['APPROACHING']

    @property
    def STOPPED(self):
        """Message constant 'STOPPED'."""
        return Metaclass_VelocityFactor.__constants['STOPPED']


class VelocityFactor(metaclass=Metaclass_VelocityFactor):
    """
    Message class 'VelocityFactor'.

    Constants:
      UNKNOWN
      APPROACHING
      STOPPED
    """

    __slots__ = [
        '_pose',
        '_distance',
        '_status',
        '_behavior',
        '_sequence',
        '_detail',
        '_cooperation',
    ]

    _fields_and_field_types = {
        'pose': 'geometry_msgs/Pose',
        'distance': 'float',
        'status': 'uint16',
        'behavior': 'string',
        'sequence': 'string',
        'detail': 'string',
        'cooperation': 'sequence<autoware_adapi_v1_msgs/CooperationStatus, 1>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['autoware_adapi_v1_msgs', 'msg'], 'CooperationStatus'), 1),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Pose
        self.pose = kwargs.get('pose', Pose())
        self.distance = kwargs.get('distance', float())
        self.status = kwargs.get('status', int())
        self.behavior = kwargs.get('behavior', str())
        self.sequence = kwargs.get('sequence', str())
        self.detail = kwargs.get('detail', str())
        self.cooperation = kwargs.get('cooperation', [])

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
        if self.pose != other.pose:
            return False
        if self.distance != other.distance:
            return False
        if self.status != other.status:
            return False
        if self.behavior != other.behavior:
            return False
        if self.sequence != other.sequence:
            return False
        if self.detail != other.detail:
            return False
        if self.cooperation != other.cooperation:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'pose' field must be a sub message of type 'Pose'"
        self._pose = value

    @builtins.property
    def distance(self):
        """Message field 'distance'."""
        return self._distance

    @distance.setter
    def distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'status' field must be an unsigned integer in [0, 65535]"
        self._status = value

    @builtins.property
    def behavior(self):
        """Message field 'behavior'."""
        return self._behavior

    @behavior.setter
    def behavior(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'behavior' field must be of type 'str'"
        self._behavior = value

    @builtins.property
    def sequence(self):
        """Message field 'sequence'."""
        return self._sequence

    @sequence.setter
    def sequence(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'sequence' field must be of type 'str'"
        self._sequence = value

    @builtins.property
    def detail(self):
        """Message field 'detail'."""
        return self._detail

    @detail.setter
    def detail(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'detail' field must be of type 'str'"
        self._detail = value

    @builtins.property
    def cooperation(self):
        """Message field 'cooperation'."""
        return self._cooperation

    @cooperation.setter
    def cooperation(self, value):
        if __debug__:
            from autoware_adapi_v1_msgs.msg import CooperationStatus
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
                 len(value) <= 1 and
                 all(isinstance(v, CooperationStatus) for v in value) and
                 True), \
                "The 'cooperation' field must be a set or sequence with length <= 1 and each value of type 'CooperationStatus'"
        self._cooperation = value
