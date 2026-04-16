# generated from rosidl_generator_py/resource/_idl.py.em
# with input from autoware_adapi_v1_msgs:msg/DiagGraphStruct.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DiagGraphStruct(type):
    """Metaclass of message 'DiagGraphStruct'."""

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
            module = import_type_support('autoware_adapi_v1_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'autoware_adapi_v1_msgs.msg.DiagGraphStruct')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__diag_graph_struct
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__diag_graph_struct
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__diag_graph_struct
            cls._TYPE_SUPPORT = module.type_support_msg__msg__diag_graph_struct
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__diag_graph_struct

            from autoware_adapi_v1_msgs.msg import DiagLeafStruct
            if DiagLeafStruct.__class__._TYPE_SUPPORT is None:
                DiagLeafStruct.__class__.__import_type_support__()

            from autoware_adapi_v1_msgs.msg import DiagLinkStruct
            if DiagLinkStruct.__class__._TYPE_SUPPORT is None:
                DiagLinkStruct.__class__.__import_type_support__()

            from autoware_adapi_v1_msgs.msg import DiagNodeStruct
            if DiagNodeStruct.__class__._TYPE_SUPPORT is None:
                DiagNodeStruct.__class__.__import_type_support__()

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DiagGraphStruct(metaclass=Metaclass_DiagGraphStruct):
    """Message class 'DiagGraphStruct'."""

    __slots__ = [
        '_stamp',
        '_id',
        '_nodes',
        '_diags',
        '_links',
    ]

    _fields_and_field_types = {
        'stamp': 'builtin_interfaces/Time',
        'id': 'string',
        'nodes': 'sequence<autoware_adapi_v1_msgs/DiagNodeStruct>',
        'diags': 'sequence<autoware_adapi_v1_msgs/DiagLeafStruct>',
        'links': 'sequence<autoware_adapi_v1_msgs/DiagLinkStruct>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['autoware_adapi_v1_msgs', 'msg'], 'DiagNodeStruct')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['autoware_adapi_v1_msgs', 'msg'], 'DiagLeafStruct')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['autoware_adapi_v1_msgs', 'msg'], 'DiagLinkStruct')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())
        self.id = kwargs.get('id', str())
        self.nodes = kwargs.get('nodes', [])
        self.diags = kwargs.get('diags', [])
        self.links = kwargs.get('links', [])

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
        if self.stamp != other.stamp:
            return False
        if self.id != other.id:
            return False
        if self.nodes != other.nodes:
            return False
        if self.diags != other.diags:
            return False
        if self.links != other.links:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'id' field must be of type 'str'"
        self._id = value

    @builtins.property
    def nodes(self):
        """Message field 'nodes'."""
        return self._nodes

    @nodes.setter
    def nodes(self, value):
        if __debug__:
            from autoware_adapi_v1_msgs.msg import DiagNodeStruct
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
                 all(isinstance(v, DiagNodeStruct) for v in value) and
                 True), \
                "The 'nodes' field must be a set or sequence and each value of type 'DiagNodeStruct'"
        self._nodes = value

    @builtins.property
    def diags(self):
        """Message field 'diags'."""
        return self._diags

    @diags.setter
    def diags(self, value):
        if __debug__:
            from autoware_adapi_v1_msgs.msg import DiagLeafStruct
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
                 all(isinstance(v, DiagLeafStruct) for v in value) and
                 True), \
                "The 'diags' field must be a set or sequence and each value of type 'DiagLeafStruct'"
        self._diags = value

    @builtins.property
    def links(self):
        """Message field 'links'."""
        return self._links

    @links.setter
    def links(self, value):
        if __debug__:
            from autoware_adapi_v1_msgs.msg import DiagLinkStruct
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
                 all(isinstance(v, DiagLinkStruct) for v in value) and
                 True), \
                "The 'links' field must be a set or sequence and each value of type 'DiagLinkStruct'"
        self._links = value
