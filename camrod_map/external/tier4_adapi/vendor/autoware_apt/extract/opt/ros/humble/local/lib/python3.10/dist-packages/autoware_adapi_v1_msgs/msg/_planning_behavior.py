# generated from rosidl_generator_py/resource/_idl.py.em
# with input from autoware_adapi_v1_msgs:msg/PlanningBehavior.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PlanningBehavior(type):
    """Metaclass of message 'PlanningBehavior'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'UNKNOWN': '',
        'AVOIDANCE': 'avoidance',
        'CROSSWALK': 'crosswalk',
        'GOAL_PLANNER': 'goal-planner',
        'INTERSECTION': 'intersection',
        'LANE_CHANGE': 'lane-change',
        'MERGE': 'merge',
        'NO_DRIVABLE_LANE': 'no-drivable-lane',
        'NO_STOPPING_AREA': 'no-stopping-area',
        'REAR_CHECK': 'rear-check',
        'ROUTE_OBSTACLE': 'route-obstacle',
        'SIDEWALK': 'sidewalk',
        'START_PLANNER': 'start-planner',
        'STOP_SIGN': 'stop-sign',
        'SURROUNDING_OBSTACLE': 'surrounding-obstacle',
        'TRAFFIC_SIGNAL': 'traffic-signal',
        'USER_DEFINED_DETECTION_AREA': 'user-defined-attention-area',
        'VIRTUAL_TRAFFIC_LIGHT': 'virtual-traffic-light',
        'RUN_OUT': 'run-out',
        'ADAPTIVE_CRUISE': 'adaptive-cruise',
        'ROUNDABOUT': 'roundabout',
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
                'autoware_adapi_v1_msgs.msg.PlanningBehavior')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__planning_behavior
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__planning_behavior
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__planning_behavior
            cls._TYPE_SUPPORT = module.type_support_msg__msg__planning_behavior
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__planning_behavior

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'UNKNOWN': cls.__constants['UNKNOWN'],
            'AVOIDANCE': cls.__constants['AVOIDANCE'],
            'CROSSWALK': cls.__constants['CROSSWALK'],
            'GOAL_PLANNER': cls.__constants['GOAL_PLANNER'],
            'INTERSECTION': cls.__constants['INTERSECTION'],
            'LANE_CHANGE': cls.__constants['LANE_CHANGE'],
            'MERGE': cls.__constants['MERGE'],
            'NO_DRIVABLE_LANE': cls.__constants['NO_DRIVABLE_LANE'],
            'NO_STOPPING_AREA': cls.__constants['NO_STOPPING_AREA'],
            'REAR_CHECK': cls.__constants['REAR_CHECK'],
            'ROUTE_OBSTACLE': cls.__constants['ROUTE_OBSTACLE'],
            'SIDEWALK': cls.__constants['SIDEWALK'],
            'START_PLANNER': cls.__constants['START_PLANNER'],
            'STOP_SIGN': cls.__constants['STOP_SIGN'],
            'SURROUNDING_OBSTACLE': cls.__constants['SURROUNDING_OBSTACLE'],
            'TRAFFIC_SIGNAL': cls.__constants['TRAFFIC_SIGNAL'],
            'USER_DEFINED_DETECTION_AREA': cls.__constants['USER_DEFINED_DETECTION_AREA'],
            'VIRTUAL_TRAFFIC_LIGHT': cls.__constants['VIRTUAL_TRAFFIC_LIGHT'],
            'RUN_OUT': cls.__constants['RUN_OUT'],
            'ADAPTIVE_CRUISE': cls.__constants['ADAPTIVE_CRUISE'],
            'ROUNDABOUT': cls.__constants['ROUNDABOUT'],
        }

    @property
    def UNKNOWN(self):
        """Message constant 'UNKNOWN'."""
        return Metaclass_PlanningBehavior.__constants['UNKNOWN']

    @property
    def AVOIDANCE(self):
        """Message constant 'AVOIDANCE'."""
        return Metaclass_PlanningBehavior.__constants['AVOIDANCE']

    @property
    def CROSSWALK(self):
        """Message constant 'CROSSWALK'."""
        return Metaclass_PlanningBehavior.__constants['CROSSWALK']

    @property
    def GOAL_PLANNER(self):
        """Message constant 'GOAL_PLANNER'."""
        return Metaclass_PlanningBehavior.__constants['GOAL_PLANNER']

    @property
    def INTERSECTION(self):
        """Message constant 'INTERSECTION'."""
        return Metaclass_PlanningBehavior.__constants['INTERSECTION']

    @property
    def LANE_CHANGE(self):
        """Message constant 'LANE_CHANGE'."""
        return Metaclass_PlanningBehavior.__constants['LANE_CHANGE']

    @property
    def MERGE(self):
        """Message constant 'MERGE'."""
        return Metaclass_PlanningBehavior.__constants['MERGE']

    @property
    def NO_DRIVABLE_LANE(self):
        """Message constant 'NO_DRIVABLE_LANE'."""
        return Metaclass_PlanningBehavior.__constants['NO_DRIVABLE_LANE']

    @property
    def NO_STOPPING_AREA(self):
        """Message constant 'NO_STOPPING_AREA'."""
        return Metaclass_PlanningBehavior.__constants['NO_STOPPING_AREA']

    @property
    def REAR_CHECK(self):
        """Message constant 'REAR_CHECK'."""
        return Metaclass_PlanningBehavior.__constants['REAR_CHECK']

    @property
    def ROUTE_OBSTACLE(self):
        """Message constant 'ROUTE_OBSTACLE'."""
        return Metaclass_PlanningBehavior.__constants['ROUTE_OBSTACLE']

    @property
    def SIDEWALK(self):
        """Message constant 'SIDEWALK'."""
        return Metaclass_PlanningBehavior.__constants['SIDEWALK']

    @property
    def START_PLANNER(self):
        """Message constant 'START_PLANNER'."""
        return Metaclass_PlanningBehavior.__constants['START_PLANNER']

    @property
    def STOP_SIGN(self):
        """Message constant 'STOP_SIGN'."""
        return Metaclass_PlanningBehavior.__constants['STOP_SIGN']

    @property
    def SURROUNDING_OBSTACLE(self):
        """Message constant 'SURROUNDING_OBSTACLE'."""
        return Metaclass_PlanningBehavior.__constants['SURROUNDING_OBSTACLE']

    @property
    def TRAFFIC_SIGNAL(self):
        """Message constant 'TRAFFIC_SIGNAL'."""
        return Metaclass_PlanningBehavior.__constants['TRAFFIC_SIGNAL']

    @property
    def USER_DEFINED_DETECTION_AREA(self):
        """Message constant 'USER_DEFINED_DETECTION_AREA'."""
        return Metaclass_PlanningBehavior.__constants['USER_DEFINED_DETECTION_AREA']

    @property
    def VIRTUAL_TRAFFIC_LIGHT(self):
        """Message constant 'VIRTUAL_TRAFFIC_LIGHT'."""
        return Metaclass_PlanningBehavior.__constants['VIRTUAL_TRAFFIC_LIGHT']

    @property
    def RUN_OUT(self):
        """Message constant 'RUN_OUT'."""
        return Metaclass_PlanningBehavior.__constants['RUN_OUT']

    @property
    def ADAPTIVE_CRUISE(self):
        """Message constant 'ADAPTIVE_CRUISE'."""
        return Metaclass_PlanningBehavior.__constants['ADAPTIVE_CRUISE']

    @property
    def ROUNDABOUT(self):
        """Message constant 'ROUNDABOUT'."""
        return Metaclass_PlanningBehavior.__constants['ROUNDABOUT']


class PlanningBehavior(metaclass=Metaclass_PlanningBehavior):
    """
    Message class 'PlanningBehavior'.

    Constants:
      UNKNOWN
      AVOIDANCE
      CROSSWALK
      GOAL_PLANNER
      INTERSECTION
      LANE_CHANGE
      MERGE
      NO_DRIVABLE_LANE
      NO_STOPPING_AREA
      REAR_CHECK
      ROUTE_OBSTACLE
      SIDEWALK
      START_PLANNER
      STOP_SIGN
      SURROUNDING_OBSTACLE
      TRAFFIC_SIGNAL
      USER_DEFINED_DETECTION_AREA
      VIRTUAL_TRAFFIC_LIGHT
      RUN_OUT
      ADAPTIVE_CRUISE
      ROUNDABOUT
    """

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)
