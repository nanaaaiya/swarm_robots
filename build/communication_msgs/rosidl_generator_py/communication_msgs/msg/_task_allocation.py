# generated from rosidl_generator_py/resource/_idl.py.em
# with input from communication_msgs:msg/TaskAllocation.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'task_ids'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TaskAllocation(type):
    """Metaclass of message 'TaskAllocation'."""

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
            module = import_type_support('communication_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'communication_msgs.msg.TaskAllocation')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__task_allocation
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__task_allocation
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__task_allocation
            cls._TYPE_SUPPORT = module.type_support_msg__msg__task_allocation
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__task_allocation

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TaskAllocation(metaclass=Metaclass_TaskAllocation):
    """Message class 'TaskAllocation'."""

    __slots__ = [
        '_robot_id',
        '_task_ids',
        '_total_distance',
    ]

    _fields_and_field_types = {
        'robot_id': 'int32',
        'task_ids': 'sequence<int32>',
        'total_distance': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robot_id = kwargs.get('robot_id', int())
        self.task_ids = array.array('i', kwargs.get('task_ids', []))
        self.total_distance = kwargs.get('total_distance', float())

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
        if self.robot_id != other.robot_id:
            return False
        if self.task_ids != other.task_ids:
            return False
        if self.total_distance != other.total_distance:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def robot_id(self):
        """Message field 'robot_id'."""
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robot_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'robot_id' field must be an integer in [-2147483648, 2147483647]"
        self._robot_id = value

    @builtins.property
    def task_ids(self):
        """Message field 'task_ids'."""
        return self._task_ids

    @task_ids.setter
    def task_ids(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'task_ids' array.array() must have the type code of 'i'"
            self._task_ids = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'task_ids' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._task_ids = array.array('i', value)

    @builtins.property
    def total_distance(self):
        """Message field 'total_distance'."""
        return self._total_distance

    @total_distance.setter
    def total_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'total_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._total_distance = value
