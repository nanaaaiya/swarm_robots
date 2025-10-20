# generated from rosidl_generator_py/resource/_idl.py.em
# with input from communication_msgs:msg/Task.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Task(type):
    """Metaclass of message 'Task'."""

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
                'communication_msgs.msg.Task')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__task
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__task
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__task
            cls._TYPE_SUPPORT = module.type_support_msg__msg__task
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__task

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Task(metaclass=Metaclass_Task):
    """Message class 'Task'."""

    __slots__ = [
        '_task_id',
        '_pickup_x',
        '_pickup_y',
        '_dropoff_x',
        '_dropoff_y',
        '_available',
    ]

    _fields_and_field_types = {
        'task_id': 'int32',
        'pickup_x': 'float',
        'pickup_y': 'float',
        'dropoff_x': 'float',
        'dropoff_y': 'float',
        'available': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.task_id = kwargs.get('task_id', int())
        self.pickup_x = kwargs.get('pickup_x', float())
        self.pickup_y = kwargs.get('pickup_y', float())
        self.dropoff_x = kwargs.get('dropoff_x', float())
        self.dropoff_y = kwargs.get('dropoff_y', float())
        self.available = kwargs.get('available', bool())

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
        if self.task_id != other.task_id:
            return False
        if self.pickup_x != other.pickup_x:
            return False
        if self.pickup_y != other.pickup_y:
            return False
        if self.dropoff_x != other.dropoff_x:
            return False
        if self.dropoff_y != other.dropoff_y:
            return False
        if self.available != other.available:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def task_id(self):
        """Message field 'task_id'."""
        return self._task_id

    @task_id.setter
    def task_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'task_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'task_id' field must be an integer in [-2147483648, 2147483647]"
        self._task_id = value

    @builtins.property
    def pickup_x(self):
        """Message field 'pickup_x'."""
        return self._pickup_x

    @pickup_x.setter
    def pickup_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pickup_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pickup_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pickup_x = value

    @builtins.property
    def pickup_y(self):
        """Message field 'pickup_y'."""
        return self._pickup_y

    @pickup_y.setter
    def pickup_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pickup_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pickup_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pickup_y = value

    @builtins.property
    def dropoff_x(self):
        """Message field 'dropoff_x'."""
        return self._dropoff_x

    @dropoff_x.setter
    def dropoff_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dropoff_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'dropoff_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._dropoff_x = value

    @builtins.property
    def dropoff_y(self):
        """Message field 'dropoff_y'."""
        return self._dropoff_y

    @dropoff_y.setter
    def dropoff_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dropoff_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'dropoff_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._dropoff_y = value

    @builtins.property
    def available(self):
        """Message field 'available'."""
        return self._available

    @available.setter
    def available(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'available' field must be of type 'bool'"
        self._available = value
