# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cg_interfaces:msg/RobotSensors.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobotSensors(type):
    """Metaclass of message 'RobotSensors'."""

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
            module = import_type_support('cg_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cg_interfaces.msg.RobotSensors')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robot_sensors
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robot_sensors
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robot_sensors
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robot_sensors
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robot_sensors

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RobotSensors(metaclass=Metaclass_RobotSensors):
    """Message class 'RobotSensors'."""

    __slots__ = [
        '_up',
        '_down',
        '_left',
        '_right',
        '_up_left',
        '_up_right',
        '_down_left',
        '_down_right',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'up': 'string',
        'down': 'string',
        'left': 'string',
        'right': 'string',
        'up_left': 'string',
        'up_right': 'string',
        'down_left': 'string',
        'down_right': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.up = kwargs.get('up', str())
        self.down = kwargs.get('down', str())
        self.left = kwargs.get('left', str())
        self.right = kwargs.get('right', str())
        self.up_left = kwargs.get('up_left', str())
        self.up_right = kwargs.get('up_right', str())
        self.down_left = kwargs.get('down_left', str())
        self.down_right = kwargs.get('down_right', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.up != other.up:
            return False
        if self.down != other.down:
            return False
        if self.left != other.left:
            return False
        if self.right != other.right:
            return False
        if self.up_left != other.up_left:
            return False
        if self.up_right != other.up_right:
            return False
        if self.down_left != other.down_left:
            return False
        if self.down_right != other.down_right:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def up(self):
        """Message field 'up'."""
        return self._up

    @up.setter
    def up(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'up' field must be of type 'str'"
        self._up = value

    @builtins.property
    def down(self):
        """Message field 'down'."""
        return self._down

    @down.setter
    def down(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'down' field must be of type 'str'"
        self._down = value

    @builtins.property
    def left(self):
        """Message field 'left'."""
        return self._left

    @left.setter
    def left(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'left' field must be of type 'str'"
        self._left = value

    @builtins.property
    def right(self):
        """Message field 'right'."""
        return self._right

    @right.setter
    def right(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'right' field must be of type 'str'"
        self._right = value

    @builtins.property
    def up_left(self):
        """Message field 'up_left'."""
        return self._up_left

    @up_left.setter
    def up_left(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'up_left' field must be of type 'str'"
        self._up_left = value

    @builtins.property
    def up_right(self):
        """Message field 'up_right'."""
        return self._up_right

    @up_right.setter
    def up_right(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'up_right' field must be of type 'str'"
        self._up_right = value

    @builtins.property
    def down_left(self):
        """Message field 'down_left'."""
        return self._down_left

    @down_left.setter
    def down_left(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'down_left' field must be of type 'str'"
        self._down_left = value

    @builtins.property
    def down_right(self):
        """Message field 'down_right'."""
        return self._down_right

    @down_right.setter
    def down_right(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'down_right' field must be of type 'str'"
        self._down_right = value
