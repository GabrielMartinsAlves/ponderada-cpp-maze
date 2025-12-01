# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cg_interfaces:srv/Reset.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Reset_Request(type):
    """Metaclass of message 'Reset_Request'."""

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
                'cg_interfaces.srv.Reset_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__reset__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__reset__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__reset__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__reset__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__reset__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Reset_Request(metaclass=Metaclass_Reset_Request):
    """Message class 'Reset_Request'."""

    __slots__ = [
        '_is_random',
        '_map_name',
    ]

    _fields_and_field_types = {
        'is_random': 'boolean',
        'map_name': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.is_random = kwargs.get('is_random', bool())
        self.map_name = kwargs.get('map_name', str())

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
        if self.is_random != other.is_random:
            return False
        if self.map_name != other.map_name:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def is_random(self):
        """Message field 'is_random'."""
        return self._is_random

    @is_random.setter
    def is_random(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_random' field must be of type 'bool'"
        self._is_random = value

    @builtins.property
    def map_name(self):
        """Message field 'map_name'."""
        return self._map_name

    @map_name.setter
    def map_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'map_name' field must be of type 'str'"
        self._map_name = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Reset_Response(type):
    """Metaclass of message 'Reset_Response'."""

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
                'cg_interfaces.srv.Reset_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__reset__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__reset__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__reset__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__reset__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__reset__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Reset_Response(metaclass=Metaclass_Reset_Response):
    """Message class 'Reset_Response'."""

    __slots__ = [
        '_success',
        '_loaded_map_name',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'loaded_map_name': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.loaded_map_name = kwargs.get('loaded_map_name', str())

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
        if self.loaded_map_name != other.loaded_map_name:
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

    @builtins.property
    def loaded_map_name(self):
        """Message field 'loaded_map_name'."""
        return self._loaded_map_name

    @loaded_map_name.setter
    def loaded_map_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'loaded_map_name' field must be of type 'str'"
        self._loaded_map_name = value


class Metaclass_Reset(type):
    """Metaclass of service 'Reset'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('cg_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cg_interfaces.srv.Reset')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__reset

            from cg_interfaces.srv import _reset
            if _reset.Metaclass_Reset_Request._TYPE_SUPPORT is None:
                _reset.Metaclass_Reset_Request.__import_type_support__()
            if _reset.Metaclass_Reset_Response._TYPE_SUPPORT is None:
                _reset.Metaclass_Reset_Response.__import_type_support__()


class Reset(metaclass=Metaclass_Reset):
    from cg_interfaces.srv._reset import Reset_Request as Request
    from cg_interfaces.srv._reset import Reset_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
