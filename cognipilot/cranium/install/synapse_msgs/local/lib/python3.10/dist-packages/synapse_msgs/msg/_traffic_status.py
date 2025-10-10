# generated from rosidl_generator_py/resource/_idl.py.em
# with input from synapse_msgs:msg/TrafficStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TrafficStatus(type):
    """Metaclass of message 'TrafficStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'TRAFFIC_LIGHT': 0,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('synapse_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'synapse_msgs.msg.TrafficStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__traffic_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__traffic_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__traffic_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__traffic_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__traffic_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'TRAFFIC_LIGHT': cls.__constants['TRAFFIC_LIGHT'],
        }

    @property
    def TRAFFIC_LIGHT(self):
        """Message constant 'TRAFFIC_LIGHT'."""
        return Metaclass_TrafficStatus.__constants['TRAFFIC_LIGHT']


class TrafficStatus(metaclass=Metaclass_TrafficStatus):
    """
    Message class 'TrafficStatus'.

    Constants:
      TRAFFIC_LIGHT
    """

    __slots__ = [
        '_stop_sign',
        '_left_sign',
        '_straight_sign',
        '_right_sign',
    ]

    _fields_and_field_types = {
        'stop_sign': 'boolean',
        'left_sign': 'boolean',
        'straight_sign': 'boolean',
        'right_sign': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.stop_sign = kwargs.get('stop_sign', bool())
        self.left_sign = kwargs.get('left_sign', bool())
        self.straight_sign = kwargs.get('straight_sign', bool())
        self.right_sign = kwargs.get('right_sign', bool())

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
        if self.stop_sign != other.stop_sign:
            return False
        if self.left_sign != other.left_sign:
            return False
        if self.straight_sign != other.straight_sign:
            return False
        if self.right_sign != other.right_sign:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def stop_sign(self):
        """Message field 'stop_sign'."""
        return self._stop_sign

    @stop_sign.setter
    def stop_sign(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'stop_sign' field must be of type 'bool'"
        self._stop_sign = value

    @builtins.property
    def left_sign(self):
        """Message field 'left_sign'."""
        return self._left_sign

    @left_sign.setter
    def left_sign(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'left_sign' field must be of type 'bool'"
        self._left_sign = value

    @builtins.property
    def straight_sign(self):
        """Message field 'straight_sign'."""
        return self._straight_sign

    @straight_sign.setter
    def straight_sign(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'straight_sign' field must be of type 'bool'"
        self._straight_sign = value

    @builtins.property
    def right_sign(self):
        """Message field 'right_sign'."""
        return self._right_sign

    @right_sign.setter
    def right_sign(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'right_sign' field must be of type 'bool'"
        self._right_sign = value
