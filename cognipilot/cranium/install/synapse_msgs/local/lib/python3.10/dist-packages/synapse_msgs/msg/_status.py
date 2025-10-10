# generated from rosidl_generator_py/resource/_idl.py.em
# with input from synapse_msgs:msg/Status.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Status(type):
    """Metaclass of message 'Status'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ARMING_UNKNOWN': 0,
        'ARMING_DISARMED': 1,
        'ARMING_ARMED': 2,
        'FUEL_UNKNOWN': 0,
        'FUEL_CRITICAL': 1,
        'FUEL_LOW': 2,
        'FUEL_NOMINAL': 3,
        'FUEL_OVER_CAPACITY': 4,
        'JOY_UNKNOWN': 0,
        'JOY_DISABLED': 1,
        'JOY_LOSS': 2,
        'JOY_NOMINAL': 3,
        'MODE_UNKNOWN': 0,
        'MODE_MANUAL': 1,
        'MODE_AUTO': 2,
        'MODE_CMD_VEL': 3,
        'MODE_CALIBRATION': 4,
        'SAFETY_UNKNOWN': 0,
        'SAFETY_SAFE': 1,
        'SAFETY_UNSAFE': 2,
        'SYNAPSE_UNKNOWN': 0,
        'SYNAPSE_DISABLED': 1,
        'SYNAPSE_LOSS': 2,
        'SYNAPSE_NOMINAL': 3,
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
                'synapse_msgs.msg.Status')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ARMING_UNKNOWN': cls.__constants['ARMING_UNKNOWN'],
            'ARMING_DISARMED': cls.__constants['ARMING_DISARMED'],
            'ARMING_ARMED': cls.__constants['ARMING_ARMED'],
            'FUEL_UNKNOWN': cls.__constants['FUEL_UNKNOWN'],
            'FUEL_CRITICAL': cls.__constants['FUEL_CRITICAL'],
            'FUEL_LOW': cls.__constants['FUEL_LOW'],
            'FUEL_NOMINAL': cls.__constants['FUEL_NOMINAL'],
            'FUEL_OVER_CAPACITY': cls.__constants['FUEL_OVER_CAPACITY'],
            'JOY_UNKNOWN': cls.__constants['JOY_UNKNOWN'],
            'JOY_DISABLED': cls.__constants['JOY_DISABLED'],
            'JOY_LOSS': cls.__constants['JOY_LOSS'],
            'JOY_NOMINAL': cls.__constants['JOY_NOMINAL'],
            'MODE_UNKNOWN': cls.__constants['MODE_UNKNOWN'],
            'MODE_MANUAL': cls.__constants['MODE_MANUAL'],
            'MODE_AUTO': cls.__constants['MODE_AUTO'],
            'MODE_CMD_VEL': cls.__constants['MODE_CMD_VEL'],
            'MODE_CALIBRATION': cls.__constants['MODE_CALIBRATION'],
            'SAFETY_UNKNOWN': cls.__constants['SAFETY_UNKNOWN'],
            'SAFETY_SAFE': cls.__constants['SAFETY_SAFE'],
            'SAFETY_UNSAFE': cls.__constants['SAFETY_UNSAFE'],
            'SYNAPSE_UNKNOWN': cls.__constants['SYNAPSE_UNKNOWN'],
            'SYNAPSE_DISABLED': cls.__constants['SYNAPSE_DISABLED'],
            'SYNAPSE_LOSS': cls.__constants['SYNAPSE_LOSS'],
            'SYNAPSE_NOMINAL': cls.__constants['SYNAPSE_NOMINAL'],
        }

    @property
    def ARMING_UNKNOWN(self):
        """Message constant 'ARMING_UNKNOWN'."""
        return Metaclass_Status.__constants['ARMING_UNKNOWN']

    @property
    def ARMING_DISARMED(self):
        """Message constant 'ARMING_DISARMED'."""
        return Metaclass_Status.__constants['ARMING_DISARMED']

    @property
    def ARMING_ARMED(self):
        """Message constant 'ARMING_ARMED'."""
        return Metaclass_Status.__constants['ARMING_ARMED']

    @property
    def FUEL_UNKNOWN(self):
        """Message constant 'FUEL_UNKNOWN'."""
        return Metaclass_Status.__constants['FUEL_UNKNOWN']

    @property
    def FUEL_CRITICAL(self):
        """Message constant 'FUEL_CRITICAL'."""
        return Metaclass_Status.__constants['FUEL_CRITICAL']

    @property
    def FUEL_LOW(self):
        """Message constant 'FUEL_LOW'."""
        return Metaclass_Status.__constants['FUEL_LOW']

    @property
    def FUEL_NOMINAL(self):
        """Message constant 'FUEL_NOMINAL'."""
        return Metaclass_Status.__constants['FUEL_NOMINAL']

    @property
    def FUEL_OVER_CAPACITY(self):
        """Message constant 'FUEL_OVER_CAPACITY'."""
        return Metaclass_Status.__constants['FUEL_OVER_CAPACITY']

    @property
    def JOY_UNKNOWN(self):
        """Message constant 'JOY_UNKNOWN'."""
        return Metaclass_Status.__constants['JOY_UNKNOWN']

    @property
    def JOY_DISABLED(self):
        """Message constant 'JOY_DISABLED'."""
        return Metaclass_Status.__constants['JOY_DISABLED']

    @property
    def JOY_LOSS(self):
        """Message constant 'JOY_LOSS'."""
        return Metaclass_Status.__constants['JOY_LOSS']

    @property
    def JOY_NOMINAL(self):
        """Message constant 'JOY_NOMINAL'."""
        return Metaclass_Status.__constants['JOY_NOMINAL']

    @property
    def MODE_UNKNOWN(self):
        """Message constant 'MODE_UNKNOWN'."""
        return Metaclass_Status.__constants['MODE_UNKNOWN']

    @property
    def MODE_MANUAL(self):
        """Message constant 'MODE_MANUAL'."""
        return Metaclass_Status.__constants['MODE_MANUAL']

    @property
    def MODE_AUTO(self):
        """Message constant 'MODE_AUTO'."""
        return Metaclass_Status.__constants['MODE_AUTO']

    @property
    def MODE_CMD_VEL(self):
        """Message constant 'MODE_CMD_VEL'."""
        return Metaclass_Status.__constants['MODE_CMD_VEL']

    @property
    def MODE_CALIBRATION(self):
        """Message constant 'MODE_CALIBRATION'."""
        return Metaclass_Status.__constants['MODE_CALIBRATION']

    @property
    def SAFETY_UNKNOWN(self):
        """Message constant 'SAFETY_UNKNOWN'."""
        return Metaclass_Status.__constants['SAFETY_UNKNOWN']

    @property
    def SAFETY_SAFE(self):
        """Message constant 'SAFETY_SAFE'."""
        return Metaclass_Status.__constants['SAFETY_SAFE']

    @property
    def SAFETY_UNSAFE(self):
        """Message constant 'SAFETY_UNSAFE'."""
        return Metaclass_Status.__constants['SAFETY_UNSAFE']

    @property
    def SYNAPSE_UNKNOWN(self):
        """Message constant 'SYNAPSE_UNKNOWN'."""
        return Metaclass_Status.__constants['SYNAPSE_UNKNOWN']

    @property
    def SYNAPSE_DISABLED(self):
        """Message constant 'SYNAPSE_DISABLED'."""
        return Metaclass_Status.__constants['SYNAPSE_DISABLED']

    @property
    def SYNAPSE_LOSS(self):
        """Message constant 'SYNAPSE_LOSS'."""
        return Metaclass_Status.__constants['SYNAPSE_LOSS']

    @property
    def SYNAPSE_NOMINAL(self):
        """Message constant 'SYNAPSE_NOMINAL'."""
        return Metaclass_Status.__constants['SYNAPSE_NOMINAL']


class Status(metaclass=Metaclass_Status):
    """
    Message class 'Status'.

    Constants:
      ARMING_UNKNOWN
      ARMING_DISARMED
      ARMING_ARMED
      FUEL_UNKNOWN
      FUEL_CRITICAL
      FUEL_LOW
      FUEL_NOMINAL
      FUEL_OVER_CAPACITY
      JOY_UNKNOWN
      JOY_DISABLED
      JOY_LOSS
      JOY_NOMINAL
      MODE_UNKNOWN
      MODE_MANUAL
      MODE_AUTO
      MODE_CMD_VEL
      MODE_CALIBRATION
      SAFETY_UNKNOWN
      SAFETY_SAFE
      SAFETY_UNSAFE
      SYNAPSE_UNKNOWN
      SYNAPSE_DISABLED
      SYNAPSE_LOSS
      SYNAPSE_NOMINAL
    """

    __slots__ = [
        '_header',
        '_arming',
        '_joy',
        '_fuel',
        '_mode',
        '_safety',
        '_synapse',
        '_status_message',
        '_fuel_percentage',
        '_power',
        '_request_seq',
        '_request_rejected',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'arming': 'uint8',
        'joy': 'uint8',
        'fuel': 'uint8',
        'mode': 'uint8',
        'safety': 'uint8',
        'synapse': 'uint8',
        'status_message': 'string',
        'fuel_percentage': 'uint8',
        'power': 'float',
        'request_seq': 'uint32',
        'request_rejected': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.arming = kwargs.get('arming', int())
        self.joy = kwargs.get('joy', int())
        self.fuel = kwargs.get('fuel', int())
        self.mode = kwargs.get('mode', int())
        self.safety = kwargs.get('safety', int())
        self.synapse = kwargs.get('synapse', int())
        self.status_message = kwargs.get('status_message', str())
        self.fuel_percentage = kwargs.get('fuel_percentage', int())
        self.power = kwargs.get('power', float())
        self.request_seq = kwargs.get('request_seq', int())
        self.request_rejected = kwargs.get('request_rejected', bool())

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
        if self.header != other.header:
            return False
        if self.arming != other.arming:
            return False
        if self.joy != other.joy:
            return False
        if self.fuel != other.fuel:
            return False
        if self.mode != other.mode:
            return False
        if self.safety != other.safety:
            return False
        if self.synapse != other.synapse:
            return False
        if self.status_message != other.status_message:
            return False
        if self.fuel_percentage != other.fuel_percentage:
            return False
        if self.power != other.power:
            return False
        if self.request_seq != other.request_seq:
            return False
        if self.request_rejected != other.request_rejected:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def arming(self):
        """Message field 'arming'."""
        return self._arming

    @arming.setter
    def arming(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'arming' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'arming' field must be an unsigned integer in [0, 255]"
        self._arming = value

    @builtins.property
    def joy(self):
        """Message field 'joy'."""
        return self._joy

    @joy.setter
    def joy(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'joy' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'joy' field must be an unsigned integer in [0, 255]"
        self._joy = value

    @builtins.property
    def fuel(self):
        """Message field 'fuel'."""
        return self._fuel

    @fuel.setter
    def fuel(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fuel' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'fuel' field must be an unsigned integer in [0, 255]"
        self._fuel = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value

    @builtins.property
    def safety(self):
        """Message field 'safety'."""
        return self._safety

    @safety.setter
    def safety(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'safety' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'safety' field must be an unsigned integer in [0, 255]"
        self._safety = value

    @builtins.property
    def synapse(self):
        """Message field 'synapse'."""
        return self._synapse

    @synapse.setter
    def synapse(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'synapse' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'synapse' field must be an unsigned integer in [0, 255]"
        self._synapse = value

    @builtins.property
    def status_message(self):
        """Message field 'status_message'."""
        return self._status_message

    @status_message.setter
    def status_message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'status_message' field must be of type 'str'"
        self._status_message = value

    @builtins.property
    def fuel_percentage(self):
        """Message field 'fuel_percentage'."""
        return self._fuel_percentage

    @fuel_percentage.setter
    def fuel_percentage(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fuel_percentage' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'fuel_percentage' field must be an unsigned integer in [0, 255]"
        self._fuel_percentage = value

    @builtins.property
    def power(self):
        """Message field 'power'."""
        return self._power

    @power.setter
    def power(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'power' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'power' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._power = value

    @builtins.property
    def request_seq(self):
        """Message field 'request_seq'."""
        return self._request_seq

    @request_seq.setter
    def request_seq(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'request_seq' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'request_seq' field must be an unsigned integer in [0, 4294967295]"
        self._request_seq = value

    @builtins.property
    def request_rejected(self):
        """Message field 'request_rejected'."""
        return self._request_rejected

    @request_rejected.setter
    def request_rejected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'request_rejected' field must be of type 'bool'"
        self._request_rejected = value
