# generated from rosidl_generator_py/resource/_idl.py.em
# with input from synapse_msgs:msg/EdgeVectors.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EdgeVectors(type):
    """Metaclass of message 'EdgeVectors'."""

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
            module = import_type_support('synapse_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'synapse_msgs.msg.EdgeVectors')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__edge_vectors
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__edge_vectors
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__edge_vectors
            cls._TYPE_SUPPORT = module.type_support_msg__msg__edge_vectors
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__edge_vectors

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class EdgeVectors(metaclass=Metaclass_EdgeVectors):
    """Message class 'EdgeVectors'."""

    __slots__ = [
        '_vector_1',
        '_vector_2',
        '_vector_count',
        '_image_height',
        '_image_width',
    ]

    _fields_and_field_types = {
        'vector_1': 'geometry_msgs/Point[2]',
        'vector_2': 'geometry_msgs/Point[2]',
        'vector_count': 'uint8',
        'image_height': 'uint16',
        'image_width': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'), 2),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'), 2),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Point
        self.vector_1 = kwargs.get(
            'vector_1',
            [Point() for x in range(2)]
        )
        from geometry_msgs.msg import Point
        self.vector_2 = kwargs.get(
            'vector_2',
            [Point() for x in range(2)]
        )
        self.vector_count = kwargs.get('vector_count', int())
        self.image_height = kwargs.get('image_height', int())
        self.image_width = kwargs.get('image_width', int())

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
        if self.vector_1 != other.vector_1:
            return False
        if self.vector_2 != other.vector_2:
            return False
        if self.vector_count != other.vector_count:
            return False
        if self.image_height != other.image_height:
            return False
        if self.image_width != other.image_width:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def vector_1(self):
        """Message field 'vector_1'."""
        return self._vector_1

    @vector_1.setter
    def vector_1(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
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
                 len(value) == 2 and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'vector_1' field must be a set or sequence with length 2 and each value of type 'Point'"
        self._vector_1 = value

    @builtins.property
    def vector_2(self):
        """Message field 'vector_2'."""
        return self._vector_2

    @vector_2.setter
    def vector_2(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
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
                 len(value) == 2 and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'vector_2' field must be a set or sequence with length 2 and each value of type 'Point'"
        self._vector_2 = value

    @builtins.property
    def vector_count(self):
        """Message field 'vector_count'."""
        return self._vector_count

    @vector_count.setter
    def vector_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vector_count' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'vector_count' field must be an unsigned integer in [0, 255]"
        self._vector_count = value

    @builtins.property
    def image_height(self):
        """Message field 'image_height'."""
        return self._image_height

    @image_height.setter
    def image_height(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'image_height' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'image_height' field must be an unsigned integer in [0, 65535]"
        self._image_height = value

    @builtins.property
    def image_width(self):
        """Message field 'image_width'."""
        return self._image_width

    @image_width.setter
    def image_width(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'image_width' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'image_width' field must be an unsigned integer in [0, 65535]"
        self._image_width = value
