# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hand_interfaces:msg/Pinch.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Pinch(type):
    """Metaclass of message 'Pinch'."""

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
            module = import_type_support('hand_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hand_interfaces.msg.Pinch')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pinch
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pinch
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pinch
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pinch
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pinch

            from hand_interfaces.msg import FingerData
            if FingerData.__class__._TYPE_SUPPORT is None:
                FingerData.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Pinch(metaclass=Metaclass_Pinch):
    """Message class 'Pinch'."""

    __slots__ = [
        '_wrist',
        '_thumb',
        '_index',
        '_middle',
        '_ring',
        '_pinky',
    ]

    _fields_and_field_types = {
        'wrist': 'hand_interfaces/FingerData',
        'thumb': 'hand_interfaces/FingerData',
        'index': 'hand_interfaces/FingerData',
        'middle': 'hand_interfaces/FingerData',
        'ring': 'hand_interfaces/FingerData',
        'pinky': 'hand_interfaces/FingerData',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['hand_interfaces', 'msg'], 'FingerData'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['hand_interfaces', 'msg'], 'FingerData'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['hand_interfaces', 'msg'], 'FingerData'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['hand_interfaces', 'msg'], 'FingerData'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['hand_interfaces', 'msg'], 'FingerData'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['hand_interfaces', 'msg'], 'FingerData'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from hand_interfaces.msg import FingerData
        self.wrist = kwargs.get('wrist', FingerData())
        from hand_interfaces.msg import FingerData
        self.thumb = kwargs.get('thumb', FingerData())
        from hand_interfaces.msg import FingerData
        self.index = kwargs.get('index', FingerData())
        from hand_interfaces.msg import FingerData
        self.middle = kwargs.get('middle', FingerData())
        from hand_interfaces.msg import FingerData
        self.ring = kwargs.get('ring', FingerData())
        from hand_interfaces.msg import FingerData
        self.pinky = kwargs.get('pinky', FingerData())

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
        if self.wrist != other.wrist:
            return False
        if self.thumb != other.thumb:
            return False
        if self.index != other.index:
            return False
        if self.middle != other.middle:
            return False
        if self.ring != other.ring:
            return False
        if self.pinky != other.pinky:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def wrist(self):
        """Message field 'wrist'."""
        return self._wrist

    @wrist.setter
    def wrist(self, value):
        if __debug__:
            from hand_interfaces.msg import FingerData
            assert \
                isinstance(value, FingerData), \
                "The 'wrist' field must be a sub message of type 'FingerData'"
        self._wrist = value

    @builtins.property
    def thumb(self):
        """Message field 'thumb'."""
        return self._thumb

    @thumb.setter
    def thumb(self, value):
        if __debug__:
            from hand_interfaces.msg import FingerData
            assert \
                isinstance(value, FingerData), \
                "The 'thumb' field must be a sub message of type 'FingerData'"
        self._thumb = value

    @builtins.property
    def index(self):
        """Message field 'index'."""
        return self._index

    @index.setter
    def index(self, value):
        if __debug__:
            from hand_interfaces.msg import FingerData
            assert \
                isinstance(value, FingerData), \
                "The 'index' field must be a sub message of type 'FingerData'"
        self._index = value

    @builtins.property
    def middle(self):
        """Message field 'middle'."""
        return self._middle

    @middle.setter
    def middle(self, value):
        if __debug__:
            from hand_interfaces.msg import FingerData
            assert \
                isinstance(value, FingerData), \
                "The 'middle' field must be a sub message of type 'FingerData'"
        self._middle = value

    @builtins.property
    def ring(self):
        """Message field 'ring'."""
        return self._ring

    @ring.setter
    def ring(self, value):
        if __debug__:
            from hand_interfaces.msg import FingerData
            assert \
                isinstance(value, FingerData), \
                "The 'ring' field must be a sub message of type 'FingerData'"
        self._ring = value

    @builtins.property
    def pinky(self):
        """Message field 'pinky'."""
        return self._pinky

    @pinky.setter
    def pinky(self, value):
        if __debug__:
            from hand_interfaces.msg import FingerData
            assert \
                isinstance(value, FingerData), \
                "The 'pinky' field must be a sub message of type 'FingerData'"
        self._pinky = value
