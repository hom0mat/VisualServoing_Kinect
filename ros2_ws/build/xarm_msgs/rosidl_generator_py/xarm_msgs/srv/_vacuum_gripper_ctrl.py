# generated from rosidl_generator_py/resource/_idl.py.em
# with input from xarm_msgs:srv/VacuumGripperCtrl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VacuumGripperCtrl_Request(type):
    """Metaclass of message 'VacuumGripperCtrl_Request'."""

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
            module = import_type_support('xarm_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'xarm_msgs.srv.VacuumGripperCtrl_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__vacuum_gripper_ctrl__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__vacuum_gripper_ctrl__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__vacuum_gripper_ctrl__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__vacuum_gripper_ctrl__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__vacuum_gripper_ctrl__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'WAIT__DEFAULT': False,
            'TIMEOUT__DEFAULT': 3.0,
            'DELAY_SEC__DEFAULT': 0.0,
            'SYNC__DEFAULT': True,
            'HARDWARE_VERSION__DEFAULT': 0,
        }

    @property
    def WAIT__DEFAULT(cls):
        """Return default value for message field 'wait'."""
        return False

    @property
    def TIMEOUT__DEFAULT(cls):
        """Return default value for message field 'timeout'."""
        return 3.0

    @property
    def DELAY_SEC__DEFAULT(cls):
        """Return default value for message field 'delay_sec'."""
        return 0.0

    @property
    def SYNC__DEFAULT(cls):
        """Return default value for message field 'sync'."""
        return True

    @property
    def HARDWARE_VERSION__DEFAULT(cls):
        """Return default value for message field 'hardware_version'."""
        return 0


class VacuumGripperCtrl_Request(metaclass=Metaclass_VacuumGripperCtrl_Request):
    """Message class 'VacuumGripperCtrl_Request'."""

    __slots__ = [
        '_on',
        '_wait',
        '_timeout',
        '_delay_sec',
        '_sync',
        '_hardware_version',
    ]

    _fields_and_field_types = {
        'on': 'boolean',
        'wait': 'boolean',
        'timeout': 'float',
        'delay_sec': 'float',
        'sync': 'boolean',
        'hardware_version': 'int16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.on = kwargs.get('on', bool())
        self.wait = kwargs.get(
            'wait', VacuumGripperCtrl_Request.WAIT__DEFAULT)
        self.timeout = kwargs.get(
            'timeout', VacuumGripperCtrl_Request.TIMEOUT__DEFAULT)
        self.delay_sec = kwargs.get(
            'delay_sec', VacuumGripperCtrl_Request.DELAY_SEC__DEFAULT)
        self.sync = kwargs.get(
            'sync', VacuumGripperCtrl_Request.SYNC__DEFAULT)
        self.hardware_version = kwargs.get(
            'hardware_version', VacuumGripperCtrl_Request.HARDWARE_VERSION__DEFAULT)

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
        if self.on != other.on:
            return False
        if self.wait != other.wait:
            return False
        if self.timeout != other.timeout:
            return False
        if self.delay_sec != other.delay_sec:
            return False
        if self.sync != other.sync:
            return False
        if self.hardware_version != other.hardware_version:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def on(self):
        """Message field 'on'."""
        return self._on

    @on.setter
    def on(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'on' field must be of type 'bool'"
        self._on = value

    @builtins.property
    def wait(self):
        """Message field 'wait'."""
        return self._wait

    @wait.setter
    def wait(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'wait' field must be of type 'bool'"
        self._wait = value

    @builtins.property
    def timeout(self):
        """Message field 'timeout'."""
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'timeout' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'timeout' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._timeout = value

    @builtins.property
    def delay_sec(self):
        """Message field 'delay_sec'."""
        return self._delay_sec

    @delay_sec.setter
    def delay_sec(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'delay_sec' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'delay_sec' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._delay_sec = value

    @builtins.property
    def sync(self):
        """Message field 'sync'."""
        return self._sync

    @sync.setter
    def sync(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'sync' field must be of type 'bool'"
        self._sync = value

    @builtins.property
    def hardware_version(self):
        """Message field 'hardware_version'."""
        return self._hardware_version

    @hardware_version.setter
    def hardware_version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'hardware_version' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'hardware_version' field must be an integer in [-32768, 32767]"
        self._hardware_version = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_VacuumGripperCtrl_Response(type):
    """Metaclass of message 'VacuumGripperCtrl_Response'."""

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
            module = import_type_support('xarm_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'xarm_msgs.srv.VacuumGripperCtrl_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__vacuum_gripper_ctrl__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__vacuum_gripper_ctrl__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__vacuum_gripper_ctrl__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__vacuum_gripper_ctrl__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__vacuum_gripper_ctrl__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class VacuumGripperCtrl_Response(metaclass=Metaclass_VacuumGripperCtrl_Response):
    """Message class 'VacuumGripperCtrl_Response'."""

    __slots__ = [
        '_ret',
        '_message',
    ]

    _fields_and_field_types = {
        'ret': 'int16',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ret = kwargs.get('ret', int())
        self.message = kwargs.get('message', str())

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
        if self.ret != other.ret:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ret(self):
        """Message field 'ret'."""
        return self._ret

    @ret.setter
    def ret(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ret' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'ret' field must be an integer in [-32768, 32767]"
        self._ret = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_VacuumGripperCtrl(type):
    """Metaclass of service 'VacuumGripperCtrl'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('xarm_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'xarm_msgs.srv.VacuumGripperCtrl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__vacuum_gripper_ctrl

            from xarm_msgs.srv import _vacuum_gripper_ctrl
            if _vacuum_gripper_ctrl.Metaclass_VacuumGripperCtrl_Request._TYPE_SUPPORT is None:
                _vacuum_gripper_ctrl.Metaclass_VacuumGripperCtrl_Request.__import_type_support__()
            if _vacuum_gripper_ctrl.Metaclass_VacuumGripperCtrl_Response._TYPE_SUPPORT is None:
                _vacuum_gripper_ctrl.Metaclass_VacuumGripperCtrl_Response.__import_type_support__()


class VacuumGripperCtrl(metaclass=Metaclass_VacuumGripperCtrl):
    from xarm_msgs.srv._vacuum_gripper_ctrl import VacuumGripperCtrl_Request as Request
    from xarm_msgs.srv._vacuum_gripper_ctrl import VacuumGripperCtrl_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
