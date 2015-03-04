"""autogenerated by genpy from messages/query_cliffRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class query_cliffRequest(genpy.Message):
  _md5sum = "b905a1935d1880468e57600b5a6733f5"
  _type = "messages/query_cliffRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int8 units

"""
  __slots__ = ['units']
  _slot_types = ['int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       units

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(query_cliffRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.units is None:
        self.units = 0
    else:
      self.units = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      buff.write(_struct_b.pack(self.units))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.units,) = _struct_b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(_struct_b.pack(self.units))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.units,) = _struct_b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_b = struct.Struct("<b")
"""autogenerated by genpy from messages/query_cliffResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class query_cliffResponse(genpy.Message):
  _md5sum = "ea977847e353241c9db547f948075479"
  _type = "messages/query_cliffResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int8 front_raw
int8 left_raw
int8 right_raw
int8 cliff_status


"""
  __slots__ = ['front_raw','left_raw','right_raw','cliff_status']
  _slot_types = ['int8','int8','int8','int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       front_raw,left_raw,right_raw,cliff_status

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(query_cliffResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.front_raw is None:
        self.front_raw = 0
      if self.left_raw is None:
        self.left_raw = 0
      if self.right_raw is None:
        self.right_raw = 0
      if self.cliff_status is None:
        self.cliff_status = 0
    else:
      self.front_raw = 0
      self.left_raw = 0
      self.right_raw = 0
      self.cliff_status = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_4b.pack(_x.front_raw, _x.left_raw, _x.right_raw, _x.cliff_status))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 4
      (_x.front_raw, _x.left_raw, _x.right_raw, _x.cliff_status,) = _struct_4b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_4b.pack(_x.front_raw, _x.left_raw, _x.right_raw, _x.cliff_status))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 4
      (_x.front_raw, _x.left_raw, _x.right_raw, _x.cliff_status,) = _struct_4b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4b = struct.Struct("<4b")
class query_cliff(object):
  _type          = 'messages/query_cliff'
  _md5sum = 'f5f120c4ee643ed75fe29879f8885144'
  _request_class  = query_cliffRequest
  _response_class = query_cliffResponse
