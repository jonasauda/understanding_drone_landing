# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: model.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='model.proto',
  package='',
  syntax='proto3',
  serialized_pb=_b('\n\x0bmodel.proto\"\xb9\x05\n\nMocapFrame\x12\x15\n\rElapsedMillis\x18\x01 \x01(\x03\x12\x10\n\x08SourceId\x18\x02 \x01(\t\x12\x13\n\x0b\x41\x64\x61pterType\x18\x03 \x01(\t\x12\x0f\n\x07Gesture\x18\x04 \x01(\t\x12\x0f\n\x07Latency\x18\x05 \x01(\x02\x12 \n\x06\x62odies\x18\x06 \x03(\x0b\x32\x10.MocapFrame.Body\x1a\xa8\x04\n\x04\x42ody\x12,\n\x08\x42odyType\x18\x01 \x01(\x0e\x32\x1a.MocapFrame.Body.EBodyType\x12,\n\x08SideType\x18\x02 \x01(\x0e\x32\x1a.MocapFrame.Body.ESideType\x12&\n\x06Points\x18\x03 \x03(\x0b\x32\x16.MocapFrame.Body.Point\x12-\n\x08Rotation\x18\x04 \x01(\x0b\x32\x1b.MocapFrame.Body.Quaternion\x12*\n\x08\x43\x65ntroid\x18\x05 \x01(\x0b\x32\x18.MocapFrame.Body.Vector3\x12\x0c\n\x04Name\x18\x06 \x01(\t\x1a*\n\x07Vector3\x12\t\n\x01X\x18\x01 \x01(\x02\x12\t\n\x01Y\x18\x02 \x01(\x02\x12\t\n\x01Z\x18\x03 \x01(\x02\x1aP\n\x05Point\x12\x0c\n\x04Name\x18\x01 \x01(\t\x12\r\n\x05State\x18\x02 \x01(\t\x12*\n\x08Position\x18\x03 \x01(\x0b\x32\x18.MocapFrame.Body.Vector3\x1a\x38\n\nQuaternion\x12\t\n\x01X\x18\x01 \x01(\x02\x12\t\n\x01Y\x18\x02 \x01(\x02\x12\t\n\x01Z\x18\x03 \x01(\x02\x12\t\n\x01W\x18\x04 \x01(\x02\"M\n\tEBodyType\x12\n\n\x06Marker\x10\x00\x12\r\n\tMarkerSet\x10\x01\x12\r\n\tRigidBody\x10\x02\x12\x0c\n\x08Skeleton\x10\x03\x12\x08\n\x04Hand\x10\x04\",\n\tESideType\x12\x08\n\x04Left\x10\x00\x12\t\n\x05Right\x10\x01\x12\n\n\x06NoSide\x10\x02\"M\n\x0fSessionMetadata\x12\x0c\n\x04Name\x18\x01 \x01(\t\x12\x10\n\x08\x44uration\x18\x02 \x01(\x03\x12\x1a\n\x12SessionStartMillis\x18\x03 \x01(\x03\"F\n\x07Session\x12\x1e\n\x04meta\x18\x01 \x01(\x0b\x32\x10.SessionMetadata\x12\x1b\n\x06\x66rames\x18\x02 \x03(\x0b\x32\x0b.MocapFrame\"\xa2\x01\n\x10SessionsMetadata\x12>\n\x0finputSourceMeta\x18\x01 \x03(\x0b\x32%.SessionsMetadata.InputSourceMetadata\x1aN\n\x13InputSourceMetadata\x12\x10\n\x08SourceId\x18\x01 \x01(\t\x12%\n\x0bsessionMeta\x18\x02 \x03(\x0b\x32\x10.SessionMetadataB\x13\xaa\x02\x10VinteR.Model.Genb\x06proto3')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_MOCAPFRAME_BODY_EBODYTYPE = _descriptor.EnumDescriptor(
  name='EBodyType',
  full_name='MocapFrame.Body.EBodyType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='Marker', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MarkerSet', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RigidBody', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='Skeleton', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='Hand', index=4, number=4,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=590,
  serialized_end=667,
)
_sym_db.RegisterEnumDescriptor(_MOCAPFRAME_BODY_EBODYTYPE)

_MOCAPFRAME_BODY_ESIDETYPE = _descriptor.EnumDescriptor(
  name='ESideType',
  full_name='MocapFrame.Body.ESideType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='Left', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='Right', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NoSide', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=669,
  serialized_end=713,
)
_sym_db.RegisterEnumDescriptor(_MOCAPFRAME_BODY_ESIDETYPE)


_MOCAPFRAME_BODY_VECTOR3 = _descriptor.Descriptor(
  name='Vector3',
  full_name='MocapFrame.Body.Vector3',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='X', full_name='MocapFrame.Body.Vector3.X', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Y', full_name='MocapFrame.Body.Vector3.Y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Z', full_name='MocapFrame.Body.Vector3.Z', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=406,
  serialized_end=448,
)

_MOCAPFRAME_BODY_POINT = _descriptor.Descriptor(
  name='Point',
  full_name='MocapFrame.Body.Point',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='Name', full_name='MocapFrame.Body.Point.Name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='State', full_name='MocapFrame.Body.Point.State', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Position', full_name='MocapFrame.Body.Point.Position', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=450,
  serialized_end=530,
)

_MOCAPFRAME_BODY_QUATERNION = _descriptor.Descriptor(
  name='Quaternion',
  full_name='MocapFrame.Body.Quaternion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='X', full_name='MocapFrame.Body.Quaternion.X', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Y', full_name='MocapFrame.Body.Quaternion.Y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Z', full_name='MocapFrame.Body.Quaternion.Z', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='W', full_name='MocapFrame.Body.Quaternion.W', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=532,
  serialized_end=588,
)

_MOCAPFRAME_BODY = _descriptor.Descriptor(
  name='Body',
  full_name='MocapFrame.Body',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='BodyType', full_name='MocapFrame.Body.BodyType', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='SideType', full_name='MocapFrame.Body.SideType', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Points', full_name='MocapFrame.Body.Points', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Rotation', full_name='MocapFrame.Body.Rotation', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Centroid', full_name='MocapFrame.Body.Centroid', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Name', full_name='MocapFrame.Body.Name', index=5,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_MOCAPFRAME_BODY_VECTOR3, _MOCAPFRAME_BODY_POINT, _MOCAPFRAME_BODY_QUATERNION, ],
  enum_types=[
    _MOCAPFRAME_BODY_EBODYTYPE,
    _MOCAPFRAME_BODY_ESIDETYPE,
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=161,
  serialized_end=713,
)

_MOCAPFRAME = _descriptor.Descriptor(
  name='MocapFrame',
  full_name='MocapFrame',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ElapsedMillis', full_name='MocapFrame.ElapsedMillis', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='SourceId', full_name='MocapFrame.SourceId', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='AdapterType', full_name='MocapFrame.AdapterType', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Gesture', full_name='MocapFrame.Gesture', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Latency', full_name='MocapFrame.Latency', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='bodies', full_name='MocapFrame.bodies', index=5,
      number=6, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_MOCAPFRAME_BODY, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=16,
  serialized_end=713,
)


_SESSIONMETADATA = _descriptor.Descriptor(
  name='SessionMetadata',
  full_name='SessionMetadata',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='Name', full_name='SessionMetadata.Name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Duration', full_name='SessionMetadata.Duration', index=1,
      number=2, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='SessionStartMillis', full_name='SessionMetadata.SessionStartMillis', index=2,
      number=3, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=715,
  serialized_end=792,
)


_SESSION = _descriptor.Descriptor(
  name='Session',
  full_name='Session',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='meta', full_name='Session.meta', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='frames', full_name='Session.frames', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=794,
  serialized_end=864,
)


_SESSIONSMETADATA_INPUTSOURCEMETADATA = _descriptor.Descriptor(
  name='InputSourceMetadata',
  full_name='SessionsMetadata.InputSourceMetadata',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='SourceId', full_name='SessionsMetadata.InputSourceMetadata.SourceId', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sessionMeta', full_name='SessionsMetadata.InputSourceMetadata.sessionMeta', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=951,
  serialized_end=1029,
)

_SESSIONSMETADATA = _descriptor.Descriptor(
  name='SessionsMetadata',
  full_name='SessionsMetadata',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='inputSourceMeta', full_name='SessionsMetadata.inputSourceMeta', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_SESSIONSMETADATA_INPUTSOURCEMETADATA, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=867,
  serialized_end=1029,
)

_MOCAPFRAME_BODY_VECTOR3.containing_type = _MOCAPFRAME_BODY
_MOCAPFRAME_BODY_POINT.fields_by_name['Position'].message_type = _MOCAPFRAME_BODY_VECTOR3
_MOCAPFRAME_BODY_POINT.containing_type = _MOCAPFRAME_BODY
_MOCAPFRAME_BODY_QUATERNION.containing_type = _MOCAPFRAME_BODY
_MOCAPFRAME_BODY.fields_by_name['BodyType'].enum_type = _MOCAPFRAME_BODY_EBODYTYPE
_MOCAPFRAME_BODY.fields_by_name['SideType'].enum_type = _MOCAPFRAME_BODY_ESIDETYPE
_MOCAPFRAME_BODY.fields_by_name['Points'].message_type = _MOCAPFRAME_BODY_POINT
_MOCAPFRAME_BODY.fields_by_name['Rotation'].message_type = _MOCAPFRAME_BODY_QUATERNION
_MOCAPFRAME_BODY.fields_by_name['Centroid'].message_type = _MOCAPFRAME_BODY_VECTOR3
_MOCAPFRAME_BODY.containing_type = _MOCAPFRAME
_MOCAPFRAME_BODY_EBODYTYPE.containing_type = _MOCAPFRAME_BODY
_MOCAPFRAME_BODY_ESIDETYPE.containing_type = _MOCAPFRAME_BODY
_MOCAPFRAME.fields_by_name['bodies'].message_type = _MOCAPFRAME_BODY
_SESSION.fields_by_name['meta'].message_type = _SESSIONMETADATA
_SESSION.fields_by_name['frames'].message_type = _MOCAPFRAME
_SESSIONSMETADATA_INPUTSOURCEMETADATA.fields_by_name['sessionMeta'].message_type = _SESSIONMETADATA
_SESSIONSMETADATA_INPUTSOURCEMETADATA.containing_type = _SESSIONSMETADATA
_SESSIONSMETADATA.fields_by_name['inputSourceMeta'].message_type = _SESSIONSMETADATA_INPUTSOURCEMETADATA
DESCRIPTOR.message_types_by_name['MocapFrame'] = _MOCAPFRAME
DESCRIPTOR.message_types_by_name['SessionMetadata'] = _SESSIONMETADATA
DESCRIPTOR.message_types_by_name['Session'] = _SESSION
DESCRIPTOR.message_types_by_name['SessionsMetadata'] = _SESSIONSMETADATA

MocapFrame = _reflection.GeneratedProtocolMessageType('MocapFrame', (_message.Message,), dict(

  Body = _reflection.GeneratedProtocolMessageType('Body', (_message.Message,), dict(

    Vector3 = _reflection.GeneratedProtocolMessageType('Vector3', (_message.Message,), dict(
      DESCRIPTOR = _MOCAPFRAME_BODY_VECTOR3,
      __module__ = 'model_pb2'
      # @@protoc_insertion_point(class_scope:MocapFrame.Body.Vector3)
      ))
    ,

    Point = _reflection.GeneratedProtocolMessageType('Point', (_message.Message,), dict(
      DESCRIPTOR = _MOCAPFRAME_BODY_POINT,
      __module__ = 'model_pb2'
      # @@protoc_insertion_point(class_scope:MocapFrame.Body.Point)
      ))
    ,

    Quaternion = _reflection.GeneratedProtocolMessageType('Quaternion', (_message.Message,), dict(
      DESCRIPTOR = _MOCAPFRAME_BODY_QUATERNION,
      __module__ = 'model_pb2'
      # @@protoc_insertion_point(class_scope:MocapFrame.Body.Quaternion)
      ))
    ,
    DESCRIPTOR = _MOCAPFRAME_BODY,
    __module__ = 'model_pb2'
    # @@protoc_insertion_point(class_scope:MocapFrame.Body)
    ))
  ,
  DESCRIPTOR = _MOCAPFRAME,
  __module__ = 'model_pb2'
  # @@protoc_insertion_point(class_scope:MocapFrame)
  ))
_sym_db.RegisterMessage(MocapFrame)
_sym_db.RegisterMessage(MocapFrame.Body)
_sym_db.RegisterMessage(MocapFrame.Body.Vector3)
_sym_db.RegisterMessage(MocapFrame.Body.Point)
_sym_db.RegisterMessage(MocapFrame.Body.Quaternion)

SessionMetadata = _reflection.GeneratedProtocolMessageType('SessionMetadata', (_message.Message,), dict(
  DESCRIPTOR = _SESSIONMETADATA,
  __module__ = 'model_pb2'
  # @@protoc_insertion_point(class_scope:SessionMetadata)
  ))
_sym_db.RegisterMessage(SessionMetadata)

Session = _reflection.GeneratedProtocolMessageType('Session', (_message.Message,), dict(
  DESCRIPTOR = _SESSION,
  __module__ = 'model_pb2'
  # @@protoc_insertion_point(class_scope:Session)
  ))
_sym_db.RegisterMessage(Session)

SessionsMetadata = _reflection.GeneratedProtocolMessageType('SessionsMetadata', (_message.Message,), dict(

  InputSourceMetadata = _reflection.GeneratedProtocolMessageType('InputSourceMetadata', (_message.Message,), dict(
    DESCRIPTOR = _SESSIONSMETADATA_INPUTSOURCEMETADATA,
    __module__ = 'model_pb2'
    # @@protoc_insertion_point(class_scope:SessionsMetadata.InputSourceMetadata)
    ))
  ,
  DESCRIPTOR = _SESSIONSMETADATA,
  __module__ = 'model_pb2'
  # @@protoc_insertion_point(class_scope:SessionsMetadata)
  ))
_sym_db.RegisterMessage(SessionsMetadata)
_sym_db.RegisterMessage(SessionsMetadata.InputSourceMetadata)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\252\002\020VinteR.Model.Gen'))
# @@protoc_insertion_point(module_scope)
