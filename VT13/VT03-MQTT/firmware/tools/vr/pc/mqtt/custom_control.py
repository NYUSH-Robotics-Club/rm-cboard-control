"""Official protobuf CustomControl (bytes data = 1), maximum 30 payload bytes."""
from google.protobuf import descriptor_pb2, descriptor_pool, message_factory

_file = descriptor_pb2.FileDescriptorProto(name="vr_custom_control.proto", package="vr_mqtt", syntax="proto3")
_message = _file.message_type.add(name="CustomControl")
_message.field.add(name="data", number=1, type=12, label=1)
_pool = descriptor_pool.DescriptorPool()
_pool.Add(_file)
CustomControl = message_factory.GetMessageClass(_pool.FindMessageTypeByName("vr_mqtt.CustomControl"))


def encode(payload):
    if len(payload) > 30:
        raise ValueError("CustomControl.data must not exceed 30 bytes")
    return CustomControl(data=payload).SerializeToString()
