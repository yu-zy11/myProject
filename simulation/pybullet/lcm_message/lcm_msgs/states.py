"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class states(object):
    __slots__ = ["joint_position", "joint_velocity", "torque_feedback", "contact_state", "body_position", "body_velocity", "body_acceleration", "body_quaternion", "body_rpy"]

    __typenames__ = ["float", "float", "float", "float", "float", "float", "float", "float", "float"]

    __dimensions__ = [[12], [12], [12], [4], [3], [3], [3], [4], [4]]

    def __init__(self):
        self.joint_position = [ 0.0 for dim0 in range(12) ]
        self.joint_velocity = [ 0.0 for dim0 in range(12) ]
        self.torque_feedback = [ 0.0 for dim0 in range(12) ]
        self.contact_state = [ 0.0 for dim0 in range(4) ]
        self.body_position = [ 0.0 for dim0 in range(3) ]
        self.body_velocity = [ 0.0 for dim0 in range(3) ]
        self.body_acceleration = [ 0.0 for dim0 in range(3) ]
        self.body_quaternion = [ 0.0 for dim0 in range(4) ]
        self.body_rpy = [ 0.0 for dim0 in range(4) ]

    def encode(self):
        buf = BytesIO()
        buf.write(states._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>12f', *self.joint_position[:12]))
        buf.write(struct.pack('>12f', *self.joint_velocity[:12]))
        buf.write(struct.pack('>12f', *self.torque_feedback[:12]))
        buf.write(struct.pack('>4f', *self.contact_state[:4]))
        buf.write(struct.pack('>3f', *self.body_position[:3]))
        buf.write(struct.pack('>3f', *self.body_velocity[:3]))
        buf.write(struct.pack('>3f', *self.body_acceleration[:3]))
        buf.write(struct.pack('>4f', *self.body_quaternion[:4]))
        buf.write(struct.pack('>4f', *self.body_rpy[:4]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != states._get_packed_fingerprint():
            raise ValueError("Decode error")
        return states._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = states()
        self.joint_position = struct.unpack('>12f', buf.read(48))
        self.joint_velocity = struct.unpack('>12f', buf.read(48))
        self.torque_feedback = struct.unpack('>12f', buf.read(48))
        self.contact_state = struct.unpack('>4f', buf.read(16))
        self.body_position = struct.unpack('>3f', buf.read(12))
        self.body_velocity = struct.unpack('>3f', buf.read(12))
        self.body_acceleration = struct.unpack('>3f', buf.read(12))
        self.body_quaternion = struct.unpack('>4f', buf.read(16))
        self.body_rpy = struct.unpack('>4f', buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if states in parents: return 0
        tmphash = (0x59581c597873bc7d) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if states._packed_fingerprint is None:
            states._packed_fingerprint = struct.pack(">Q", states._get_hash_recursive([]))
        return states._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", states._get_packed_fingerprint())[0]

