"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class mbot_imu_t(object):
    __slots__ = ["utime", "gyro", "accel", "mag", "tb_angles", "temp"]

    __typenames__ = ["int64_t", "float", "float", "float", "float", "float"]

    __dimensions__ = [None, [3], [3], [3], [3], None]

    def __init__(self):
        self.utime = 0
        self.gyro = [ 0.0 for dim0 in range(3) ]
        self.accel = [ 0.0 for dim0 in range(3) ]
        self.mag = [ 0.0 for dim0 in range(3) ]
        self.tb_angles = [ 0.0 for dim0 in range(3) ]
        self.temp = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(mbot_imu_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.utime))
        buf.write(struct.pack('>3f', *self.gyro[:3]))
        buf.write(struct.pack('>3f', *self.accel[:3]))
        buf.write(struct.pack('>3f', *self.mag[:3]))
        buf.write(struct.pack('>3f', *self.tb_angles[:3]))
        buf.write(struct.pack(">f", self.temp))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != mbot_imu_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return mbot_imu_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = mbot_imu_t()
        self.utime = struct.unpack(">q", buf.read(8))[0]
        self.gyro = struct.unpack('>3f', buf.read(12))
        self.accel = struct.unpack('>3f', buf.read(12))
        self.mag = struct.unpack('>3f', buf.read(12))
        self.tb_angles = struct.unpack('>3f', buf.read(12))
        self.temp = struct.unpack(">f", buf.read(4))[0]
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if mbot_imu_t in parents: return 0
        tmphash = (0x2e9f8f63a10a11c1) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if mbot_imu_t._packed_fingerprint is None:
            mbot_imu_t._packed_fingerprint = struct.pack(">Q", mbot_imu_t._get_hash_recursive([]))
        return mbot_imu_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
