"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class TimeStamp_t(object):
    __slots__ = ["time_stamp"]

    __typenames__ = ["int64_t"]

    __dimensions__ = [None]

    def __init__(self):
        self.time_stamp = 0

    def encode(self):
        buf = BytesIO()
        buf.write(TimeStamp_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.time_stamp))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != TimeStamp_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return TimeStamp_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = TimeStamp_t()
        self.time_stamp = struct.unpack(">q", buf.read(8))[0]
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if TimeStamp_t in parents: return 0
        tmphash = (0x7cdccc00def406c5) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if TimeStamp_t._packed_fingerprint is None:
            TimeStamp_t._packed_fingerprint = struct.pack(">Q", TimeStamp_t._get_hash_recursive([]))
        return TimeStamp_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

