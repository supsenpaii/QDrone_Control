"""Offline framing tests. No drone sockets and no MAVLink transmissions."""
import math
import random
import struct
import unittest

from qdrone_to_qgc import TelemetryParser


def packet(seq):
    # Matches capture_tool/qdrone_capture_packet.m, including unavailable fields.
    return struct.pack('<30d', 51234567, 1, seq, seq * 0.001,
                       *([math.nan] * 4), 12.4,
                       0.1, 0.2, 0.3, 0, 0, 9.81,
                       *([math.nan] * 5), *([0] * 10))


class FramingTests(unittest.TestCase):
    def test_every_split_point(self):
        wire = packet(1234)
        for split in range(1, len(wire)):
            parser = TelemetryParser()
            self.assertIsNone(parser.feed(wire[:split]))
            p = parser.feed(wire[split:])
            self.assertEqual(p[2], 1234)
            self.assertEqual(p[8], 12.4)
            self.assertEqual(p[14], 9.81)
            self.assertTrue(math.isnan(p[4]))
            self.assertEqual(parser.count, 1)

    def test_noise_multiple_packets_and_random_fragmentation(self):
        wire = b'noise' + b''.join(packet(i) for i in range(100))
        parser = TelemetryParser()
        rng = random.Random(7)
        pos, last = 0, None
        while pos < len(wire):
            size = rng.randrange(1, 400)
            result = parser.feed(wire[pos:pos + size])
            if result is not None:
                last = result
            pos += size
        self.assertEqual(parser.count, 100)
        self.assertEqual(last[2], 99)

    def test_corrupt_timestamp_resynchronizes(self):
        bad = bytearray(packet(1))
        struct.pack_into('<d', bad, 24, math.nan)
        parser = TelemetryParser()
        p = parser.feed(bad + packet(2))
        self.assertEqual(parser.count, 1)
        self.assertEqual(p[2], 2)

    def test_noise_does_not_grow_buffer(self):
        parser = TelemetryParser()
        self.assertIsNone(parser.feed(b'X' * 100000))
        self.assertLessEqual(len(parser.buffer), 15)


if __name__ == '__main__':
    unittest.main()
