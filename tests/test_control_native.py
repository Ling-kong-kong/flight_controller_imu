"""Real firmware command/ACK tests with mocked USB, never a hardware claim."""
import ctypes
import shutil
import struct
import unittest
import zlib

import test_firmware_native as native


def recalibrate_command(request_id):
    prefix = struct.pack('<4sBBHIIQI', b'IMU2', 1, 0x10, 32, request_id, 1, 0, 0)
    return prefix + struct.pack('<I', zlib.crc32(prefix))


@unittest.skipUnless(shutil.which('cc'), 'native C tests require cc')
class ControlNativeTests(unittest.TestCase):
    setUpClass = classmethod(native.FirmwareNativeTests.setUpClass.__func__)
    tearDownClass = classmethod(native.FirmwareNativeTests.tearDownClass.__func__)
    tearDown = native.FirmwareNativeTests.tearDown
    activate = native.FirmwareNativeTests.activate
    usb_pump = native.FirmwareNativeTests.usb_pump
    sample = native.FirmwareNativeTests.sample
    wire = native.FirmwareNativeTests.wire

    def setUp(self):
        native.FirmwareNativeTests.setUp(self)
        u8p = ctypes.POINTER(ctypes.c_uint8)
        self.lib.mock_rx_data.argtypes = [u8p, ctypes.c_uint32]
        self.lib.imu_decode_recalibrate.argtypes = [u8p, ctypes.POINTER(ctypes.c_uint32)]
        self.lib.imu_decode_recalibrate.restype = ctypes.c_bool
        self.lib.imu_encode_control_reply.argtypes = [u8p, ctypes.c_uint32,
                                                     ctypes.c_uint64, ctypes.c_uint32]

    def receive(self, data, available=4096):
        self.lib.mock_rx_data((ctypes.c_uint8 * len(data)).from_buffer_copy(data), len(data))
        while self.lib.mock_get_rx():
            self.usb_pump(available=available, increment=0)

    def tick(self, g=(0, 0, 0), process=True, pump=True):
        self.now += 1000
        self.sample(1, self.now, (0, 0, 1365))
        self.sample(2, self.now, g)
        self.lib.stream_output_tick(self.now, self.now, 0)
        if process:
            self.lib.mock_process_at(self.now)
        if pump:
            self.usb_pump(increment=0)

    def records(self):
        parser = native.imu.StreamParser()
        parser.feed(self.wire())
        self.assertEqual(parser.crc_errors, 0)
        self.assertEqual(parser.header_errors, 0)
        self.assertFalse(parser.buffer)
        return parser.records

    def replies(self):
        return [r for r in self.records() if isinstance(r, native.imu.ControlReply)]

    def sensors(self):
        return [r for r in self.records() if isinstance(r, native.imu.PairedFrame)]

    def calibrate(self):
        for _ in range(2250):
            self.tick()
        self.assertTrue(self.sensors()[-1].attitude_valid)

    def test_control_golden_vectors_and_strict_command_validation(self):
        command = recalibrate_command(0x12345678)
        self.assertEqual(command.hex(),
                         '494d55320110200078563412010000000000000000000000000000002ce0a8d9')
        request = ctypes.c_uint32()
        def decode(data):
            return self.lib.imu_decode_recalibrate(
                (ctypes.c_uint8 * 32).from_buffer_copy(data), ctypes.byref(request))
        self.assertTrue(decode(command))
        self.assertEqual(request.value, 0x12345678)
        for offset in (0, 4, 5, 6, 7, 12, 16, 20, 24, 28):
            bad = bytearray(command)
            bad[offset] ^= 1
            if offset != 28:
                struct.pack_into('<I', bad, 28, zlib.crc32(bad[:28]))
            self.assertFalse(decode(bad), offset)
        bad = bytearray(command)
        struct.pack_into('<I', bad, 8, 0)
        struct.pack_into('<I', bad, 28, zlib.crc32(bad[:28]))
        self.assertFalse(decode(bad))
        ack = (ctypes.c_uint8 * 32)()
        self.lib.imu_encode_control_reply(ack, 0x12345678, 0x123456789abcdef0, 321)
        self.assertEqual(bytes(ack), native.imu.ControlReply(
            0x12345678, 0, 0x123456789abcdef0, 321).encode())
        self.assertEqual(zlib.crc32(bytes(ack)[:28]), struct.unpack('<I', bytes(ack)[28:])[0])

    def test_fragmented_junk_corrupt_commands_recover_and_ack_after_execution_only(self):
        self.activate()
        self.tick()
        resets = self.lib.mock_fusion_resets()
        command = recalibrate_command(42)
        bad = bytearray(command); bad[-1] ^= 1
        self.receive(b'noiseIMU2'+bad)
        for byte in command:
            self.receive(bytes([byte]))
        self.assertEqual(self.replies(), [])
        self.assertEqual(self.lib.mock_fusion_resets(), resets)
        boundary = self.now + 1000
        self.tick()
        self.assertEqual(self.replies(), [native.imu.ControlReply(42, 0, boundary, 1)])
        self.assertEqual(self.lib.mock_fusion_resets(), resets+1)
        self.assertTrue(self.sensors()[-1].flags & 1024)
        self.assertTrue(self.sensors()[-1].flags & 4096)
        self.assertFalse(self.sensors()[-1].attitude_valid)

    def test_boundary_excludes_backlog_and_keeps_raw_time_sequence_continuous(self):
        self.activate()
        self.calibrate()
        count = len(self.sensors())
        for _ in range(4):
            self.tick(process=False, pump=False)
        self.receive(recalibrate_command(9))
        self.lib.mock_process_at(self.now)
        self.usb_pump(increment=0)
        self.assertEqual(self.replies(), [])
        self.assertTrue(all(f.attitude_valid for f in self.sensors()[-4:]))
        self.tick()
        ack = self.replies()[0]
        self.assertEqual((ack.timestamp_us, ack.sequence), (self.now, count+4))
        samples = self.sensors()
        self.assertEqual([f.sequence for f in samples], list(range(len(samples))))
        self.assertTrue(all(b.timestamp_us-a.timestamp_us == 1000
                            for a, b in zip(samples, samples[1:])))
        self.assertTrue(all((f.ax, f.ay, f.az, f.gx, f.gy, f.gz) == (0,0,1365,0,0,0)
                            for f in samples))
        self.assertFalse(samples[-1].attitude_valid)

    def test_latest_duplicate_only_resends_ack_and_movement_delays_revalidation(self):
        self.activate()
        self.calibrate()
        self.receive(recalibrate_command(10))
        self.tick()
        ack = self.replies()[0]
        resets = self.lib.mock_fusion_resets()
        self.receive(recalibrate_command(10))
        self.tick(g=(0,0,-1475))
        self.assertEqual(self.replies(), [ack, ack])
        self.assertEqual(self.lib.mock_fusion_resets(), resets)
        for _ in range(2300):
            self.tick(g=(0,0,-1475))
        self.assertFalse(self.sensors()[-1].attitude_valid)
        for _ in range(2500):
            self.tick()
        self.assertTrue(self.sensors()[-1].attitude_valid)
        self.assertAlmostEqual(self.sensors()[-1].yaw_deg, 0, delta=.01)
        self.receive(recalibrate_command(10))
        self.tick()
        self.assertEqual(self.replies(), [ack]*3)
        self.assertEqual(self.lib.mock_fusion_resets(), resets)
        self.assertTrue(self.sensors()[-1].attitude_valid)

    def test_ack_backpressure_priority_busy_discard_and_reconnect_preserves_fusion(self):
        self.activate()
        self.calibrate()
        self.receive(recalibrate_command(30))
        # A second, different request while execution is pending is discarded.
        self.receive(recalibrate_command(31))
        self.tick(pump=False)
        before = len(self.wire())
        self.usb_pump(available=31, increment=0)
        self.assertEqual(len(self.wire()), before)
        self.receive(recalibrate_command(32), available=31)
        self.now += 1000
        self.usb_pump(available=32, increment=0)
        self.assertEqual(len(self.wire()), before+32)
        self.assertEqual([r.request_id for r in self.replies()], [30])
        resets = self.lib.mock_fusion_resets()
        self.calibrate()
        self.assertEqual(self.lib.mock_fusion_resets(), resets)
        self.lib.mock_line_state(False)
        for _ in range(5):
            self.tick()
        self.lib.mock_line_state(True)
        self.tick()
        self.assertTrue(self.sensors()[-1].attitude_valid)
        self.assertEqual(self.sensors()[-1].sequence, 0)
        self.assertEqual(self.lib.mock_fusion_resets(), resets)

    def test_overflow_does_not_ack_or_lose_pending_request(self):
        self.activate()
        self.tick()
        self.receive(recalibrate_command(99))
        for _ in range(65):
            self.tick(process=False, pump=False)
        self.lib.mock_process_at(self.now)
        self.usb_pump(increment=0)
        self.assertEqual(self.replies(), [])
        self.assertGreater(self.lib.mock_fusion_drops(), 0)
        self.tick()
        self.assertEqual(self.replies(), [native.imu.ControlReply(99,0,self.now,66)])
        self.assertFalse(self.sensors()[-1].attitude_valid)


if __name__ == '__main__':
    unittest.main()
