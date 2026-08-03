"""Host-side tests for PCA9685 writes used by servo_mapper_pico.py."""

import sys
import types
import unittest


# CPython does not provide MicroPython's machine module. The mapper only needs
# these names to exist at import time; tests supply a recording I2C object.
machine = types.ModuleType("machine")
machine.I2C = object
machine.Pin = object
sys.modules.setdefault("machine", machine)

import servo_mapper_pico as mapper


class RecordingI2C:
    def __init__(self):
        self.memory = {mapper.MODE1: 0x00}
        self.writes = []

    def writeto_mem(self, address, register, payload):
        payload = bytes(payload)
        self.writes.append((address, register, payload))
        for offset, value in enumerate(payload):
            self.memory[register + offset] = value

    def readfrom_mem(self, address, register, length):
        return bytes(self.memory.get(register + offset, 0x00) for offset in range(length))


class PCA9685Tests(unittest.TestCase):
    def setUp(self):
        self.original_sleep_ms = getattr(mapper.time, "sleep_ms", None)
        mapper.time.sleep_ms = lambda milliseconds: None
        self.i2c = RecordingI2C()
        self.pca = mapper.PCA9685(self.i2c, 0x40)
        self.i2c.writes.clear()

    def tearDown(self):
        if self.original_sleep_ms is None:
            del mapper.time.sleep_ms
        else:
            mapper.time.sleep_ms = self.original_sleep_ms

    def test_1500_us_pulse_writes_expected_12_bit_tick(self):
        self.pca.set_pulse_us(5, 1500)

        self.assertEqual(
            self.i2c.writes[-1],
            (0x40, mapper.LED0_ON_L + 4 * 5, bytes((0x00, 0x00, 0x33, 0x01))),
        )

    def test_relax_sets_full_off_bit(self):
        self.pca.relax(7)

        self.assertEqual(
            self.i2c.writes[-1],
            (0x40, mapper.LED0_ON_L + 4 * 7, bytes((0x00, 0x00, 0x00, 0x10))),
        )

    def test_probe_wiggles_around_center_then_relaxes(self):
        mapper.boards = [self.pca, None]
        register = mapper.LED0_ON_L + 4 * 2

        response = mapper.probe({
            "board": 0,
            "channel": 2,
            "center_us": 1500,
            "delta_us": 40,
            "cycles": 3,
            "hold_ms": 60,
            "relax_after": True,
        })

        channel_writes = [payload for _, written_register, payload in self.i2c.writes if written_register == register]
        expected_center = bytes((0x00, 0x00, 0x33, 0x01))
        expected_left = bytes((0x00, 0x00, 0x2B, 0x01))
        expected_right = bytes((0x00, 0x00, 0x3B, 0x01))

        self.assertEqual(channel_writes[0], expected_center)
        self.assertEqual(channel_writes[1:7], [expected_left, expected_right] * 3)
        self.assertEqual(channel_writes[7], expected_center)
        self.assertEqual(channel_writes[8], bytes((0x00, 0x00, 0x00, 0x10)))
        self.assertEqual(response["relaxed"], True)

    def test_set_pulse_command_drives_requested_channel(self):
        mapper.boards = [self.pca, None]

        response = mapper.handle({
            "cmd": "set_pulse",
            "board": 0,
            "channel": 3,
            "pulse_us": 1600,
        })

        self.assertEqual(response["pulse_us"], 1600)
        self.assertEqual(
            self.i2c.writes[-1],
            (0x40, mapper.LED0_ON_L + 4 * 3, bytes((0x00, 0x00, 0x48, 0x01))),
        )

    def test_set_pulse_rejects_value_outside_pwm_frame(self):
        mapper.boards = [self.pca, None]

        with self.assertRaisesRegex(ValueError, "PCA9685 frame range"):
            mapper.handle({
                "cmd": "set_pulse",
                "board": 0,
                "channel": 3,
                "pulse_us": mapper.MAX_HARDWARE_PULSE_US + 1,
            })

    def test_set_pulse_accepts_nonstandard_calibration_value(self):
        mapper.boards = [self.pca, None]

        mapper.handle({
            "cmd": "set_pulse",
            "board": 0,
            "channel": 1,
            "pulse_us": 4000,
        })

        self.assertEqual(
            self.i2c.writes[-1],
            (0x40, mapper.LED0_ON_L + 4, bytes((0x00, 0x00, 0x33, 0x03))),
        )

    def test_set_pulses_command_updates_complete_frame(self):
        mapper.boards = [self.pca, None]

        response = mapper.handle({
            "cmd": "set_pulses",
            "pulses": [
                {"board": 0, "channel": 1, "pulse_us": 1400},
                {"board": 0, "channel": 4, "pulse_us": 1600},
                {"board": 0, "channel": 7, "pulse_us": 1800},
            ],
        })

        self.assertEqual(response["count"], 3)
        self.assertEqual(
            [register for _, register, _ in self.i2c.writes],
            [mapper.LED0_ON_L + 4, mapper.LED0_ON_L + 16, mapper.LED0_ON_L + 28],
        )

    def test_set_pulses_validates_complete_frame_before_writing(self):
        mapper.boards = [self.pca, None]

        with self.assertRaisesRegex(ValueError, "PCA9685 frame range"):
            mapper.handle({
                "cmd": "set_pulses",
                "pulses": [
                    {"board": 0, "channel": 1, "pulse_us": 1400},
                    {"board": 0, "channel": 4, "pulse_us": mapper.MAX_HARDWARE_PULSE_US + 1},
                ],
            })

        self.assertEqual(self.i2c.writes, [])

    def test_probe_rejects_wiggle_past_pulse_boundary(self):
        mapper.boards = [self.pca, None]

        with self.assertRaisesRegex(ValueError, "below the control range"):
            mapper.probe({
                "board": 0,
                "channel": 0,
                "center_us": 500,
                "delta_us": 40,
                "cycles": 1,
                "hold_ms": 60,
            })


if __name__ == "__main__":
    unittest.main()
