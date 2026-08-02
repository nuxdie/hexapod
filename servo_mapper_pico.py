"""
HEX-01 servo mapper bridge for Raspberry Pi Pico / MicroPython.

Run this on the Pico as main.py, then open servo_mapper.html in a Web Serial
capable browser. The browser sends one JSON command per line over USB serial.

Pico wiring:
    GP0  -> SDA on both PCA9685 boards
    GP1  -> SCL on both PCA9685 boards
    3V3  -> VCC on both PCA9685 boards (logic power only)
    GND  -> GND on both boards and the external servo supply

PCA9685 addresses:
    Board 1: 0x40 (default)
    Board 2: 0x41 (A0 solder bridge closed)

The external regulated 5 V servo supply connects to V+, never to Pico 3V3.
All outputs start relaxed. No servo moves until a probe command is received.
"""

from machine import I2C, Pin
import select
import sys
import time

try:
    import ujson as json
except ImportError:
    import json


MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

PCA_ADDRESSES = (0x40, 0x41)
PWM_FREQUENCY_HZ = 50
PWM_PERIOD_US = 1_000_000 // PWM_FREQUENCY_HZ

# Conservative protocol limits. Individual servo travel still needs calibration.
MIN_CENTER_US = 900
MAX_CENTER_US = 2100
MIN_DELTA_US = 5
MAX_DELTA_US = 60
MIN_HOLD_MS = 60
MAX_HOLD_MS = 400
MAX_CYCLES = 4


class PCA9685:
    def __init__(self, i2c, address):
        self.i2c = i2c
        self.address = address
        self._write(MODE1, 0x20)  # Auto-increment enabled, normal mode.
        self.set_pwm_frequency(PWM_FREQUENCY_HZ)
        self.relax_all()

    def _write(self, register, value):
        self.i2c.writeto_mem(self.address, register, bytes((value,)))

    def _read(self, register):
        return self.i2c.readfrom_mem(self.address, register, 1)[0]

    def set_pwm_frequency(self, frequency_hz):
        prescale = round(25_000_000 / (4096 * frequency_hz)) - 1
        previous_mode = self._read(MODE1)
        self._write(MODE1, (previous_mode & 0x7F) | 0x10)
        self._write(PRESCALE, prescale)
        self._write(MODE1, previous_mode)
        time.sleep_ms(5)
        self._write(MODE1, previous_mode | 0xA0)  # Restart + auto-increment.

    def set_pulse_us(self, channel, pulse_us):
        self._validate_channel(channel)
        off_tick = round(pulse_us * 4096 / PWM_PERIOD_US)
        register = LED0_ON_L + 4 * channel
        self.i2c.writeto_mem(
            self.address,
            register,
            bytes((0x00, 0x00, off_tick & 0xFF, (off_tick >> 8) & 0x0F)),
        )

    def relax(self, channel):
        self._validate_channel(channel)
        register = LED0_ON_L + 4 * channel
        # Bit 4 in LEDn_OFF_H forces the output fully off.
        self.i2c.writeto_mem(
            self.address,
            register,
            bytes((0x00, 0x00, 0x00, 0x10)),
        )

    def relax_all(self):
        for channel in range(16):
            self.relax(channel)

    @staticmethod
    def _validate_channel(channel):
        if not 0 <= channel <= 15:
            raise ValueError("channel must be between 0 and 15")


def send(message):
    sys.stdout.write(json.dumps(message) + "\n")
    try:
        sys.stdout.flush()
    except AttributeError:
        pass


def integer(command, name, default=None):
    value = command.get(name, default)
    if value is None:
        raise ValueError("missing " + name)
    if isinstance(value, bool):
        raise ValueError(name + " must be an integer")
    return int(value)


def board_for(command):
    board_index = integer(command, "board")
    if board_index not in (0, 1):
        raise ValueError("board must be 0 or 1")
    board = boards[board_index]
    if board is None:
        raise ValueError("PCA9685 board {} is not available".format(board_index + 1))
    return board_index, board


def relax_all():
    for board in boards:
        if board is not None:
            board.relax_all()


def probe(command):
    board_index, board = board_for(command)
    channel = integer(command, "channel")
    center_us = integer(command, "center_us", 1500)
    delta_us = integer(command, "delta_us", 20)
    cycles = integer(command, "cycles", 2)
    hold_ms = integer(command, "hold_ms", 140)
    relax_after = command.get("relax_after", True) is not False

    if not 0 <= channel <= 15:
        raise ValueError("channel must be between 0 and 15")
    if not MIN_CENTER_US <= center_us <= MAX_CENTER_US:
        raise ValueError("center_us is outside the safe mapper range")
    if not MIN_DELTA_US <= delta_us <= MAX_DELTA_US:
        raise ValueError("delta_us is outside the safe mapper range")
    if not 1 <= cycles <= MAX_CYCLES:
        raise ValueError("cycles is outside the safe mapper range")
    if not MIN_HOLD_MS <= hold_ms <= MAX_HOLD_MS:
        raise ValueError("hold_ms is outside the safe mapper range")

    # The first command establishes a known center. This can cause more movement
    # than delta_us because an analog SG90 has no readable starting position.
    board.set_pulse_us(channel, center_us)
    time.sleep_ms(300)

    try:
        for _ in range(cycles):
            board.set_pulse_us(channel, center_us - delta_us)
            time.sleep_ms(hold_ms)
            board.set_pulse_us(channel, center_us + delta_us)
            time.sleep_ms(hold_ms)
        board.set_pulse_us(channel, center_us)
        time.sleep_ms(hold_ms)
    finally:
        if relax_after:
            board.relax(channel)

    return {
        "ok": True,
        "cmd": "probe",
        "board": board_index,
        "channel": channel,
        "relaxed": relax_after,
    }


def handle(command):
    name = command.get("cmd")

    if name == "hello":
        return {
            "event": "ready",
            "version": 1,
            "addresses": [hex(address) for address in detected_addresses],
        }

    if name == "probe":
        return probe(command)

    if name == "relax":
        board_index, board = board_for(command)
        channel = integer(command, "channel")
        board.relax(channel)
        return {
            "ok": True,
            "cmd": "relax",
            "board": board_index,
            "channel": channel,
        }

    if name == "relax_all":
        relax_all()
        return {"ok": True, "cmd": "relax_all"}

    if name == "scan":
        return {
            "ok": True,
            "cmd": "scan",
            "addresses": [hex(address) for address in i2c.scan()],
        }

    raise ValueError("unknown command")


i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
detected_addresses = i2c.scan()
boards = []

for address in PCA_ADDRESSES:
    if address in detected_addresses:
        boards.append(PCA9685(i2c, address))
    else:
        boards.append(None)

send({
    "event": "ready",
    "version": 1,
    "addresses": [hex(address) for address in detected_addresses],
})

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

try:
    while True:
        if not poll.poll(100):
            continue

        line = sys.stdin.readline()
        if not line:
            time.sleep_ms(10)
            continue

        try:
            command = json.loads(line)
            if not isinstance(command, dict):
                raise ValueError("command must be a JSON object")
            send(handle(command))
        except Exception as error:
            send({"ok": False, "error": str(error)})
except KeyboardInterrupt:
    relax_all()
    send({"event": "stopped", "relaxed": True})
