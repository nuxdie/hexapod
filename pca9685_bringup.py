"""
PCA9685 Driver + 18-Servo Bring-Up Test
Raspberry Pi Pico / MicroPython

Purpose: verify wiring by centering all 18 SG90 servos to a safe neutral
angle (90 degrees) one at a time, with a short pause so you can visually
confirm each leg joint moves correctly before running any gait code.

Wiring:
    Pico GP0  -> PCA9685 SDA
    Pico GP1  -> PCA9685 SCL
    Pico 3.3V -> PCA9685 VCC (logic power)
    Pico GND  -> PCA9685 GND (shared with servo supply GND)
    External 5-6V supply -> PCA9685 V+ terminal block (servo power)

If using TWO PCA9685 boards (18 servos > 16 channels on one board):
    Board 1 (default address 0x40): channels 0-15
    Board 2 (bridge A0 solder pad -> address 0x41): channels 0-1 (or however
        you choose to split it)
"""

from machine import Pin, I2C
import time

# ---------------------------------------------------------------------------
# PCA9685 register map (only what we need)
# ---------------------------------------------------------------------------
PCA9685_MODE1      = 0x00
PCA9685_PRESCALE   = 0xFE
PCA9685_LED0_ON_L  = 0x06  # first channel's "on" low byte; each channel = 4 regs

class PCA9685:
    def __init__(self, i2c, address=0x40):
        self.i2c = i2c
        self.address = address
        self._write(PCA9685_MODE1, 0x20)  # AI (auto-increment) bit enabled, normal mode

    def _write(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes([value]))

    def _read(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def set_pwm_freq(self, freq_hz):
        """Set PWM frequency (50Hz standard for analog servos like SG90)."""
        prescale_val = int(25000000.0 / (4096 * freq_hz) - 1)
        old_mode = self._read(PCA9685_MODE1)
        self._write(PCA9685_MODE1, (old_mode & 0x7F) | 0x10)  # sleep
        self._write(PCA9685_PRESCALE, prescale_val)
        self._write(PCA9685_MODE1, old_mode)
        time.sleep_ms(5)
        self._write(PCA9685_MODE1, old_mode | 0x80)  # restart

    def set_pwm(self, channel, on, off):
        """Set raw ON/OFF tick values (0-4095) for a channel."""
        reg = PCA9685_LED0_ON_L + 4 * channel
        self.i2c.writeto_mem(self.address, reg,
            bytes([on & 0xFF, on >> 8, off & 0xFF, off >> 8]))

    def set_servo_angle(self, channel, angle_deg, min_us=500, max_us=2400):
        """Set a servo channel to an angle in degrees (0-180)."""
        angle_deg = max(0, min(180, angle_deg))
        pulse_us = min_us + (max_us - min_us) * (angle_deg / 180.0)
        # convert microseconds to 12-bit ticks at 50Hz (20000us period)
        off_ticks = int(pulse_us / 20000 * 4096)
        self.set_pwm(channel, 0, off_ticks)


# ---------------------------------------------------------------------------
# BRING-UP TEST
# ---------------------------------------------------------------------------
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

print("Scanning I2C bus...")
devices = i2c.scan()
print("Found devices at:", [hex(d) for d in devices])

if not devices:
    print("No I2C devices found! Check wiring: SDA=GP0, SCL=GP1, VCC=3.3V, GND shared.")
else:
    pca = PCA9685(i2c, address=0x40)
    pca.set_pwm_freq(50)

    # If you have a second PCA9685 board for the remaining channels:
    pca2 = PCA9685(i2c, address=0x41)
    pca2.set_pwm_freq(50)

    NUM_CHANNELS = 16  # channels available on this one board (0-15)

    print("Centering all servos to 90 degrees, one at a time...")
    for ch in range(NUM_CHANNELS):
        print("Channel", ch, "-> 90 deg")
        pca.set_servo_angle(ch, 90)
        time.sleep(0.5)  # watch each joint move before continuing
    for ch in range(NUM_CHANNELS):
        print("Channel", ch, "-> 90 deg")
        pca2.set_servo_angle(ch, 90)
        time.sleep(0.5)  # watch each joint move before continuing

    print("Done. All servos on board 1 centered.")
    print("If any servo didn't move, or moved to a strange angle, check:")
    print(" - that servo's wiring/channel assignment")
    print(" - servo power supply voltage/current (brownout under load)")
    print(" - common ground between Pico, PCA9685, and servo supply")