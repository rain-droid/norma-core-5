"""ST3215 servo register layout, state parsing, and the tick <-> angle mapping."""

from __future__ import annotations

import struct
from dataclasses import dataclass

TICKS = 4096

# RAM registers (address, size in bytes)
REG_OFFSET = 0x1F  # EEPROM: position offset the servo subtracts from its encoder reading
REG_OVERLOAD_TORQUE = 0x24  # EEPROM: output (%) that, held for 2 s, trips overload protection
REG_TORQUE_ENABLE = 0x28
REG_ACC = 0x29
REG_GOAL_POSITION = 0x2A
REG_GOAL_SPEED = 0x2E
REG_TORQUE_LIMIT = 0x30
REG_LOCK = 0x37  # 0 lets EEPROM registers be written, 1 protects them
REG_PRESENT_POSITION = 0x38
REG_PRESENT_SPEED = 0x3A
REG_PRESENT_LOAD = 0x3C
REG_PRESENT_VOLTAGE = 0x3E
REG_PRESENT_TEMPERATURE = 0x3F
REG_STATUS = 0x41
REG_MOVING = 0x42
REG_PRESENT_CURRENT = 0x45
STATE_DUMP_LEN = 0x47  # the Station publishes registers 0x00..0x46 for every servo

ARM_IDS = (1, 2, 3, 4, 5, 6, 7)
GRIPPER_ID = 8
ALL_IDS = ARM_IDS + (GRIPPER_ID,)


def _u16(buf: bytes, addr: int) -> int:
    return struct.unpack_from("<H", buf, addr)[0] if len(buf) >= addr + 2 else 0


def fold_position(raw: int) -> int:
    """Present/goal position registers carry a sign bit; fold them back into 0..4095."""
    if raw & 0x8000:
        return (TICKS - (raw & (TICKS - 1))) & (TICKS - 1)
    return raw & (TICKS - 1)


def _signed(raw: int, sign_bit: int) -> int:
    mag = raw & ((1 << sign_bit) - 1)
    return -mag if raw & (1 << sign_bit) else mag


@dataclass
class MotorState:
    id: int
    tick: int = 2048
    goal_tick: int = 2048
    torque: bool = False
    speed: int = 0  # ticks/s, signed
    load: int = 0  # 0.1 % of stall torque, signed
    current: int = 0  # raw units, ~6.5 mA each, magnitude
    volt: float = 0.0
    temp: int = 0
    status: int = 0  # servo error bits, 0 = healthy
    moving: bool = False
    range_min: int = 0
    range_max: int = 0
    range_frozen: bool = False
    has_error: bool = False  # bus-level error reported by the Station for this servo
    offset: int = 0  # EEPROM position offset; raw encoder position = (tick + offset) mod 4096
    overload_torque: int = 0  # EEPROM overload-protection threshold, percent of full output

    @property
    def encoder(self) -> int:
        """Raw encoder position, which does not change when calibration rewrites the offset."""
        return (self.tick + self.offset) % TICKS

    @property
    def calibrated(self) -> bool:
        return self.range_min != self.range_max

    @classmethod
    def from_dump(cls, motor_id: int, dump: bytes, **extra) -> "MotorState":
        if len(dump) < STATE_DUMP_LEN:
            return cls(id=motor_id, **extra)
        return cls(
            id=motor_id,
            tick=fold_position(_u16(dump, REG_PRESENT_POSITION)),
            goal_tick=fold_position(_u16(dump, REG_GOAL_POSITION)),
            torque=dump[REG_TORQUE_ENABLE] != 0,
            speed=_signed(_u16(dump, REG_PRESENT_SPEED), 15),
            load=_signed(_u16(dump, REG_PRESENT_LOAD), 10),
            current=abs(_signed(_u16(dump, REG_PRESENT_CURRENT), 15)),
            volt=dump[REG_PRESENT_VOLTAGE] / 10.0,
            temp=dump[REG_PRESENT_TEMPERATURE],
            status=dump[REG_STATUS],
            moving=dump[REG_MOVING] != 0,
            offset=struct.unpack_from("<h", dump, REG_OFFSET)[0],
            overload_torque=dump[REG_OVERLOAD_TORQUE],
            **extra,
        )

    def to_dump(self) -> bytes:
        """Inverse of from_dump, used by the simulator."""
        buf = bytearray(STATE_DUMP_LEN)
        buf[REG_TORQUE_ENABLE] = 1 if self.torque else 0
        struct.pack_into("<H", buf, REG_GOAL_POSITION, self.goal_tick & 0xFFFF)
        struct.pack_into("<H", buf, REG_PRESENT_POSITION, self.tick & 0xFFFF)
        struct.pack_into("<H", buf, REG_PRESENT_SPEED, (abs(self.speed) & 0x7FFF) | (0x8000 if self.speed < 0 else 0))
        struct.pack_into("<H", buf, REG_PRESENT_LOAD, (abs(self.load) & 0x3FF) | (0x400 if self.load < 0 else 0))
        buf[REG_PRESENT_VOLTAGE] = int(round(self.volt * 10)) & 0xFF
        buf[REG_PRESENT_TEMPERATURE] = self.temp & 0xFF
        buf[REG_STATUS] = self.status & 0xFF
        buf[REG_MOVING] = 1 if self.moving else 0
        struct.pack_into("<H", buf, REG_PRESENT_CURRENT, self.current & 0x7FFF)
        return bytes(buf)


TICK_RAD = 2 * 3.141592653589793 / TICKS


@dataclass
class JointRange:
    """One servo's calibrated tick arc and how it maps to joint angle.

    The arc comes from the Station's calibration: `range_min`/`range_max` are the ticks at the
    joint's two mechanical end stops, and after a freeze the arc is centred on tick 2048.

    Angle uses the servo's true scale (4096 ticks per turn), anchored so the middle of the
    measured arc is the middle of the URDF joint range. That keeps angles right even when the
    measured arc is a little shorter or longer than the URDF says, which a proportional mapping
    (what the Station's viewer does) would turn into a scale error across the whole range.
    """

    lower: float  # URDF limits, rad
    upper: float
    range_min: int
    range_max: int
    sign: float = 1.0  # +1: ticks increase with angle

    @property
    def span(self) -> int:
        return self.range_max - self.range_min

    @property
    def valid(self) -> bool:
        """Usable only once frozen: a non-wrapping arc with real width."""
        return 0 <= self.range_min < self.range_max < TICKS and self.span > 50

    @property
    def mid_tick(self) -> float:
        return 0.5 * (self.range_min + self.range_max)

    @property
    def mid_angle(self) -> float:
        return 0.5 * (self.lower + self.upper)

    def angle(self, tick: float) -> float:
        return self.mid_angle + self.sign * (tick - self.mid_tick) * TICK_RAD

    def tick(self, angle: float, margin_ticks: int = 0) -> int:
        t = self.mid_tick + self.sign * (angle - self.mid_angle) / TICK_RAD
        return int(round(min(self.range_max - margin_ticks, max(self.range_min + margin_ticks, t))))

    def angle_limits(self, margin: float = 0.0) -> tuple[float, float]:
        """Reachable angle interval: the measured stops, never wider than the URDF limits."""
        a, b = sorted((self.angle(self.range_min), self.angle(self.range_max)))
        return max(a, self.lower) + margin, min(b, self.upper) - margin

    # --- gripper-style use: position within the arc as a 0..1 fraction

    def norm(self, tick: float) -> float:
        return min(1.0, max(0.0, (tick - self.range_min) / max(1, self.span)))

    def tick_from_norm(self, n: float) -> int:
        return int(round(self.range_min + min(1.0, max(0.0, n)) * self.span))
