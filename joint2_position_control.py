#!/usr/bin/env python3
"""Interactive constant-velocity position control for six_dof Joint 2."""

import argparse
import select
import sys
import time

import pigpio
from smbus2 import SMBus

from as5600_mux_test import find_mux, read_as5600


# Joint 2 hardware configuration (BCM GPIO numbering).
STEP_GPIO = 17
DIR_GPIO = 27
ENCODER_MUX_PORT = 2

# Motor, driver, and fixed gearbox configuration.
FULL_STEPS_PER_REV = 200
MICROSTEPS = 16
MOTOR_PULSES_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS
GEAR_RATIO = 15.0
OUTPUT_PULSES_PER_REV = MOTOR_PULSES_PER_REV * GEAR_RATIO

# Motion and safety configuration.
MIN_JOINT_RPM = 0.1
MAX_JOINT_RPM = 80.0
DEFAULT_MIN_ANGLE_DEG = -170.0
DEFAULT_MAX_ANGLE_DEG = 170.0
POSITION_TOLERANCE_DEG = 0.25
POLL_INTERVAL_S = 0.02
DIRECTION_SETTLE_S = 0.01
DIRECTION_CHECK_AFTER_S = 0.75
MIN_DIRECTION_PROGRESS_DEG = 0.20
STEP_PULSE_US = 5
MIN_LOW_US = 5


class AngleTracker:
    """Track continuous joint angle relative to a captured single-turn zero."""

    def __init__(self, zero_absolute_deg: float) -> None:
        self.last_absolute_deg = zero_absolute_deg
        self.position_deg = 0.0

    def update(self, absolute_deg: float) -> float:
        change = (absolute_deg - self.last_absolute_deg + 180.0) % 360.0 - 180.0
        self.position_deg += change
        self.last_absolute_deg = absolute_deg
        return self.position_deg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture Joint 2 zero from the mux-port-2 AS5600, then move to "
            "target angles at a requested joint-output RPM."
        )
    )
    parser.add_argument(
        "--min-angle",
        type=float,
        default=DEFAULT_MIN_ANGLE_DEG,
        help=f"lower software limit (default: {DEFAULT_MIN_ANGLE_DEG:g})",
    )
    parser.add_argument(
        "--max-angle",
        type=float,
        default=DEFAULT_MAX_ANGLE_DEG,
        help=f"upper software limit (default: {DEFAULT_MAX_ANGLE_DEG:g})",
    )
    parser.add_argument(
        "--invert-direction",
        action="store_true",
        help="invert the default DIR level for increasing joint angle",
    )
    args = parser.parse_args()

    if args.min_angle >= args.max_angle:
        parser.error("--min-angle must be less than --max-angle")
    if args.max_angle - args.min_angle >= 360.0:
        parser.error("the software-limit span must be less than 360 degrees")
    return args


def read_angle(bus: SMBus) -> tuple[float, int, int]:
    """Read the AS5600 angle and status without gating on magnet flags."""
    status, _, angle_raw = read_as5600(bus)
    return angle_raw * 360.0 / 4096.0, angle_raw, status


def capture_zero(bus: SMBus) -> float:
    """Display live absolute angle until Enter captures the session zero."""
    print("Move Joint 2 to the desired zero position.")
    print("Press Enter to capture zero, or type q then Enter to quit.")
    latest_angle: float | None = None
    latest_raw: int | None = None

    while True:
        try:
            latest_angle, latest_raw, status = read_angle(bus)
            display = (
                f"absolute={latest_angle:7.2f} deg raw={latest_raw:4d} "
                f"status=0x{status:02X}"
            )
        except OSError as error:
            latest_angle = None
            latest_raw = None
            display = f"encoder not ready: {error}"

        print(f"\r{display:<90}", end="", flush=True)
        readable, _, _ = select.select([sys.stdin], [], [], 0.10)
        if not readable:
            continue

        response = sys.stdin.readline().strip().lower()
        print()
        if response in {"q", "quit", "exit"}:
            raise KeyboardInterrupt
        if latest_angle is None or latest_raw is None:
            print("Cannot capture zero until the encoder returns an angle value.")
            continue
        print(f"Captured Joint 2 zero: absolute={latest_angle:.2f} deg, raw={latest_raw}")
        return latest_angle


def make_constant_speed_wave(
    pi: pigpio.pi,
    joint_rpm: float,
) -> tuple[int, float]:
    """Create a repeating STEP waveform for a requested joint-output RPM."""
    pulses_per_second = joint_rpm * OUTPUT_PULSES_PER_REV / 60.0
    period_us = round(1_000_000.0 / pulses_per_second)
    if period_us < STEP_PULSE_US + MIN_LOW_US:
        raise ValueError("requested joint RPM exceeds STEP pulse timing")

    step_mask = 1 << STEP_GPIO
    pi.wave_add_new()
    pi.wave_add_generic(
        [
            pigpio.pulse(step_mask, 0, STEP_PULSE_US),
            pigpio.pulse(0, step_mask, period_us - STEP_PULSE_US),
        ]
    )
    wave_id = pi.wave_create()
    if wave_id < 0:
        raise RuntimeError(f"pigpio could not create STEP waveform (error {wave_id})")

    actual_joint_rpm = 60_000_000.0 / (period_us * OUTPUT_PULSES_PER_REV)
    return wave_id, actual_joint_rpm


def move_to_target(
    pi: pigpio.pi,
    bus: SMBus,
    tracker: AngleTracker,
    wave_id: int,
    target_deg: float,
    positive_direction_level: int,
) -> float:
    """Move toward target with constant pulse rate and encoder supervision."""
    absolute_deg, _, _ = read_angle(bus)
    current_deg = tracker.update(absolute_deg)
    initial_error = target_deg - current_deg
    initial_distance = abs(initial_error)
    if initial_distance <= POSITION_TOLERANCE_DEG:
        print(f"Already at {current_deg:.2f} deg.")
        return current_deg

    direction_sign = 1 if initial_error > 0 else -1
    direction_level = (
        positive_direction_level if direction_sign > 0 else 1 - positive_direction_level
    )
    pi.write(DIR_GPIO, direction_level)
    time.sleep(DIRECTION_SETTLE_S)
    result = pi.wave_send_repeat(wave_id)
    if result < 0:
        raise RuntimeError(f"pigpio could not start STEP waveform (error {result})")

    start_time = time.monotonic()
    try:
        while True:
            time.sleep(POLL_INTERVAL_S)
            absolute_deg, _, _ = read_angle(bus)
            current_deg = tracker.update(absolute_deg)
            error = target_deg - current_deg
            print(
                f"\rposition={current_deg:8.2f} deg target={target_deg:8.2f} deg "
                f"error={error:8.2f} deg",
                end="",
                flush=True,
            )

            if abs(error) <= POSITION_TOLERANCE_DEG or direction_sign * error <= 0:
                break

            elapsed = time.monotonic() - start_time
            progress = initial_distance - abs(error)
            if elapsed >= DIRECTION_CHECK_AFTER_S and progress < MIN_DIRECTION_PROGRESS_DEG:
                raise RuntimeError(
                    "encoder position is not approaching the target; stop and rerun "
                    "with --invert-direction if the motor moved the wrong way"
                )
    finally:
        pi.wave_tx_stop()
        pi.write(STEP_GPIO, 0)
        print()

    absolute_deg, _, _ = read_angle(bus)
    current_deg = tracker.update(absolute_deg)
    print(f"Stopped at {current_deg:.2f} deg (target {target_deg:.2f} deg).")
    return current_deg


def prompt_targets(min_angle: float, max_angle: float) -> list[float] | None:
    while True:
        raw = input(
            f"Target angle or comma-separated sequence ({min_angle:g} to "
            f"{max_angle:g} deg, q to quit): "
        ).strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None
        entries = [entry.strip() for entry in raw.split(",")]
        if not entries or any(not entry for entry in entries):
            print("Enter one angle or a comma-separated sequence, such as 20,-15,0.")
            continue
        try:
            targets = [float(entry) for entry in entries]
        except ValueError:
            print("Every target must be an angle in degrees.")
            continue
        invalid_targets = [
            target for target in targets if not min_angle <= target <= max_angle
        ]
        if invalid_targets:
            invalid_text = ", ".join(f"{target:g}" for target in invalid_targets)
            print(
                f"Targets must be between {min_angle:g} and {max_angle:g} "
                f"degrees; invalid: {invalid_text}."
            )
            continue
        return targets


def prompt_joint_rpm() -> float | None:
    while True:
        raw = input(
            f"Joint 2 output RPM ({MIN_JOINT_RPM:g} to {MAX_JOINT_RPM:g}, q to quit): "
        ).strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None
        try:
            joint_rpm = float(raw)
        except ValueError:
            print("Enter the joint-output RPM or q.")
            continue
        if not MIN_JOINT_RPM <= joint_rpm <= MAX_JOINT_RPM:
            print(
                f"Joint RPM must be between {MIN_JOINT_RPM:g} and "
                f"{MAX_JOINT_RPM:g}."
            )
            continue
        return joint_rpm


def main() -> int:
    args = parse_args()
    # Increasing joint angle uses the normal DIR level unless explicitly inverted.
    positive_direction_level = 0 if args.invert_direction else 1
    pi: pigpio.pi | None = None
    wave_id: int | None = None

    try:
        with SMBus(1) as bus:
            mux_address, original_mux_control = find_mux(bus)
            try:
                port_mask = 1 << ENCODER_MUX_PORT
                bus.write_byte(mux_address, port_mask)
                time.sleep(0.01)
                print(
                    f"Joint 2: STEP=BCM{STEP_GPIO}, DIR=BCM{DIR_GPIO}, "
                    f"encoder=mux 0x{mux_address:02X} port {ENCODER_MUX_PORT}"
                )
                zero_absolute_deg = capture_zero(bus)
                tracker = AngleTracker(zero_absolute_deg)

                pi = pigpio.pi()
                if not pi.connected:
                    raise RuntimeError(
                        "Could not connect to pigpiod. Check: systemctl status pigpiod"
                    )
                pi.set_mode(STEP_GPIO, pigpio.OUTPUT)
                pi.set_mode(DIR_GPIO, pigpio.OUTPUT)
                pi.write(STEP_GPIO, 0)
                pi.wave_clear()
                print(
                    f"Fixed gear ratio: {GEAR_RATIO:g}:1; software limits: "
                    f"{args.min_angle:g} to {args.max_angle:g} deg"
                )

                while True:
                    targets = prompt_targets(args.min_angle, args.max_angle)
                    if targets is None:
                        break
                    joint_rpm = prompt_joint_rpm()
                    if joint_rpm is None:
                        break
                    wave_id, actual_joint_rpm = make_constant_speed_wave(pi, joint_rpm)
                    print(
                        f"Moving at {actual_joint_rpm:.3f} joint RPM "
                        f"({actual_joint_rpm * GEAR_RATIO:.3f} motor RPM)."
                    )
                    try:
                        for move_number, target in enumerate(targets, start=1):
                            if len(targets) > 1:
                                print(
                                    f"Sequence move {move_number}/{len(targets)}: "
                                    f"target {target:g} deg"
                                )
                            move_to_target(
                                pi,
                                bus,
                                tracker,
                                wave_id,
                                target,
                                positive_direction_level,
                            )
                    finally:
                        pi.wave_tx_stop()
                        pi.write(STEP_GPIO, 0)
                        pi.wave_delete(wave_id)
                        wave_id = None
            finally:
                if pi is not None:
                    pi.wave_tx_stop()
                    pi.write(STEP_GPIO, 0)
                    if wave_id is not None:
                        pi.wave_delete(wave_id)
                    pi.wave_clear()
                    pi.stop()
                bus.write_byte(mux_address, original_mux_control)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        print(f"Joint 2 control failed: {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
