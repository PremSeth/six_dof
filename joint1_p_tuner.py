#!/usr/bin/env python3
"""Interactively tune a position P controller for six_dof Joint 1."""

import argparse
import math
import sys
import time

import pigpio
from smbus2 import SMBus

import joint1_position_control as joint1
from as5600_mux_test import find_mux


DEFAULT_KP = 1.0
MAX_KP = 100.0
DEFAULT_MAX_VELOCITY_DEG_S = 120.0
MAX_ALLOWED_VELOCITY_DEG_S = 120.0
MIN_COMMAND_VELOCITY_DEG_S = 1.0
VELOCITY_INCREMENT_DEG_S = 1.0
DEFAULT_TOLERANCE_DEG = 0.5
DEFAULT_SETTLE_TIME_S = 0.3
CONTROL_INTERVAL_S = 0.02
DIRECTION_CHECK_AFTER_S = 0.12
MIN_DIRECTION_PROGRESS_DEG = 0.05


class VelocityWaveBank:
    """Pre-create discrete constant-speed pigpio waves for fast P updates."""

    def __init__(self, pi: pigpio.pi, max_velocity_deg_s: float) -> None:
        self.pi = pi
        self.entries: list[tuple[float, int]] = []
        self._build(max_velocity_deg_s)

    def _build(self, max_velocity_deg_s: float) -> None:
        period_to_wave: dict[int, int] = {}
        step_mask = 1 << joint1.STEP_GPIO
        count = math.ceil(max_velocity_deg_s / VELOCITY_INCREMENT_DEG_S)

        for index in range(1, count + 1):
            requested_velocity = min(
                index * VELOCITY_INCREMENT_DEG_S,
                max_velocity_deg_s,
            )
            pulses_per_second = (
                requested_velocity * joint1.OUTPUT_PULSES_PER_REV / 360.0
            )
            period_us = round(1_000_000.0 / pulses_per_second)
            if period_us < joint1.STEP_PULSE_US + joint1.MIN_LOW_US:
                raise ValueError(
                    f"{requested_velocity:g} deg/s exceeds STEP pulse timing"
                )
            if period_us in period_to_wave:
                continue

            self.pi.wave_add_new()
            self.pi.wave_add_generic(
                [
                    pigpio.pulse(step_mask, 0, joint1.STEP_PULSE_US),
                    pigpio.pulse(
                        0,
                        step_mask,
                        period_us - joint1.STEP_PULSE_US,
                    ),
                ]
            )
            wave_id = self.pi.wave_create()
            if wave_id < 0:
                raise RuntimeError(
                    f"pigpio could not create velocity waveform (error {wave_id})"
                )
            period_to_wave[period_us] = wave_id
            actual_velocity = (
                1_000_000.0
                * 360.0
                / (period_us * joint1.OUTPUT_PULSES_PER_REV)
            )
            self.entries.append((actual_velocity, wave_id))

        self.entries.sort()
        if not self.entries:
            raise RuntimeError("no velocity waveforms were created")

    def closest(self, requested_velocity_deg_s: float) -> tuple[float, int]:
        bounded = max(MIN_COMMAND_VELOCITY_DEG_S, requested_velocity_deg_s)
        return min(self.entries, key=lambda entry: abs(entry[0] - bounded))


class StepperVelocityOutput:
    """Apply signed velocity commands using the pre-created wave bank."""

    def __init__(
        self,
        pi: pigpio.pi,
        wave_bank: VelocityWaveBank,
        positive_direction_level: int,
    ) -> None:
        self.pi = pi
        self.wave_bank = wave_bank
        self.positive_direction_level = positive_direction_level
        self.active_wave_id: int | None = None
        self.active_direction_level: int | None = None

    def stop(self) -> None:
        self.pi.wave_tx_stop()
        self.pi.write(joint1.STEP_GPIO, 0)
        self.active_wave_id = None
        self.active_direction_level = None

    def command(self, signed_velocity_deg_s: float) -> float:
        if signed_velocity_deg_s == 0:
            self.stop()
            return 0.0

        direction_level = (
            self.positive_direction_level
            if signed_velocity_deg_s > 0
            else 1 - self.positive_direction_level
        )
        actual_speed, wave_id = self.wave_bank.closest(abs(signed_velocity_deg_s))

        if direction_level != self.active_direction_level:
            self.stop()
            self.pi.write(joint1.DIR_GPIO, direction_level)
            time.sleep(joint1.DIRECTION_SETTLE_S)
            result = self.pi.wave_send_repeat(wave_id)
        elif wave_id != self.active_wave_id:
            result = self.pi.wave_send_using_mode(
                wave_id,
                pigpio.WAVE_MODE_REPEAT_SYNC,
            )
        else:
            result = 0

        if result < 0:
            raise RuntimeError(f"pigpio could not apply velocity wave (error {result})")

        self.active_wave_id = wave_id
        self.active_direction_level = direction_level
        return actual_speed if signed_velocity_deg_s > 0 else -actual_speed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture Joint 1 zero, then repeatedly test position P gains. "
            "Velocity command = Kp * position error."
        )
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=DEFAULT_KP,
        help=f"initial proportional gain in 1/s (default: {DEFAULT_KP:g})",
    )
    parser.add_argument(
        "--max-velocity",
        type=float,
        default=DEFAULT_MAX_VELOCITY_DEG_S,
        help=(
            "maximum absolute joint velocity in degrees/second "
            f"(default: {DEFAULT_MAX_VELOCITY_DEG_S:g})"
        ),
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE_DEG,
        help=f"target tolerance in degrees (default: {DEFAULT_TOLERANCE_DEG:g})",
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=DEFAULT_SETTLE_TIME_S,
        help=f"time required inside tolerance (default: {DEFAULT_SETTLE_TIME_S:g} s)",
    )
    parser.add_argument(
        "--min-angle",
        type=float,
        default=joint1.DEFAULT_MIN_ANGLE_DEG,
        help=f"lower software limit (default: {joint1.DEFAULT_MIN_ANGLE_DEG:g})",
    )
    parser.add_argument(
        "--max-angle",
        type=float,
        default=joint1.DEFAULT_MAX_ANGLE_DEG,
        help=f"upper software limit (default: {joint1.DEFAULT_MAX_ANGLE_DEG:g})",
    )
    parser.add_argument(
        "--invert-direction",
        action="store_true",
        help="invert the DIR level used for increasing Joint 1 angle",
    )
    args = parser.parse_args()

    if not 0 < args.kp <= MAX_KP:
        parser.error(f"--kp must be greater than 0 and at most {MAX_KP:g}")
    if not MIN_COMMAND_VELOCITY_DEG_S <= args.max_velocity <= MAX_ALLOWED_VELOCITY_DEG_S:
        parser.error(
            f"--max-velocity must be {MIN_COMMAND_VELOCITY_DEG_S:g} through "
            f"{MAX_ALLOWED_VELOCITY_DEG_S:g}"
        )
    if args.tolerance <= 0:
        parser.error("--tolerance must be greater than 0")
    if args.settle_time < 0:
        parser.error("--settle-time cannot be negative")
    if args.min_angle >= args.max_angle:
        parser.error("--min-angle must be less than --max-angle")
    if args.max_angle - args.min_angle >= 360.0:
        parser.error("the software-limit span must be less than 360 degrees")
    return args


def prompt_kp(current_kp: float) -> float | None:
    while True:
        raw = input(
            f"P gain Kp in 1/s (Enter keeps {current_kp:g}, q to quit): "
        ).strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None
        if not raw:
            return current_kp
        try:
            kp = float(raw)
        except ValueError:
            print("Enter a positive P gain, press Enter, or type q.")
            continue
        if not 0 < kp <= MAX_KP:
            print(f"Kp must be greater than 0 and at most {MAX_KP:g}.")
            continue
        return kp


def prompt_target(min_angle: float, max_angle: float) -> float | None:
    while True:
        raw = input(
            f"Target Joint 1 angle ({min_angle:g} to {max_angle:g} deg, q to quit): "
        ).strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None
        try:
            target = float(raw)
        except ValueError:
            print("Enter a target angle or q.")
            continue
        if not min_angle <= target <= max_angle:
            print(f"Target must be between {min_angle:g} and {max_angle:g} degrees.")
            continue
        return target


def run_trial(
    bus: SMBus,
    tracker: joint1.AngleTracker,
    motor: StepperVelocityOutput,
    kp: float,
    max_velocity_deg_s: float,
    target_deg: float,
    tolerance_deg: float,
    settle_time_s: float,
) -> None:
    absolute_deg, _, _ = joint1.read_angle(bus)
    position_deg = tracker.update(absolute_deg)
    initial_error = target_deg - position_deg
    initial_distance = abs(initial_error)
    if initial_distance <= tolerance_deg:
        print(f"Already within tolerance at {position_deg:.2f} deg.")
        return

    initial_direction = 1 if initial_error > 0 else -1
    max_overshoot_deg = 0.0
    peak_command_deg_s = 0.0
    settled_since: float | None = None
    direction_verified = False
    start_time = time.monotonic()

    try:
        while True:
            absolute_deg, _, status = joint1.read_angle(bus)
            position_deg = tracker.update(absolute_deg)
            error_deg = target_deg - position_deg
            elapsed = time.monotonic() - start_time

            if abs(error_deg) <= tolerance_deg:
                applied_velocity = motor.command(0.0)
                if settled_since is None:
                    settled_since = time.monotonic()
                if time.monotonic() - settled_since >= settle_time_s:
                    break
            else:
                settled_since = None
                requested_speed = min(max_velocity_deg_s, kp * abs(error_deg))
                requested_speed = max(MIN_COMMAND_VELOCITY_DEG_S, requested_speed)
                signed_velocity = requested_speed if error_deg > 0 else -requested_speed
                applied_velocity = motor.command(signed_velocity)
                peak_command_deg_s = max(peak_command_deg_s, abs(applied_velocity))

            overshoot = initial_direction * (position_deg - target_deg)
            max_overshoot_deg = max(max_overshoot_deg, overshoot)
            print(
                f"\rt={elapsed:6.2f}s pos={position_deg:8.2f} target={target_deg:8.2f} "
                f"error={error_deg:8.2f} cmd={applied_velocity:7.2f} deg/s "
                f"Kp={kp:g} status=0x{status:02X}",
                end="",
                flush=True,
            )

            if not direction_verified and elapsed >= DIRECTION_CHECK_AFTER_S:
                progress = initial_distance - abs(error_deg)
                if progress < MIN_DIRECTION_PROGRESS_DEG:
                    raise RuntimeError(
                        "encoder position is not approaching the target; rerun with "
                        "--invert-direction if the motor moved the wrong way"
                    )
                direction_verified = True

            time.sleep(CONTROL_INTERVAL_S)
    finally:
        motor.stop()
        print()

    elapsed = time.monotonic() - start_time
    absolute_deg, _, _ = joint1.read_angle(bus)
    final_position = tracker.update(absolute_deg)
    print(
        f"Trial complete: Kp={kp:g}, elapsed={elapsed:.2f}s, "
        f"final={final_position:.2f} deg, final error={target_deg-final_position:.2f} deg, "
        f"max overshoot={max_overshoot_deg:.2f} deg, "
        f"peak command={peak_command_deg_s:.2f} deg/s"
    )


def main() -> int:
    args = parse_args()
    positive_direction_level = 0 if args.invert_direction else 1
    pi: pigpio.pi | None = None
    motor: StepperVelocityOutput | None = None

    try:
        with SMBus(1) as bus:
            mux_address, original_mux_control = find_mux(bus)
            try:
                bus.write_byte(mux_address, 1 << joint1.ENCODER_MUX_PORT)
                time.sleep(0.01)
                print(
                    f"Joint 1 P tuner: STEP=BCM{joint1.STEP_GPIO}, "
                    f"DIR=BCM{joint1.DIR_GPIO}, encoder=mux port "
                    f"{joint1.ENCODER_MUX_PORT}, ratio={joint1.GEAR_RATIO:g}:1"
                )
                zero_absolute_deg = joint1.capture_zero(bus)
                tracker = joint1.AngleTracker(zero_absolute_deg)

                pi = pigpio.pi()
                if not pi.connected:
                    raise RuntimeError(
                        "Could not connect to pigpiod. Check: systemctl status pigpiod"
                    )
                pi.set_mode(joint1.STEP_GPIO, pigpio.OUTPUT)
                pi.set_mode(joint1.DIR_GPIO, pigpio.OUTPUT)
                pi.write(joint1.STEP_GPIO, 0)
                pi.wave_clear()
                print("Building velocity waveform bank...")
                wave_bank = VelocityWaveBank(pi, args.max_velocity)
                motor = StepperVelocityOutput(
                    pi,
                    wave_bank,
                    positive_direction_level,
                )
                print(
                    f"P command: velocity = Kp * error, capped at "
                    f"{args.max_velocity:g} deg/s; tolerance={args.tolerance:g} deg"
                )

                current_kp = args.kp
                while True:
                    selected_kp = prompt_kp(current_kp)
                    if selected_kp is None:
                        break
                    current_kp = selected_kp
                    target = prompt_target(args.min_angle, args.max_angle)
                    if target is None:
                        break
                    run_trial(
                        bus,
                        tracker,
                        motor,
                        current_kp,
                        args.max_velocity,
                        target,
                        args.tolerance,
                        args.settle_time,
                    )
            finally:
                if motor is not None:
                    motor.stop()
                if pi is not None:
                    pi.wave_clear()
                    pi.write(joint1.STEP_GPIO, 0)
                    pi.stop()
                bus.write_byte(mux_address, original_mux_control)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        print(f"Joint 1 P tuning failed: {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
