#!/usr/bin/env python3
"""Run the Joint 2 STEP/DIR motor forward and reverse open-loop."""

import time

import pigpio


# Raspberry Pi BCM GPIO numbers for the Joint 2 motor driver.
STEP_GPIO = 17
DIR_GPIO = 27

# Swap 1 and 0 if the physical forward direction is reversed.
FORWARD_LEVEL = 1

# Motor and driver configuration. Match MICROSTEPS to the driver switches.
FULL_STEPS_PER_REV = 200
MICROSTEPS = 16
PULSES_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS

# Driver timing and test limits.
STEP_PULSE_US = 5
MIN_LOW_US = 5
DIRECTION_SETTLE_S = 0.01
TURNAROUND_PAUSE_S = 1.0
MIN_DURATION_S = 0.1
MAX_DURATION_S = 120.0
MAX_RPM = 1200.0


def rpm_to_period_us(rpm: float) -> int:
    """Convert motor RPM into the period of one STEP pulse."""
    pulses_per_second = rpm * PULSES_PER_REV / 60.0
    period_us = round(1_000_000.0 / pulses_per_second)
    if period_us < STEP_PULSE_US + MIN_LOW_US:
        raise ValueError("RPM requires pulses faster than the configured timing allows")
    return period_us


def create_step_wave(pi: pigpio.pi, rpm: float) -> tuple[int, float]:
    """Create a one-pulse waveform that repeats at constant motor RPM."""
    period_us = rpm_to_period_us(rpm)
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

    actual_rpm = 60_000_000.0 / (period_us * PULSES_PER_REV)
    return wave_id, actual_rpm


def run_for_duration(
    pi: pigpio.pi,
    wave_id: int,
    direction_level: int,
    duration_s: float,
) -> None:
    """Run one direction at constant speed for the requested duration."""
    pi.write(DIR_GPIO, direction_level)
    time.sleep(DIRECTION_SETTLE_S)
    result = pi.wave_send_repeat(wave_id)
    if result < 0:
        raise RuntimeError(f"pigpio could not start STEP waveform (error {result})")

    end_time = time.monotonic() + duration_s
    try:
        while True:
            remaining = end_time - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(remaining, 0.05))
    finally:
        pi.wave_tx_stop()
        pi.write(STEP_GPIO, 0)


def prompt_duration() -> float | None:
    while True:
        raw = input(
            f"Seconds per direction ({MIN_DURATION_S:g}-{MAX_DURATION_S:g}, q to quit): "
        ).strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None
        try:
            duration_s = float(raw)
        except ValueError:
            print("Enter a duration in seconds or q.")
            continue
        if not MIN_DURATION_S <= duration_s <= MAX_DURATION_S:
            print(
                f"Duration must be from {MIN_DURATION_S:g} through "
                f"{MAX_DURATION_S:g} seconds."
            )
            continue
        return duration_s


def prompt_rpm() -> float | None:
    while True:
        raw = input(f"Motor RPM (1-{MAX_RPM:g}, q to quit): ").strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None
        try:
            rpm = float(raw)
        except ValueError:
            print("Enter a motor RPM or q.")
            continue
        if not 0 < rpm <= MAX_RPM:
            print(f"RPM must be greater than 0 and at most {MAX_RPM:g}.")
            continue
        return rpm


def main() -> None:
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("Could not connect to pigpiod. Check: systemctl status pigpiod")

    pi.set_mode(STEP_GPIO, pigpio.OUTPUT)
    pi.set_mode(DIR_GPIO, pigpio.OUTPUT)
    pi.write(STEP_GPIO, 0)
    pi.wave_clear()

    print(f"Joint 2: PUL/STEP=BCM{STEP_GPIO}, DIR=BCM{DIR_GPIO}")
    print(f"Driver setting: {PULSES_PER_REV} pulses/motor revolution")

    try:
        while True:
            duration_s = prompt_duration()
            if duration_s is None:
                break
            rpm = prompt_rpm()
            if rpm is None:
                break

            wave_id, actual_rpm = create_step_wave(pi, rpm)
            estimated_pulses = duration_s * actual_rpm * PULSES_PER_REV / 60.0
            try:
                confirmation = input(
                    f"Press Enter to run Joint 2 forward for {duration_s:g} s, "
                    f"pause, then reverse for {duration_s:g} s at "
                    f"{actual_rpm:.2f} motor RPM (~{estimated_pulses:,.0f} pulses "
                    f"each way). Type q to cancel: "
                ).strip()
                if confirmation.lower() in {"q", "quit", "exit"}:
                    print("Cancelled; the motor did not move.")
                    continue

                print("Running Joint 2 forward...")
                run_for_duration(pi, wave_id, FORWARD_LEVEL, duration_s)
                time.sleep(TURNAROUND_PAUSE_S)

                print("Running Joint 2 in reverse...")
                run_for_duration(pi, wave_id, 1 - FORWARD_LEVEL, duration_s)
                print("Done.\n")
            finally:
                pi.wave_tx_stop()
                pi.wave_delete(wave_id)
                pi.write(STEP_GPIO, 0)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        pi.wave_tx_stop()
        pi.wave_clear()
        pi.write(STEP_GPIO, 0)
        pi.stop()


if __name__ == "__main__":
    main()
