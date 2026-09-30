#!/usr/bin/env python3
"""Test a STEP/DIR motor using a pulse count and motor-shaft RPM."""

import time

import pigpio


# Raspberry Pi BCM GPIO numbers.
DIR_GPIO = 20
STEP_GPIO = 16

# Swap 1 and 0 if the physical forward direction is reversed.
FORWARD_LEVEL = 1

# Motor and driver configuration. Match MICROSTEPS to the driver switches.
FULL_STEPS_PER_REV = 200
MICROSTEPS = 2
PULSES_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS

# Conservative timing for common opto-isolated STEP/DIR drivers.
STEP_PULSE_US = 5
MIN_LOW_US = 5
DIRECTION_SETTLE_S = 0.01
TURNAROUND_PAUSE_S = 1.0
MAX_RPM = 1200.0
MAX_PULSES = 1_000_000
MAX_CHAIN_LOOP = 65_535


def rpm_to_period_us(rpm: float) -> int:
    """Convert motor-shaft RPM to one STEP-pulse period."""
    pulses_per_second = rpm * PULSES_PER_REV / 60.0
    period_us = round(1_000_000.0 / pulses_per_second)
    if period_us < STEP_PULSE_US + MIN_LOW_US:
        raise ValueError("RPM requires pulses faster than the configured timing allows")
    return period_us


def move_pulses(
    pi: pigpio.pi,
    pulse_count: int,
    direction_level: int,
    motor_rpm: float,
) -> None:
    """Send exactly pulse_count pulses at constant motor-shaft RPM."""
    pi.write(DIR_GPIO, direction_level)
    time.sleep(DIRECTION_SETTLE_S)

    period_us = rpm_to_period_us(motor_rpm)
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

    chain: list[int] = []
    remaining = pulse_count
    while remaining:
        repetitions = min(remaining, MAX_CHAIN_LOOP)
        if repetitions == 1:
            chain.append(wave_id)
        else:
            chain.extend(
                [
                    255,
                    0,  # Loop start.
                    wave_id,
                    255,
                    1,  # Loop repeat; low byte then high byte.
                    repetitions & 0xFF,
                    repetitions >> 8,
                ]
            )
        remaining -= repetitions

    try:
        result = pi.wave_chain(chain)
        if result < 0:
            raise RuntimeError(f"pigpio could not transmit STEP chain (error {result})")
        while pi.wave_tx_busy():
            time.sleep(0.002)
    finally:
        pi.wave_tx_stop()
        pi.wave_delete(wave_id)
        pi.write(STEP_GPIO, 0)


def prompt_pulse_count() -> int | None:
    while True:
        raw = input(f"Number of STEP pulses (1-{MAX_PULSES:,}, q to quit): ").strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None
        try:
            pulse_count = int(raw)
        except ValueError:
            print("Enter a positive whole-number pulse count or q.")
            continue
        if not 0 < pulse_count <= MAX_PULSES:
            print(f"Pulse count must be from 1 through {MAX_PULSES:,}.")
            continue
        return pulse_count


def prompt_motor_rpm() -> float | None:
    while True:
        raw = input(f"Motor-shaft RPM (1-{MAX_RPM:g}, q to quit): ").strip()
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

    print(f"Stepper test: PUL/STEP=BCM{STEP_GPIO}, DIR=BCM{DIR_GPIO}")
    print(
        f"Motor={FULL_STEPS_PER_REV} full steps/rev, driver=1/{MICROSTEPS} "
        f"microstep, PUL={PULSES_PER_REV} pulses/motor revolution"
    )

    try:
        while True:
            pulse_count = prompt_pulse_count()
            if pulse_count is None:
                break
            motor_rpm = prompt_motor_rpm()
            if motor_rpm is None:
                break

            pulse_rate = motor_rpm * PULSES_PER_REV / 60.0
            motor_revolutions = pulse_count / PULSES_PER_REV
            confirmation = input(
                f"Press Enter to move {pulse_count:,} pulses "
                f"({motor_revolutions:.4f} motor rev) forward and back at "
                f"{motor_rpm:g} motor RPM ({pulse_rate:.0f} pulses/s). "
                f"Type q to cancel: "
            ).strip()
            if confirmation.lower() in {"q", "quit", "exit"}:
                print("Cancelled; the motor did not move.")
                continue

            print("Moving forward...")
            move_pulses(pi, pulse_count, FORWARD_LEVEL, motor_rpm)
            time.sleep(TURNAROUND_PAUSE_S)

            print("Moving in reverse...")
            move_pulses(pi, pulse_count, 1 - FORWARD_LEVEL, motor_rpm)
            print("Done.\n")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        pi.wave_tx_stop()
        pi.wave_clear()
        pi.write(STEP_GPIO, 0)
        pi.stop()


if __name__ == "__main__":
    main()
