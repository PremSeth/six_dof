#!/usr/bin/env python3
"""Interactively move a STEP/DIR stepper forward and back using pigpio."""

import time

import pigpio


# BCM GPIO numbers for the six-DOF motor driver.
STEP_GPIO = 14
DIR_GPIO = 15

# Swap 1 and 0 if the physical forward direction is reversed.
FORWARD_LEVEL = 1

# A typical 1.8-degree NEMA 23 has 200 full steps/revolution. This must match
# the motor driver's DIP-switch microstep setting.
FULL_STEPS_PER_REV = 200
MICROSTEPS = 16
PULSES_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS

# Conservative timing for common opto-isolated stepper drivers.
STEP_PULSE_US = 5
MIN_LOW_US = 5
DIRECTION_SETTLE_S = 0.01
TURNAROUND_PAUSE_S = 1.0
MAX_RPM = 1200.0
MAX_STEPS = 1_000_000
MAX_CHAIN_LOOP = 65_535


def rpm_to_period_us(rpm: float) -> int:
    """Convert shaft RPM to the period of one driver pulse."""
    pulses_per_second = rpm * PULSES_PER_REV / 60.0
    period_us = round(1_000_000.0 / pulses_per_second)
    if period_us < STEP_PULSE_US + MIN_LOW_US:
        raise ValueError("RPM requires pulses faster than the configured timing allows")
    return period_us


def move_steps(pi: pigpio.pi, count: int, direction_level: int, rpm: float) -> None:
    """Send exactly count DMA-timed step pulses at the requested RPM."""
    pi.write(DIR_GPIO, direction_level)
    time.sleep(DIRECTION_SETTLE_S)

    period_us = rpm_to_period_us(rpm)
    low_us = period_us - STEP_PULSE_US
    step_mask = 1 << STEP_GPIO

    # Define a single step pulse, then repeat its tiny wave with a compact
    # pigpio chain. This avoids overflowing pigpiod with thousands of pulse
    # records when a large step count is requested.
    pi.wave_add_new()
    pi.wave_add_generic(
        [
            pigpio.pulse(step_mask, 0, STEP_PULSE_US),
            pigpio.pulse(0, step_mask, low_us),
        ]
    )
    wave_id = pi.wave_create()
    if wave_id < 0:
        raise RuntimeError(f"pigpio could not create waveform (error {wave_id})")

    chain = []
    remaining = count
    while remaining:
        repetitions = min(remaining, MAX_CHAIN_LOOP)
        chain.extend(
            [
                255,
                0,  # Loop start.
                wave_id,
                255,
                1,  # Loop repeat, encoded as low byte then high byte.
                repetitions & 0xFF,
                repetitions >> 8,
            ]
        )
        remaining -= repetitions

    try:
        pi.wave_chain(chain)
        while pi.wave_tx_busy():
            time.sleep(0.002)
    finally:
        pi.wave_delete(wave_id)

    pi.write(STEP_GPIO, 0)


def prompt_step_count() -> int | None:
    while True:
        raw = input("Number of steps to move forward and back (q to quit): ").strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None

        try:
            steps = int(raw)
        except ValueError:
            print("Please enter a positive whole number or q.")
            continue

        if not 0 < steps <= MAX_STEPS:
            print(f"Please enter a number from 1 through {MAX_STEPS:,}.")
            continue

        return steps


def prompt_rpm() -> float | None:
    while True:
        raw = input(f"Target RPM (1-{MAX_RPM:g}, q to quit): ").strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None

        try:
            rpm = float(raw)
        except ValueError:
            print("Please enter an RPM number or q.")
            continue

        if not 0 < rpm <= MAX_RPM:
            print(f"Please enter an RPM from greater than 0 through {MAX_RPM:g}.")
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

    print(f"STEP=BCM{STEP_GPIO}, DIR=BCM{DIR_GPIO}")
    print(
        f"Motor={FULL_STEPS_PER_REV} full steps/rev, driver=1/{MICROSTEPS} "
        f"microstep, PUL={PULSES_PER_REV} pulses/rev"
    )

    try:
        while True:
            steps = prompt_step_count()
            if steps is None:
                break

            rpm = prompt_rpm()
            if rpm is None:
                break

            pulse_rate = rpm * PULSES_PER_REV / 60.0
            confirmation = input(
                f"Press Enter to move {steps} pulses forward and back at {rpm:g} RPM "
                f"({pulse_rate:.0f} pulses/s). Type q to cancel: "
            ).strip()
            if confirmation.lower() in {"q", "quit", "exit"}:
                print("Cancelled; the motor did not move.")
                continue

            print(f"Moving forward {steps} pulses at {rpm:g} RPM...")
            move_steps(pi, steps, FORWARD_LEVEL, rpm)
            time.sleep(TURNAROUND_PAUSE_S)

            print(f"Moving backward {steps} pulses at {rpm:g} RPM...")
            move_steps(pi, steps, 1 - FORWARD_LEVEL, rpm)
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
