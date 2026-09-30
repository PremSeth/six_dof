#!/usr/bin/env python3
"""Move a STEP/DIR motor forward and reverse with smooth speed ramps."""

import time

import pigpio


# Raspberry Pi BCM GPIO numbers.
STEP_GPIO = 14
DIR_GPIO = 15

# Swap 1 and 0 if the physical forward direction is reversed.
FORWARD_LEVEL = 1

# Motor and driver configuration. These values must match the hardware.
FULL_STEPS_PER_REV = 200
MICROSTEPS = 16
PULSES_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS

# Ten motor revolutions produce one wave-drive output revolution. This is used
# only to report the estimated output motion; pulse generation remains at the
# motor shaft.
GEAR_RATIO = 15.0

# Motion-profile configuration.
START_RPM = 10.0
RAMP_TIME_FRACTION = 0.25
# pigpio permits only 20 loop counters in one wave chain. Six ramp stages give
# a useful speed curve without excessive waveform complexity; long moves are
# split into multiple safe chains below.
RAMP_STAGES = 6
TURNAROUND_PAUSE_S = 1.0

# Conservative timing for common opto-isolated STEP/DIR drivers.
STEP_PULSE_US = 5
MIN_LOW_US = 5
DIRECTION_SETTLE_S = 0.01
MAX_RPM = 1200.0
MIN_DURATION_S = 1.0
MAX_DURATION_S = 120.0
MAX_CHAIN_LOOP = 65_535
MAX_CHAIN_COUNTERS = 20


def rpm_to_period_us(rpm: float) -> int:
    """Convert motor RPM to the period of one STEP pulse."""
    pulses_per_second = rpm * PULSES_PER_REV / 60.0
    period_us = round(1_000_000.0 / pulses_per_second)
    if period_us < STEP_PULSE_US + MIN_LOW_US:
        raise ValueError("RPM requires pulses faster than the configured timing allows")
    return period_us


def make_timed_profile(duration_s: float, peak_rpm: float) -> list[tuple[int, float]]:
    """Build a timed S-curve-like ramp up, cruise, and ramp down profile."""
    start_rpm = min(START_RPM, peak_rpm)
    if peak_rpm <= start_rpm:
        pulse_count = max(1, round(duration_s * peak_rpm * PULSES_PER_REV / 60.0))
        return [(pulse_count, peak_rpm)]

    ramp_duration_s = duration_s * RAMP_TIME_FRACTION
    stage_duration_s = ramp_duration_s / RAMP_STAGES
    stage_speeds = []

    for index in range(RAMP_STAGES):
        position = index / (RAMP_STAGES - 1)
        smooth_position = position * position * (3.0 - 2.0 * position)
        stage_speeds.append(start_rpm + (peak_rpm - start_rpm) * smooth_position)

    stage_counts = [
        max(1, round(stage_duration_s * rpm * PULSES_PER_REV / 60.0))
        for rpm in stage_speeds
    ]
    ramp_up = list(zip(stage_counts, stage_speeds))
    cruise_duration_s = duration_s - 2.0 * ramp_duration_s
    cruise_pulses = round(cruise_duration_s * peak_rpm * PULSES_PER_REV / 60.0)
    profile = ramp_up.copy()
    if cruise_pulses:
        profile.append((cruise_pulses, peak_rpm))
    profile.extend(reversed(ramp_up))

    # Merge neighboring stages that round to the same pigpio period.
    merged: list[tuple[int, float]] = []
    for count, rpm in profile:
        if merged and rpm_to_period_us(merged[-1][1]) == rpm_to_period_us(rpm):
            previous_count, previous_rpm = merged[-1]
            merged[-1] = (previous_count + count, previous_rpm)
        else:
            merged.append((count, rpm))
    return merged


def estimate_duration_s(profile: list[tuple[int, float]]) -> float:
    """Estimate the duration of a profile from its quantized pulse periods."""
    return sum(count * rpm_to_period_us(rpm) / 1_000_000.0 for count, rpm in profile)


def required_chain_counters(profile: list[tuple[int, float]]) -> int:
    """Return the number of pigpio loop counters needed by a profile."""
    return sum(
        0 if count == 1 else (count + MAX_CHAIN_LOOP - 1) // MAX_CHAIN_LOOP
        for count, _ in profile
    )


def move_profile(
    pi: pigpio.pi,
    profile: list[tuple[int, float]],
    direction_level: int,
) -> None:
    """Transmit a complete speed profile in one direction."""
    pi.write(DIR_GPIO, direction_level)
    time.sleep(DIRECTION_SETTLE_S)

    step_mask = 1 << STEP_GPIO
    wave_for_period: dict[int, int] = {}
    wave_ids: list[int] = []
    chains: list[list[int]] = [[]]
    counters_in_chain = 0

    try:
        for count, rpm in profile:
            period_us = rpm_to_period_us(rpm)
            wave_id = wave_for_period.get(period_us)
            if wave_id is None:
                pi.wave_add_new()
                pi.wave_add_generic(
                    [
                        pigpio.pulse(step_mask, 0, STEP_PULSE_US),
                        pigpio.pulse(0, step_mask, period_us - STEP_PULSE_US),
                    ]
                )
                wave_id = pi.wave_create()
                if wave_id < 0:
                    raise RuntimeError(
                        f"pigpio could not create ramp waveform (error {wave_id})"
                    )
                wave_for_period[period_us] = wave_id
                wave_ids.append(wave_id)

            remaining = count
            while remaining:
                repetitions = min(remaining, MAX_CHAIN_LOOP)
                if repetitions == 1:
                    command = [wave_id]
                    counter_cost = 0
                else:
                    command = [
                        255,
                        0,  # Loop start.
                        wave_id,
                        255,
                        1,  # Loop repeat; low byte then high byte.
                        repetitions & 0xFF,
                        repetitions >> 8,
                    ]
                    counter_cost = 1

                if counter_cost and counters_in_chain == MAX_CHAIN_COUNTERS:
                    chains.append([])
                    counters_in_chain = 0
                chains[-1].extend(command)
                counters_in_chain += counter_cost
                remaining -= repetitions

        # Long, fast moves may need more counters than pigpio permits in one
        # chain. Transmit safe batches sequentially instead of failing.
        for chain in chains:
            result = pi.wave_chain(chain)
            if result < 0:
                raise RuntimeError(
                    f"pigpio could not transmit ramp chain (error {result})"
                )
            while pi.wave_tx_busy():
                time.sleep(0.002)
    finally:
        pi.wave_tx_stop()
        for wave_id in wave_ids:
            pi.wave_delete(wave_id)
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
            print("Please enter a duration in seconds or q.")
            continue
        if not MIN_DURATION_S <= duration_s <= MAX_DURATION_S:
            print(
                f"Please enter a duration from {MIN_DURATION_S:g} through "
                f"{MAX_DURATION_S:g} seconds."
            )
            continue
        return duration_s


def prompt_peak_rpm() -> float | None:
    while True:
        raw = input(f"Peak motor RPM (1-{MAX_RPM:g}, q to quit): ").strip()
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
        f"Motor={PULSES_PER_REV} pulses/rev, gearbox={GEAR_RATIO:g}:1 reduction, "
        f"output={PULSES_PER_REV * GEAR_RATIO:,.0f} pulses/rev"
    )

    try:
        while True:
            requested_duration_s = prompt_duration()
            if requested_duration_s is None:
                break
            peak_rpm = prompt_peak_rpm()
            if peak_rpm is None:
                break

            profile = make_timed_profile(requested_duration_s, peak_rpm)
            total_pulses = sum(count for count, _ in profile)
            output_degrees = total_pulses * 360.0 / (PULSES_PER_REV * GEAR_RATIO)
            direction_time = estimate_duration_s(profile)
            confirmation = input(
                f"Press Enter to run forward for ~{direction_time:.2f} s, pause, "
                f"then reverse for ~{direction_time:.2f} s at up to "
                f"{peak_rpm:g} motor RPM. Estimated travel is "
                f"{output_degrees:.2f} output degrees each way. Type q to cancel: "
            ).strip()
            if confirmation.lower() in {"q", "quit", "exit"}:
                print("Cancelled; the motor did not move.")
                continue

            print(f"Ramping forward to {peak_rpm:g} motor RPM and back down...")
            move_profile(pi, profile, FORWARD_LEVEL)
            time.sleep(TURNAROUND_PAUSE_S)

            print(f"Ramping in reverse to {peak_rpm:g} motor RPM and back down...")
            move_profile(pi, profile, 1 - FORWARD_LEVEL)
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
