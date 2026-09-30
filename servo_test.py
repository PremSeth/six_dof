#!/usr/bin/env python3
"""Interactively test a hobby servo on Raspberry Pi BCM GPIO 11."""

import pigpio


SERVO_GPIO = 11

# goBILDA 2000 Series Dual Mode Servo (25-2, Torque), default position mode.
MIN_ANGLE_DEG = 0.0
MAX_ANGLE_DEG = 300.0
MIN_PULSE_US = 500
MAX_PULSE_US = 2500
CENTER_PULSE_US = 1500


def angle_to_pulse(angle_deg: float) -> int:
    """Map the servo's 0-300 degree range onto its specified pulse range."""
    span = MAX_PULSE_US - MIN_PULSE_US
    return round(
        MIN_PULSE_US
        + (angle_deg - MIN_ANGLE_DEG) * span / (MAX_ANGLE_DEG - MIN_ANGLE_DEG)
    )


def main() -> None:
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("Could not connect to pigpiod. Check: systemctl status pigpiod")

    pi.set_mode(SERVO_GPIO, pigpio.OUTPUT)
    pi.set_servo_pulsewidth(SERVO_GPIO, 0)

    print(f"Servo signal: BCM GPIO {SERVO_GPIO}")
    print(
        f"Angle range {MIN_ANGLE_DEG:g}-{MAX_ANGLE_DEG:g} maps to "
        f"{MIN_PULSE_US}-{MAX_PULSE_US} us; "
        f"center is {CENTER_PULSE_US} us."
    )
    print(
        f"Commands: angle {MIN_ANGLE_DEG:g}-{MAX_ANGLE_DEG:g}, "
        "p <microseconds>, c=center, off, q=quit"
    )

    try:
        while True:
            raw = input("servo> ").strip().lower()
            if raw in {"q", "quit", "exit"}:
                break
            if raw == "off":
                pi.set_servo_pulsewidth(SERVO_GPIO, 0)
                print("Servo signal disabled.")
                continue
            if raw in {"c", "center"}:
                pulse_us = CENTER_PULSE_US
                label = "center"
            elif raw.startswith("p "):
                try:
                    pulse_us = int(raw.split(maxsplit=1)[1])
                except (ValueError, IndexError):
                    print(f"Enter p followed by {MIN_PULSE_US}-{MAX_PULSE_US} microseconds.")
                    continue
                if not MIN_PULSE_US <= pulse_us <= MAX_PULSE_US:
                    print(f"Pulse must be from {MIN_PULSE_US} through {MAX_PULSE_US} us.")
                    continue
                label = f"{pulse_us} us"
            else:
                try:
                    angle_deg = float(raw)
                except ValueError:
                    print(
                        f"Enter {MIN_ANGLE_DEG:g}-{MAX_ANGLE_DEG:g}, "
                        "p <microseconds>, c, off, or q."
                    )
                    continue
                if not MIN_ANGLE_DEG <= angle_deg <= MAX_ANGLE_DEG:
                    print(
                        f"Angle must be from {MIN_ANGLE_DEG:g} through "
                        f"{MAX_ANGLE_DEG:g} degrees."
                    )
                    continue
                pulse_us = angle_to_pulse(angle_deg)
                label = f"{angle_deg:g} degrees"

            result = pi.set_servo_pulsewidth(SERVO_GPIO, pulse_us)
            if result < 0:
                raise RuntimeError(f"pigpio rejected the servo pulse (error {result})")
            print(f"Commanded {label}: {pulse_us} us on BCM GPIO {SERVO_GPIO}.")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        pi.set_servo_pulsewidth(SERVO_GPIO, 0)
        pi.stop()
        print("Servo signal disabled.")


if __name__ == "__main__":
    main()
