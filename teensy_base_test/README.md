# Standalone base test

## Current base wiring (2026-09-29)

Base STEP2, DIR3; encoder muxport0;3200ppr;2:1;positive DIR level1.
Supersedes old STEP0/DIR1 notes below. The J2 client retains its old pin-specific
handshake and will refuse this firmware, so base changes do not redirect J2.
Use base_velocity_test.py for the base. Firmware must match the selected test.

## J2 using the same temporary test wiring

Run `~/.local/share/mamba/envs/six_dof/bin/python j2_velocity_test.py`
from this same directory for J2: STEP0, DIR1, mux port0, 3200 pulses/rev,
15:1 reduction, positive DIR1 by default. Encoder assumed on joint output.
Base script remains 2:1; choose the correct script for the attached motor.
Both use the existing BASE firmware/ANGLE_BASE protocol names; no firmware
upload is needed to switch. These names identify the shared test connection,
not the physical joint. Set a fresh zero and verify direction slowly for J2.

Teensy 4.0 STEP 0 -> TB6600 PUL+, DIR 1 -> DIR+, shared ground to negative
signal terminals. User reports this direct interface works. Encoder mux port
0 is temporarily assigned to BASE, with J1 disconnected. Driver 3200 pulses
per motor revolution; base reduction 2:1. Encoder assumed on joint output.

Run on Pi:

    cd ~/six_dof/teensy_base_test
    ~/.local/share/mamba/envs/six_dof/bin/python base_velocity_test.py

Enter to zero, enter target joint degrees, enter joint degrees/second, then
confirm to move. Start with 5 degrees at 2 degrees/second. Positive direction
is increasing encoder angle; default DIR 1 was selected by the user. Override
with --positive-dir 0 if needed. z resets session zero; q exits.

Constant commanded speed with encoder stopping, 0.3-degree tolerance/crossing.
No ramp, automatic correction, travel limits, or pulse budget. Stop with Ctrl+C
if stalled. Read errors, tracking gaps, wrong-way >1 degree, and lost USB
heartbeat stop pulses. Stopping pulses does not disable driver holding torque.
Higher speed can overshoot. Use only one serial client at a time.

--check verifies firmware/encoder without motion. motor_test.py is an optional
open-loop pulse-count/motor-RPM test (no reduction applied to motor RPM).

This firmware REPLACES the running J1 test on Teensy; J1 files remain intact.
Different handshake prevents accidentally running the J1 client on base pins.
To restore J1 firmware while everything is idle:

    cd ~/six_dof/teensy_motor_test
    ~/.local/bin/micromamba run -n six_dof pio run --target upload

Firmware retains servo support on 14/15, initially disabled, and rejects
simultaneous servo/stepper tests. J1's standalone servo client expects J1
firmware; restore that project before using that client.
