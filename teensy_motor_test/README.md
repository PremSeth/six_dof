# Teensy 4.0 open-loop motor test

Teensy board pin 23 = STEP/PUL; pin 22 = DIR. Current V3 firmware uses
non-inverted GPIO signals for the user-confirmed common-anode NPN interface:
regulated 5 V to PUL+/DIR+, individual collectors to PUL-/DIR-, emitters to
shared signal ground, and Teensy outputs through 1.5 kohm to each base.
No collector pull-ups. Transistor part number remains unspecified.
These are Teensy board pins, not Raspberry Pi GPIO numbers. Driver motor
power is separate. User reported resolving the Teensy's power wiring after
reworking VIN–VUSB; its physical bridge state has not been inspected.

The firmware starts idle, with both Teensy outputs LOW (transistors off).
Uploading replaces the USB
diagnostic application. No movement occurs until a MOVE command is received.
The Python client prompts for pulse count, motor RPM, DIR level, and Enter to
start each move. DIR 0/1 does not imply a calibrated physical direction.

```bash
cd ~/six_dof/teensy_motor_test
~/.local/bin/micromamba run -n six_dof pio run --target upload
~/.local/bin/micromamba run -n six_dof python motor_test.py --check
~/.local/bin/micromamba run -n six_dof python motor_test.py
```

Default driver setting is 400 pulses/motor revolution. Override using `--ppr`
if the driver's switches differ. This does not account for any gearbox ratio.
Start with a small count at low RPM: this test starts/stops at constant speed
without acceleration and can stall at high speed or load. No automatic reverse.

IntervalTimer schedules each pulse with 5 us high time and a minimum 50 us
period (20 kHz ceiling). Software interrupt latency can stretch the waveform;
physical waveform timing requires a scope or logic analyzer to verify.
DIR settles for 10 ms before pulses start. The protocol limits a move to
10 million pulses. STATUS or PING must arrive within 1.5 seconds while moving;
otherwise pulses stop, including on USB disconnection. The firmware uses
heartbeat age rather than Serial's boolean readiness, which remains false for
15 ms after opening and previously caused a zero-pulse false-stop.
The client sends STATUS every 0.1 seconds. Ctrl+C sends
STOP. These stops end pulse generation, not driver holding torque, and do not
replace a hardware emergency stop. No elapsed-time deadline is imposed on a
move with a healthy connection. Pulse counts are not encoder feedback.

Protocol: `PING`, `STATUS`, `STOP`, `MOVE <pulses> <period_us> <dir_0_or_1>`.
STATUS states: 0 idle, 1 moving, 2 complete, 3 stopped, 4 link lost.

2026-09-24: Uploaded V2, STEP 23/DIR 22 inverted. Explicitly authorized test
generated 20/20 pulses at a 30,000 us period (5 motor RPM at 400 pulses/rev),
then stopped. Actual motor movement awaits user observation. Earlier attempt
stopped at zero pulses due to the corrected Serial readiness check.
