# J1 velocity-to-position test

Configuration: Teensy STEP 23, DIR 22, common-anode NPN interface;
AS5600 on mux port 1 (user confirmed 2026-09-29); 3200 pulses/motor revolution; reduction 15:1.
The encoder is assumed to measure joint output angle.

Run on Pi from ~/six_dof/teensy_motor_test:

    ~/.local/share/mamba/envs/six_dof/bin/python j1_velocity_test.py

Press Enter to set the current position to zero, enter target joint degrees,
then positive joint velocity in degrees/second, then Enter to move. Targets
are relative to that zero, not relative to the preceding move. `z` resets zero.
The zero is session-only. Default DIR 0 increases encoder angle; if incorrect,
restart with --positive-dir 1. Verify with a small move at 2 degrees/second.

Firmware ANGLE_J1 reads port 1 while the pulse timer runs. The host polls at
approximately 100 Hz (slower when serial/I2C adds latency), and stops within
0.3 degrees before the target or on crossing it. This is constant commanded
speed with encoder target stopping, not regulated measured velocity or a
position-holding servo. No ramp, auto-reversal, or final corrective move.
Stopping accuracy depends on speed, latency, inertia, and backlash; high
speeds can overshoot. No calibrated joint travel limits are configured.

Read failures and wrong-way movement over 1 degree stop the test; magnet
strength flags do not gate motion. At the user's request the pulse budget
has been removed. Firmware RUN drives continuously until stopped; a stalled
motor or frozen-but-readable encoder will NOT automatically stop the move.
There is no fixed move-duration deadline. Existing 1.5-second USB-command
watchdog remains enabled. Ctrl+C stops pulses, not driver holding current.
Use only one motor client at a time. Keep the arm supported and its path clear.

--check reads firmware status and the encoder without issuing motor commands.
test_j1_velocity.py contains offline calculation/control tests.

Verified 2026-09-26: firmware built/uploaded; five offline tests passed;
100 live encoder reads passed (raw 643–644). Idle status confirmed zero
generated pulses. Physical motion and direction are not yet verified.
