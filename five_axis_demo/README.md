# Five-axis teach-and-repeat demo

## One-handed numbered controls (current)

Start in TEACH mode. Numbers save positions without moving anything:

- `0` Enter: set current pose as zero/home (first saved pose).
- Hand-position a small distance; `1` Enter saves pose1, then `2` saves pose2, etc.
- Keep logic/encoders powered throughout. Support the arm before disabling
  motor drives; do not force a gearbox. The script does not control driver enable.
- When ready, turn drives on, keep clear, then `r` Enter arms REPLAY.
- In REPLAY, `0`, `1`, `2`, etc. immediately move to the saved pose. No extra RUN.
- `t` returns to TEACH, `l` lists poses, `w` prints angles, `q` exits.
- During motion, Enter (or any submitted line) stops and returns to TEACH.
  Ctrl+C stops and exits. Motion commands are not queued.

Existing numbered poses cannot be overwritten accidentally. Poses are relative
to this running session; restart to redefine home. Pose logs stay local/private.
No per-move angle cap. You must check clearance along the ENTIRE interpolated
path, not just at saved endpoints. No collision avoidance or physical joint limits.

Speed ceilings: base5deg/s,J1 5deg/s,J2 10deg/s,wristA/B each10gearboxdeg/s.
Default uses these per-axis ceilings. Optional `--speed 2` lowers every ceiling.
Acceleration stays2deg/s²(default,max3); shared timing and gentle ramps may
produce speeds below the ceilings. Firmware enforces per-axis pulse limits
[88,666,1333,888,888]Hz, rounded down so nominal output speed never exceeds caps.
These are commanded/calibrated speeds, not guaranteed measured speeds.
V2 firmware/client handshake includes these limits; old client/firmware refuses.

## Installation verification 2026-09-29

Unified firmware built/uploaded successfully. Eight offline tests passed,
including coordinated convergence with half-speed wrist simulation and STOP on
fault/interrupt. Live stationary check:100/100valid frames on every encoder;
maximum five-encoder frame latency7.05ms; all pulse counters remainedzero.
No physical coordinated replay has been performed by the assistant.

No Cartesian IK, MoveIt or collision avoidance. All five output coordinates
follow a common quintic time profile, with encoder proportional correction,
velocity/acceleration limits and atomic five-rate updates to a Teensy20kHz timer.
References start/end together; actual completion may be later due to tracking
error or backlash. Wrist coordinates are the two gearbox encoders, not pitch/roll.

Axis order / STEP / DIR / mux / ratio / encoder-positive DIR:

| Axis | STEP | DIR | mux | ratio | +DIR |
|---|---:|---:|---:|---:|---:|
| base |2|3|0|2|1|
| J1 |23|22|1|15|0|
| J2 |0|1|3|15|1|
| wrist A |4|5|5|10|1|
| wrist B |6|7|4|10|1|

All drivers3200ppr. J1 common-anode NPN, others existing direct interfaces.
Firmware handshake includes pin/port/direction table. Replaces individual test
firmware; sixth joint and servos are not driven.

## First demonstration (keep motor drives off until the replay prompt)

Support gravity-loaded links BEFORE disabling driver power—arm may drop.
Keep Pi/Teensy/encoder logic powered. Only hand-position if mechanism is safely
backdrivable; do not force a stiff gearbox. No enable-line control is implemented.

On Pi:
```
cd ~/six_dof/five_axis_demo
~/.local/share/mamba/envs/six_dof/bin/python demo.py
```
At chosen start pose: `0`. Hand-position the arm to another safely reachable
pose, then `1`. Hand-return near home while the
script continues polling. Enable drives, clear the path and safely remove supports.
Type `r`, then `1` to replay; `0` returns to home along the joint-space segment.
Angles unwrap continuously while script runs. Do not reboot logic midway.

`w` prints current coordinates; `l` lists poses; `q` exits.
`--check` reads all five encoders WITHOUT issuing motor commands.
`--speed` optionally lowers the per-axis ceilings; `--accel` default2deg/s²,max3.
The former15deg per-axis move restriction has been removed. Duration extends to obey
reference speed/acceleration limits. Firmware axis limits stated above. ±.35deg endpoint
tolerance. No calibrated physical travel limits; no guarantee path is safe.
Session-relative poses cannot be auto-loaded; last_taught_poses.json is audit-only.

## Stop behavior / limitations

Ctrl+C/EOF/errors attempt STOP for all five motors; no auto-resume. Step pulses
stop, NOT holding current. Firmware latches linkloss after300ms or stale encoder
after500ms. Host stops after300ms unaccepted readings, tracking error>4deg,
2deg travel-envelope violation or failure to settle within15sec after reference.
An isolated invalid/outlier frame uses pulse prediction without pausing/restarting.
All five encoder channels must work, including stationary axes. If disconnected,
stop and fix before replay. Collision-free endpoints do NOT imply collision-free
interpolation. Teach intermediate waypoints around obstacles.

Wrist tests measured~14–16deg for nominal30deg. Calibration remains uncertain.
Feedback can correct moderate mismatch, but do NOT interpret simulated controller
tests as proof of real hardware behavior. First physical replay must be short,
slow, attended, and independently observed. The demo is not yet hardware validated.
