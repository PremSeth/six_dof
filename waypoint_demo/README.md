# Joint targets → saved poses → demo sequence

Move the arm using typed joint angles, save measured encoder positions plus
commanded servo positions, and replay a sequence. No hand teaching, FK/IK,
CAD changes or collision model is required for this joint-space workflow.
It is experimental hardware control, not a safety-rated robot controller.

## Start on the Pi

```bash
cd ~/six_dof/waypoint_demo
~/.local/share/mamba/envs/six_dof/bin/python demo.py
```

Requires this project's `WAYPOINT_V1` firmware. If another test has been loaded,
exit all serial clients then upload with
`~/.local/bin/micromamba run -n six_dof pio run --target upload`.
`demo.py --check` reads encoders and servo command status; it sends no motion,
attach, or disable commands. All five encoders must respond.

## Workflow

1. Put the arm in a known, supported reference pose and enter `zero`. This zeros
   all five stepper encoder coordinates; it does not move or home anything.
2. Initialize the sixth-axis and claw commands with `j6 ANGLE` and `claw ANGLE`.
   Each asks for Enter confirmation. Their first movement can be large because
   actual servo position is unknown. Choose angles safe for the real mechanism.
3. `save 0` captures the current measured five encoder angles and the two
   acknowledged servo command angles. Servo pulse quantization is about 0.15°.
4. Jog: `base 10`, `j1 20`, `j2 -10`, `wa 5`, `wb -5`, `j6 160`, `claw 120`.
   These are examples, NOT a collision-free sequence. Every move asks for Enter
   confirmation. Stepper angles are absolute relative to your session zero;
   servo angles are absolute nominal 0–300° commands, never relative to zero.
5. `save 1`, jog elsewhere, then `save 2`, etc. Poses cannot silently overwrite
   existing numbers. Changing zero after saving poses is disallowed.
6. `queue 0 1 2 0` selects the sequence. `run` then Enter plays it once. You can
   also type a saved number, e.g. `1`, then Enter to replay just that pose.

`w` shows angles, including nominal differential pitch/roll and servo commands.
`list` shows poses and queue. `help` lists commands. `scale 0.5` reduces subsequent
stepper speed and acceleration caps (default1). It does NOT slow either servo.
Both servos receive their targets directly, with no software ramp or rate limit.

**Enter during motion stops the sequence**, leaving stepper holding current and
servo signals at their last commands. It does not resume or execute remaining
waypoints. A physical servo may still finish responding to its last pulse.
`off` stops step pulses and disables both servo signals. `q`, Ctrl+C, EOF or a
fatal error also disable signals; support the arm first, as servo holding may
be lost and the claw may release its load. No driver-enable or supply switching.

## Settings and mapping

| Coordinate | STEP/DIR or servo | Encoder mux | Ratio | Kp | Max °/s | Accel °/s² |
|---|---|---|---|---|---|---|
| base | 2/3 | 0 | 2:1 | 3 | 20 | 30 |
| j1 | 23/22, NPN | 1 | 15:1 | 2 | 7 | 20 |
| j2 | 0/1 | 3 | 15:1 | 2 | 15 | 50 |
| wa | 4/5 | 5 | nominal 10:1 | 2 | 20 | 50 |
| wb | 6/7 | 4 | nominal 10:1 | 2 | 20 | 50 |
| j6 | servo pin15 | none | — | internal servo | no software limit | no software limit |
| claw | servo pin14 | none | — | internal servo | no software limit | no software limit |

All steppers3200ppr. Positive encoder DIR levels1,0,1,1,1. Rates floored to
355,933,2000,1777,1777Hz. First-three settings are the user's selected tuning.
Wrist Kp/acceleration match J2; speed is doubled from10 to20deg/s as requested.
Wrist gearbox scaling and these faster settings still need physical validation.
Servos use the previously tested 500–2500µs nominal0–300° mapping, NOT calibrated
mechanical limits. Their actual angle, speed and completion cannot be measured.

`pitch` and `roll` are **nominal differential coordinates**, not validated tool
angles. Based on observed motor signs: pitch=(wb-wa)/2, roll=-(wa+wb)/2;
upward pitch commands wa negative/wb positive, rightward roll commands both
negative. These rely on equal reductions/differential scaling. Verify small
motions before using them. Raw `wa`/`wb` targets remain available. Saved poses
always store raw gearbox encoder coordinates; no unverified CAD axis mapping.

## Control and trajectory behavior

Individual joint jogs use direct P position-to-velocity control with the selected
axis's speed/acceleration limits. Other stepper axes receive zero pulses, not
active positional correction. The old distance-braking limiter is NOT used.
This retains the tuned behavior; large fast moves can still overshoot.

Replay uses a shared quintic time profile with feedforward velocity plus the
per-axis P correction. Reference duration respects only the five stepper
coordinates' nominal speed/acceleration limits. It is a smooth planned reference,
not the removed distance-based braking heuristic. All reference coordinates
for the five steppers finish together; physical completion can lag. Both servo
targets are sent directly at the START of each segment, without interpolation,
and are not synchronized to finish with the arm. Stepper endpoints require real
accepted feedback within±0.5°, zero commanded rates, stable0.25s. Servo completion
means the target pulse was sent, NOT that its shaft was verified there.
Each waypoint is a stop/settle point; there is no corner blending or automatic
looping. To close a claw only after reaching a location, save two poses at the
same arm position with different claw commands, then queue both. Otherwise claw
receives its final target at segment start while the arm follows its trajectory.

## Saved data and failure handling

`last_taught_poses.json` is a local audit of poses and queue. It is Git-ignored,
never automatically loaded, and overwritten by the next session's saves.
This version does not support restoring poses after restart: re-zero and save
new poses. This avoids silently replaying coordinates with a changed reference.

Encoder selection retries at most3 times within a100ms per-channel budget;
in-flight Wire calls can overrun that budget until the library timeout returns.
Host does NOT reject feedback for disagreement with pulse counts. Backlash,
lag, sticking and reversals remain measured motion. A single encoder-to-encoder
jump over12° triggers two immediate additional frame reads. The median actual
angle is accepted if at least two valid readings agree within4°; a consistent
new angle is accepted regardless of commanded pulses. This suppresses isolated
large glitches, not persistent plausible bad data. Ordinary samples require
no additional polling. Encoder wrap is handled before this comparison.
Failed I2C reads or inconsistent confirmation samples briefly hold the last
measured angle (NO pulse prediction), and stop after100ms without an accepted
sample. Zero/save/start/finish require accepted feedback. No Kalman or D.
This change does not remove the separate tracking/travel checks below; actual
multi-degree backlash may still trip those checks. It does not establish a
physically safe stopping strategy or prove every accepted reading is correct.
Firmware stops steps on missing SET commands>300ms, stale encoder>500ms or
invalid commands. Servo heartbeat timeout1.5s disables signals. Firmware STOP
holds servo commands; OFF releases them. Boot leaves all step pulses and servo
signals off. The ISR does not enable movement without fresh encoder reads.

Host also stops on >250ms loop gaps, >2° outside segment start/target envelope,
>4° trajectory tracking error, or not settling within15s after replay reference
ends. Direct P jogs have no overall duration/stall timeout. A frozen but plausible
encoder can still cause continued pulses. Keep independent power isolation
accessible and remain present. There are no calibrated joint limits, collision
checks, gravity compensation or verified physical stopping distances. Clear
the ENTIRE path, not only endpoints. Never force a gearbox while hand positioning.

Travel-envelope violations stop all step pulses and abort the current sequence
but RETURN TO THE PROMPT, preserving session zero and saved poses. Servo signals
hold their last commands. Nothing resumes automatically; enter a new command
only after checking the arm. Other fatal feedback/communication/tracking faults
still exit and disable servo signals. A failed STOP acknowledgment remains fatal.

J1-only backlash exception: around80–100° (upright is90° relative to the user's
original start zero), the tracking-error allowance is10° plus0.5° tolerance.
It applies when the interval between reference and measurement intersects that
zone, in either direction. Travel-envelope allowance also becomes10.5° only
for J1 segments crossing that zone and measured positions within69.5–110.5°.
All other axes/regions retain the checks above. Endpoint tolerance remains±0.5°.
This accepts measured backlash, not commanded jumps. It does NOT reduce speed,
prevent a gravity-driven drop, or validate mechanical safety. Use the SAME zero
reference each session; an arbitrary zero would put this exception in the wrong
physical region. No motion was commanded while implementing this exception.

## Offline verification

```bash
python -m unittest test_demo
g++ -std=c++17 -Wall -Wextra -I tests/native tests/firmware_test.cpp -o /tmp/waypoint_firmware_test
/tmp/waypoint_firmware_test
```

Tests cover coordinated simulation/return, single-joint isolation, differential
signs, pulse mapping, servo-only motion, outliers, saved encoder vs target angles,
queue interruption, fresh-feedback requirements, startup outputs, command caps
and watchdogs. Tests and stationary checks are NOT physical trajectory validation.

Verified 2026-10-01: 14 Python tests and native firmware tests passed. Firmware
built/uploaded; all five encoders returned100/100 valid stationary readings,
maximum frame time7.08ms. Step counts stayedzero and both servo signals stayedoff.
No real motion was commanded. First physical combined replay remains unverified.
