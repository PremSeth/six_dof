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
motion speed and acceleration caps (default1). Use a slower scale for the first
combined trial. Scale does not make an unknown first servo position rampable.

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
| wa | 4/5 | 5 | nominal 10:1 | 1.2 | 10 | 2 |
| wb | 6/7 | 4 | nominal 10:1 | 1.2 | 10 | 2 |
| j6 | servo pin15 | none | — | internal servo | 30 commanded | 60 commanded |
| claw | servo pin14 | none | — | internal servo | 30 commanded | 60 commanded |

All steppers3200ppr. Positive encoder DIR levels1,0,1,1,1. Rates floored to
355,933,2000,888,888Hz. First-three settings are the user's selected tuning.
Wrist settings remain conservative and gearbox scaling still needs validation.
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
per-axis P correction. Reference duration is chosen to respect all seven
coordinates' nominal speed/acceleration limits. It is a smooth planned reference,
not the removed distance-based braking heuristic. All reference coordinates
finish together; physical completion can lag. Stepper endpoints require real
accepted feedback within±0.5°, zero commanded rates, stable0.25s. Servo completion
means the target pulse was sent, NOT that its shaft was verified there.
Each waypoint is a stop/settle point; there is no corner blending or automatic
looping. To close a claw only after reaching a location, save two poses at the
same arm position with different claw commands, then queue both. Otherwise claw
and arm move together during the segment.

## Saved data and failure handling

`last_taught_poses.json` is a local audit of poses and queue. It is Git-ignored,
never automatically loaded, and overwritten by the next session's saves.
This version does not support restoring poses after restart: re-zero and save
new poses. This avoids silently replaying coordinates with a changed reference.

Encoder selection retries at most3 times within a100ms per-channel budget;
in-flight Wire calls can overrun that budget until the library timeout returns.
Host rejects angle innovation>.75° versus pulse prediction during motion;
brief rejected readings use prediction for<100ms, then stop. This is not proof
of actual motion. Zero/save/start/finish require real feedback. No Kalman or D.
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
