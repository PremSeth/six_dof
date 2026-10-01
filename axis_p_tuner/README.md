# Single-axis P tuner: base, J1, J2 only

One selected motor receives pulses; the other two STEP outputs remain LOW.
Firmware never configures wrist/servo pins. Holding current is controlled by
the drivers, not this program. Support gravity-loaded links before altering power.

| Axis | STEP | DIR | Encoder mux | Ratio | Positive DIR | Max speed |
|---|---:|---:|---:|---:|---:|---:|
| base | 2 | 3 | 0 | 2:1 | 1 | 5°/s |
| j1 | 23 | 22 | 1 | 15:1 | 0 | 7°/s |
| j2 | 0 | 1 | 3 | 15:1 | 1 | 15°/s |

All drivers: 3200 pulses/motor revolution. Teensy pin numbering. J1 uses
the existing common-anode NPN circuit; base/J2 use existing direct inputs.
Only the selected encoder needs to be connected. No wrist or sixth-axis tuning.

## Install and run on the Pi

Exit all other motor scripts. Upload replaces the previously installed firmware:

```bash
cd ~/six_dof/axis_p_tuner
~/.local/bin/micromamba run -n six_dof pio run --target upload
~/.local/share/mamba/envs/six_dof/bin/python tune.py j2 --check
~/.local/share/mamba/envs/six_dof/bin/python tune.py j2
```

Replace `j2` with `j1` or `base`; the same firmware supports all three.
Quit before switching axes. `--check` selects the axis and reads its encoder,
but sends no velocity/pulse commands. A running axis cannot be reselected.

At the prompt:

- `z`: make the current position zero (required before moving).
- `k 1.2`: set Kp; default 1.0, units 1/s.
- `v 2`: set the speed ceiling in output degrees/s; default 2.
- `5`: target +5° relative to your zero. Enter at the confirmation to move.
- `0`: return to zero, with confirmation.
- `w`: show position; `q`: quit.
- Enter during motion: stop and return to prompt. Ctrl+C: stop and exit.

Gains/speed are session-only; use `--kp 1.2 --speed 2` to supply launch values.
This is position P, **not** trajectory feedforward or PD:
`velocity = Kp * (target - measured angle)`, then speed/acceleration limits.
Distance-aware braking has been removed at the user's request. P still reduces
speed as position error decreases; no extra stopping-distance ceiling applies.
Long moves with high speed and low acceleration may overshoot and trip the
unchanged travel-envelope stop. This is not protection against gravity,
slipping, stalls, backlash or bad encoder calibration.
Default acceleration 2°/s²; `--accel` allows up to 50. This also sets the
deceleration limit; increase gradually and check for stalls or overshoot.
For example: `python tune.py j2 --kp 1 --speed 5 --accel 50`.
J2 allows `--speed 15` (2000 pulses/s); J1 allows `--speed 7` (933 pulses/s,
rounded down to 6.9975°/s); base remains capped at 5°/s.
The updated client requires matching firmware with `HZ=88,933,2000`.
Motor pulses use a 20kHz
timer. Host controller nominally 50Hz. Display reports angle, target, error,
commanded velocity and Kp. Within ±0.35°, requested velocity becomes zero;
after settling 0.25s, pulses stop (this is not continuous active position hold).

Start with a small clear-path move. Change one gain at a time; test positive and
negative moves. Larger Kp corrects faster but may overshoot or oscillate. Reduce
it if oscillatory. Saturated speed/acceleration can hide changes in Kp. Gains are
not physically tuned automatically. Base gear ratio and encoder resolution mean
small-angle behavior may be quantized. There is no gravity/torque compensation.

## Failure handling

Selected encoder reads use at most three attempts with a 100ms elapsed budget,
1ms between failures, and mux select/readback/deselect. In-flight Wire operations
can overrun that budget until the library timeout returns. No magnet-status
gating. Persistent I2C failure stops and prints stage/code.
At 5°/s, 100ms corresponds to 0.5° continued motion, not a hard travel bound.

ISR watchdog stops after 300ms without a velocity command or fresh encoder.
Host stops on a >250ms control-loop gap, lost feedback, or measured position
outside the start/target interval by >2°. Faults require STOP; no auto-resume.
These are not collision avoidance or calibrated joint limits. A stalled motor
with readable but frozen feedback can keep receiving pulses: **no automatic
stall or move-duration timeout**. Stay present, keep the path clear, and have
independent motor-power isolation accessible. No pulse command changes motor
holding current. Do not use this tuner as an unattended controller.

Offline tests: `python -m unittest test_tune`. Simulated convergence is not a
physical tuning result. Loading another project requires its firmware again.

## Encoder angle outlier filter

During a move, each decoded angle is compared with the last accepted angle plus
signed emitted pulses converted using the selected gear ratio. A discrepancy
over 0.75° rejects that sample without updating the accepted encoder baseline.
For less than 100ms since the last accepted reading, control uses pulse-predicted
position without a pause/restart. After that, untrusted feedback stops motion.
Predicted motion is not proof of actual motor motion; persistent mechanical
slip or incorrect scaling may also trigger the stop. This is an outlier gate,
not a Kalman filter or a fix for wiring. I2C transaction failures still stop in
firmware after its retries; the host filter handles successful-but-bogus angles.

Zeroing, starting a move, and completing a move require accepted encoder data.
Gradual real overshoot still triggers the original travel-envelope check.
At idle the acceptance margin also allows hand-positioning at up to 180°/s.
The console prints the first rejected raw/candidate/predicted angle per incident;
envelope errors now print the actual offending angle, start and target.
Gains and limits are unchanged. Restart the Python client after installing this
update; it does not require a firmware reflash. No physical moving test of this
filter has been performed by the assistant.

Verified 2026-09-29: nine offline tests passed, firmware built/uploaded, and
100/100 stationary reads passed for each of base, J1 and J2 with rate=0.
No physical motor movement was commanded during installation. Gains still
require attended physical tuning.
