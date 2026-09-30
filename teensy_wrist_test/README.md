# Wrist encoder mapping test

## Current calibration restored: 3200 pulses/revolution (2026-09-28)

Both wrist motors are back to 3200 pulses per motor revolution, 10:1 ratio.
Set both hardware drivers accordingly. Firmware allowance is again 88 pulses
(0.99 output degrees). Encoder-disagreement pauses remain removed. This
supersedes the 200-ppr calibration below; restart client after firmware upload.

## No transient encoder-disagreement pause (2026-09-28)

Angle outliers no longer force both motor rates to zero. Motion continues
using pulse prediction while samples are rechecked (PREDICT/RECHECK).
Two consistent readings still restore trusted feedback. Sustained feedback
loss/disagreement stops remain, as do target completion and axis synchronization.
This supersedes historical VERIFY/PAUSED behavior below. No firmware change.

## Current calibration: 200 pulses/revolution (2026-09-28)

Both wrist drivers must now be set to 200 pulses per motor revolution (full
steps for a 1.8-degree motor), not 200 subdivisions per step. Ratio remains
10:1: 2000 pulses/output revolution, 0.18 output degrees/pulse. Client pulse
scaling and firmware dropout guard now match this setting (5 pulses = 0.9
degrees). Existing 22.5 deg/s physical speed ceiling is unchanged. Older
3200-ppr/88-pulse notes below are historical. Reflash firmware and restart
the client together. No acceleration ramp added by this calibration change.

## Motion sequences

Inside wrist_position_test.py, enter `up 3; roll 3; down 3; roll -3`, review
the full preview, then type `run` at confirmation. Or build a queue:

    add up 3
    add roll 3; down 3
    queue
    run

The run command previews again and requires typing run a second time to
confirm. `clear` discards the queue; q exits. Adding/reviewing never moves.
All entries are validated before accepting the batch (max50 motions).
Each finishes with encoder-confirmed completion and STOP before the next
starts. All use the selected --speed, including adaptive polling. Targets
are incremental from each measured endpoint; cumulative preview is nominal,
not a calibrated collision/travel-limit check. Errors/Ctrl+C abort remaining
motions; no automatic skip, restart, replay, or loop. Queue is session-only.
19offline tests passed, including order and abort-on-fault/interrupt.

## Roll and independent gearbox targets

Same wrist_position_test.py, same --speed option. New commands:

- `roll 3`: left twist request, both encoder targets -3 degrees.
- `roll -3`: right twist request, both targets +3 degrees.
- `axes -3 5`: encoder port1 target -3deg, port2 target +5deg.
- `axes 0 3`: hold port1 near its start while port2 moves +3deg.

Every command is incremental from the position at confirmation, not an absolute
wrist pose. Amounts are pre-differential gearbox-output degrees; final wrist
roll/tilt conversion still not calibrated. Unequal targets intentionally mix
tilt and twist. Existing +/-30deg per-test limit, confirmation, feedback and
dropout behavior retained. Nothing moves until Enter confirmation.

## Latest client fixes: speed and outliers

Polling target is now min(50ms,0.2/speed seconds), including I/O time when
possible:20Hz at2deg/s,100Hz at20deg/s. Firmware88-step/250ms dropout limits
are unchanged. OS/I2C delays can still trigger them; no hard real-time claim.
>2degree angle discrepancies now pause BOTH motors and reject that sample.
Two consecutive fresh readings consistent with prediction allow resume;
otherwise250ms verification/freshness deadline stops. A missing reading does
not release a verification pause. Persistent errors identify encoder port.

14offline tests passed, including20deg/s simulated pulse progression with
two missed readings and transient angle-jump pause/resume. Hardware --check
--speed20 passed without movement. Real20deg/s motion not tested this turn.

## Latest: bounded pulse-prediction controller

`wrist_position_test.py` now uses WRIST_EST, not legacy WRIST_READ. Actual
signed generated pulse counts are tracked independently in Teensy velocityTick
and returned with each encoder sample and validity flag. Counters wrap at32bits;
client handles wrap. Prediction uses3200ppr*10:1=32000 pulses/output revolution.
This is a step-based observer, NOT a tuned Kalman filter. Fresh encoder readings
replace the predicted position; uncertain prediction is used only for brief
missing samples. ENCODER/PREDICT labels identify the active measurement state.

Grace is250ms since each encoder's last successful read; firmware also limits
travel without a fresh reading to88 generated pulses (~0.99 output degrees).
Both guards run inside the pulse ISR even if I2C foreground code stalls.
Host enforces250ms and a heuristic1degree prediction allowance; >2degree
encoder/prediction disagreement stops instead of silently correcting a large
jump. Serial errors still stop; completion requires BOTH encoders fresh and
within tolerance continuously150ms. No powered motion tested with this version.

Tests:10 offline tests passed (counter wrap/reversal, missed-read correction,
grace/allowance bounds, fresh-only completion, firmware-fault stop, controller
simulation). Built/uploaded. Stationary checks tolerated multiple port1 read
failures, but stopped on apparently successful large jumps. Detailed repeat:
after12 port1 missing reads, sample207 jumped raw1446->1359 (-7.65deg) with
zero generated steps. Hardware movement vs corrupt valid reading must still
be distinguished; DO NOT claim full reliability. Large disagreement stop kept.

Legacy WRIST_READ retains immediate-stop behavior for existing diagnostics;
encoder_read_check.py therefore still counts/reports every failure. New
capability EST_CAPS=WRIST_EST_V1 GRACE_MS=250 BLIND_STEPS=88. Previous source
backed up on Pi as src/main.cpp.before_prediction.

## Current read-reliability issue

LATEST: feedback and stationary-check pacing reduced to50ms (~20Hz plus I/O)
at user's request. Still294/300 successful,6port1 failures while stationary.
Slower polling alone does not resolve the fault; no motion tested in this run.

After user-reported read error during motion, stationary paired reads also
failed. Added100us settling after mux selection and explicit port/stage/code
errors; both motors stop before cleanup. No retry/bypass. Firmware uploaded.

Run stationary check: `python encoder_read_check.py --samples 1000` using
six_dof Python. It sends no movement commands and refuses an active motor.
Latest result:982/1000 good,18failures (17port1,1port2), zero pulses generated.
Errors include mux mask5 vs expected2, NACK register writes, missing read bytes.
Settling delay alone did NOT fix it. Physical/electrical vs software cause
remains unresolved. Five offline controller tests still pass.

## Current encoder-target controller (supersedes pulse tests below)

User confirmed encoders1/2 measure individual gearbox outputs BEFORE the
differential. New `wrist_position_test.py` controls their actual incremental
angles; final wrist pitch/roll angle mapping is not yet calibrated.

    cd ~/six_dof/teensy_wrist_test
    ~/.local/share/mamba/envs/six_dof/bin/python wrist_position_test.py

Type `up 3` then Enter confirmation. Other commands: down3, left3, right3
(with a space between word and amount), q. Degree amount is EACH encoder's
incremental target, not final wrist degrees. Front view: up=(-d,+d),
down=(+d,-d), left=(-d,-d), right=(+d,+d), ordered ports1,2.

Pi polls both encoders approximately100Hz; per-axis proportional speed limits
and normalized-progress comparison slow/pause the leading output. Independent
Teensy pulse rates run on one50us timer tick. Default requested gearbox speed
limit2deg/s (--speed),3200ppr and10:1 used ONLY for pulse-rate scaling, not
target completion. Both targets must be within0.18deg for150ms. Small automatic
overshoot correction is enabled. No hold loop after completion, no calibrated
travel limits, no acceleration ramp. Test moves limited to30 output degrees.

No pulse budget or stall timeout. A stalled/frozen-but-readable encoder can
keep its motor commanded indefinitely: supervise and Ctrl+C if stalled.
Bus errors stop both, as do wrong-way >1deg, excess measured travel >2deg
beyond target magnitude, sampling gaps, and USB heartbeat loss. Encoder
strength flags do not gate motion. Software coordination is not a certified
safety system; keep clear. Encoders cannot measure backlash AFTER themselves
inside the differential, so final wrist pose may still have error.

Firmware retains WRIST_V1 compatibility and adds WRIST_CAPS, WRIST_READ,
WRIST_VEL signedHzA signedHzB (-2000..2000). Positive rate means DIR1,
negative DIR0. Both zero stops. STOP/1.5s watchdog stops both STEP outputs.
Older pulse-count tests remain diagnostics only, not angle controllers.

Verified: five offline tests including unequal-response/backlash simulation;
firmware built/uploaded;100 paired encoder reads in0.275s; six malformed
velocity commands rejected with firmware idle. No powered motion tested with
the new controller. Backup on Pi: src/main.cpp.before_encoder_control.

## Current paired-test firmware

WRIST_V1 now supports PAIR count period_us dirA dirB (count1–800,
period5000–100000us). A=STEP0/DIR1, B=STEP2/DIR3; shared timer drives both.
STOP and heartbeat loss stop both. Existing MOVE/RUN commands still drive
B only. Older BASE firmware is NOT currently loaded.

From this directory: `python tilt_up_test.py` scans only;
`python tilt_up_test.py --move` sends400 pulses per motor at200Hz, A0/B1,
then stops. Requires six_dof Python. Both encoders must respond first.
Front-facing user observation: A DIR0=up+left twist, B DIR0=down+left twist.
Expected pure-axis mixing still requires testing/calibration.

First paired tilt-up test completed400 pulse pairs; encoder1 -1.58deg,
encoder2 +0.44deg, encoder0 unchanged. Physical result awaiting observation.
Seven malformed PAIR command tests passed without pulses. Prior firmware
source on Pi: src/main.cpp.before_pair_test.

CORRECTION after user rewired intended motor to STEP0/DIR1: previous tests
on0/1 ran the wrong motor; discard tentative port0 wrist mapping.
New800-pulse DIR0 test changed port1 -4.92deg, other ports only one count.
Current wrist mapping: STEP0/DIR1 -> port1; STEP2/DIR3 -> port2.
DIR0 decreased each corresponding encoder. Exact scaling is uncalibrated.

LATEST: user confirmed other wrist motor STEP0/DIR1. BASE_V1 firmware is
currently loaded from teensy_base_test. Use --motor 01 with this firmware;
default --motor 23 requires reloading teensy_wrist_test firmware first.
400-pulse DIR0 test on0/1 changed port0 only -0.35deg, port1/2 unchanged;
other motor encoder mapping remains inconclusive pending physical observation.

Follow-up400-pulse test: port2 moved -5.27 degrees, ports0/1 only -0.09 each.
STEP2/DIR3 therefore follows port2 in this test; DIR0 decreases its angle.
Use --move --pulses 400 for that finite diagnostic (allowed1–800, default80).
Measured scaling/differential mixing is not calibrated yet.

Active firmware uses STEP2 DIR3, direct active-high step pulses, no startup
motion. Other test STEP0 is held low. Prior base/J1 projects remain intact;
their firmware must be restored before using their clients.

From ~/six_dof/teensy_wrist_test on Pi:

    ~/.local/share/mamba/envs/six_dof/bin/python map_encoder.py

Scans only. Add --move only with wrist clear to send exactly80 pulses at200Hz,
DIR0, then STOP and compare all encoders. Ctrl+C triggers STOP during motion;
firmware also retains its heartbeat watchdog and finite pulse count.

User reports10:1 on each wrist motor;3200 pulses/rev assumed pending confirmation.
No differential mixing model or encoder mapping established yet. On2026-09-28
the test generated80/80 pulses but raw readings were unchanged on ports0,1,2
(2710,1204,3340 respectively). Physical movement not confirmed.
