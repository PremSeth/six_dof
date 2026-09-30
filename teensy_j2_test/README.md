# Dedicated J2 test

Confirmed2026-09-29: TeensySTEP0/DIR1, direct driver, muxport3,3200ppr,15:1.
DefaultpositiveDIR1 retained from previous J2 test; verify slowly.

On the Pi:
```
cd ~/six_dof/teensy_j2_test
~/.local/share/mamba/envs/six_dof/bin/python j2_velocity_test.py
```
Enter sets current positionzero; targets are relative to that sessionzero.
`z` rezeros, `q` exits, Ctrl+C stops pulses (not holdingcurrent). Start2deg at2deg/s.
`--check` only checks firmware, idle status and encoder. No magnet-status gating.
Firmware retries a failed encoder transaction up to three total attempts,
with 1 ms between attempts and a 100 ms elapsed-time budget. Each attempt
reselects, checks, reads, and deselects mux port 3. A valid read is required;
no predicted or cached angle is substituted. Pulses continue during brief
retries; persistent failure stops pulses and reports `encoder_read_failed`.
An in-flight I2C operation may exceed the budget until Wire's timeout returns.
At 5 degrees/s, 100 ms of continued motion is 0.5 degrees; this is not a hard
travel bound because an in-flight transaction can overrun the budget.
Errors include the attempt count, elapsed time, and last failed I2C stage/code.
No acceleration ramp, calibrated travel limits, pulsebudget or frozen-encoder
timeout. ContinuousRUN stops at encoder target or on read/wrongdirection error;
1.5sec command-watchdog remains. Support arm and keep path clear.

Distinct J2 firmware handshake prevents base/J1 clients driving these pins.
Upload using `~/.local/bin/micromamba run -n six_dof pio run --target upload`.
Firmware upload replaces whichever test is currently loaded. Base project is
preserved separately onSTEP2/DIR3,mux0,2:1. Older J2 script insideteensy_base_test
is historical; use this dedicated directory now.
