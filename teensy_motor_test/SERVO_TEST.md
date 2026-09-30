# Teensy servo test

Run on Pi from ~/six_dof/teensy_motor_test:

    ~/.local/share/mamba/envs/six_dof/bin/python servo_test.py

User confirmed goBILDA 300-degree positional servos on Teensy pins 14 and 15,
external servo power supply with common Teensy ground. Mapping inherited from
the prior goBILDA test: 0–300 degrees = 500–2500 us, center 150 = 1500 us.
Positions are servo commands, not calibrated arm joint angles or measured feedback.

Commands: `14 150`, `15 150`, `off 14`, `off 15`, `off`, `q`.
Each position command needs Enter confirmation. No automatic sweep/ramp;
first move may be abrupt, so keep the linkage clear. Signals stay disabled
at boot and until a position is requested. Off/quit/Ctrl+C or a 1.5-second
heartbeat loss disables signals; supply power remains on and holding may be
lost. Support the arm before testing. Servo and stepper tests are mutually
exclusive in firmware; don't run multiple serial clients simultaneously.

Firmware commands: SERVO pin pulse_us (0 disables), SERVO_STATUS,
SERVOS_OFF. Servo library uses a separate IntervalTimer from the stepper.
Attach and pulse assignment are atomic to avoid a default-center pulse.

2026-09-26: firmware built/uploaded, mapping tests and J1 offline regression
tests passed. Live status reported pin14=0 pin15=0 and zero stepper pulses.
No servo movement or electrical pulse measurement performed. J1 encoder
read failed during the final separate check; encoder wiring may have changed.
Backup: src/main.cpp.before_servos_20260926 on Pi.
