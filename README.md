# six_dof

For **typed joint targets, saved servo/encoder poses, and queued demo sequences**,
use [waypoint_demo/README.md](waypoint_demo/README.md). This is the latest demo
workflow and has dedicated firmware. Individual tuning tools remain available.

For **individual base/J1/J2 P tuning**, use
[axis_p_tuner/README.md](axis_p_tuner/README.md). Select one axis, set zero,
adjust Kp and speed, and enter targets. Wrist and sixth-axis tuning are excluded.
This has its own firmware; upload the appropriate project before switching tools.

## Current project: Pi 5 + Teensy 4.0

Experimental five-axis encoder-feedback arm control. **Not safety-rated.
No collision avoidance or calibrated
physical travel limits.** Never leave powered motion unattended.

Start with [five_axis_demo/README.md](five_axis_demo/README.md): coordinated,
smoothly ramped teach/replay. Numbered poses: TEACH `0`home,`1`,`2`save;
`r`arms REPLAY, then numbers move;`t`teaches;Enter during motion stops.

| Axis | Teensy STEP/DIR | AS5600 mux | Reduction | Commanded speed cap |
|---|---|---|---|---|
| Base |2/3|0|2:1|5deg/s|
| J1 |23/22 (NPN)|1|15:1|5deg/s|
| J2 |0/1|3|15:1|10deg/s|
| Wrist A |4/5|5|10:1 assumed|10gearboxdeg/s|
| Wrist B |6/7|4|10:1 assumed|10gearboxdeg/s|

All drivers configured3200pulses/motorrev. Wrist measured scaling still needs
calibration. These are controller ceilings, not measured mechanical guarantees.
Only one firmware project can be loaded on the Teensy at a time; scripts verify
their firmware handshake. Flashing an individual test replaces the unified demo.

On the Pi, with a Micromamba environment `six_dof` containing Python,pyserial,
and PlatformIO (Teensy platform pinned in each platformio.ini):

```bash
cd ~/six_dof/five_axis_demo
micromamba run -n six_dof pio run --target upload
micromamba run -n six_dof python demo.py --check
micromamba run -n six_dof python demo.py
```

Use a data-capable USB cable, correct driver interface levels and common ground.
Never feed driver signal-supply voltage into Teensy GPIO. Support gravity-loaded
links before disabling drives for teaching; keep encoders/logic powered.

Directories: `five_axis_demo` unified controller;`teensy_*` individual tests and
diagnostics. All CAD/models/previews, personal memory, pose logs, credentials,
environments and build caches are excluded and remain local.
Root-level pigpio programs and notes below are historical Pi3-era work, NOT the
current Pi5/Teensy control configuration. Do not use them as current pin mappings.

## Historical Raspberry Pi 3 setup

Minimal Raspberry Pi setup for a STEP/DIR stepper motor and future AS5600 use.

Target computer: Raspberry Pi 3 Model B+ with 1 GB RAM. After an OS reflash,
restore SSH/I2C, Micromamba and the `six_dof` environment, the `pigpio` daemon,
and the encoder packages. Also inspect `free -h` and `swapon --show`; configure
persistent microSD-backed swap if the OS has no adequate swap so memory
pressure is less likely to reset or kill the control process. Avoid enabling
two different swap managers at the same time.

The stepper tests use BCM GPIO 14 for PUL/STEP and BCM GPIO 15 for DIR. They do
not read the encoder yet.

Run it with:

```bash
micromamba activate six_dof
cd ~/six_dof
python stepper_test.py
```

Enter a positive pulse count and a target RPM, then press Enter at the
confirmation prompt. The motor moves that many pulses forward, pauses for one
second, and moves the same number backward. Type `q` to cancel or exit.

RPM is converted to pulse frequency with:

```text
pulses/second = RPM * PULSES_PER_REV / 60
```

The script assumes a 1.8-degree motor (200 full steps/revolution) and a driver
set to 1/16 microstepping, so `PULSES_PER_REV` is 3200. Change `MICROSTEPS` near
the top of `stepper_test.py` if the driver's DIP switches use another setting.
For example, at 1/16 microstepping, 300 RPM produces 16000 PUL pulses/second.

Start around 60-120 RPM and increase gradually. A stepper's available torque
drops as RPM rises, and this simple test does not yet use an acceleration ramp
or encoder feedback, so a high commanded RPM can stall or lose steps.

For a gentler test with acceleration and deceleration, run:

```bash
python stepper_ramp_test.py
```

Enter the desired number of seconds per direction and the peak motor RPM. For
example, enter 30 seconds to ramp up, cruise, and ramp down over roughly 30
seconds forward; the script then pauses for one second and runs roughly 30
seconds in reverse. Press Ctrl+C to stop at any time.

`stepper_ramp_test.py` is configured for the Joint 1 15:1 wave drive. With
3,200 motor pulses/revolution, one output revolution is 48,000 pulses, 90 output
degrees is 12,000 pulses, and output RPM is one fifteenth of motor RPM. The ratio is
used for reporting only and does not change the STEP signal.

The ramp test starts at 10 motor RPM and uses six speed stages over the first
and last 25 percent of the requested time. Long, fast moves are automatically
split into waveform batches that respect pigpio's 20-chain-counter limit.
Start with a modest peak speed. The test remains open-loop and cannot detect a
stall or mechanical bind.

The environment includes `pigpio`, `smbus2`, and
`adafruit-circuitpython-as5600`. The `pigpiod` service must be running.

## Hobby-servo test

`servo_test.py` interactively tests the goBILDA 2000 Series Dual Mode Torque
Servo on BCM GPIO 11. In its default position mode, it accepts an angle from 0
through 300, `c` for center, `p <microseconds>` for a direct 500-2500 us
command, `off`, or `q`. Run it with:

```bash
python servo_test.py
```

Power the servo from a suitable 4.8-7.4 V external supply rather than the
Raspberry Pi, and connect the servo-supply ground to Raspberry Pi ground. The
supply must tolerate the servo's high transient current (up to 3 A at stall).

## Fixed-pin general stepper test

`stepper_motor_test.py` uses BCM GPIO 16 for PUL/STEP and BCM GPIO 20 for DIR.
It asks for a STEP-pulse count and motor-shaft RPM, moves forward by the exact
pulse count, pauses for one second, then returns by the same count:

```bash
python stepper_motor_test.py
```

The script assumes a 200-full-step motor with the driver in 1/2-microstep mode,
giving exactly 400 STEP pulses per motor revolution. This test is open-loop and
has no acceleration ramp.

## Joint 2 open-loop motor test

Joint 2 uses BCM GPIO 17 for PUL/STEP and BCM GPIO 27 for DIR. Run its
standalone constant-speed test with:

```bash
python joint2_motor_test.py
```

Enter seconds per direction and motor RPM. The script runs forward, stops for
one second, then reverses for the same duration. It has no encoder feedback or
acceleration ramp; start with a short duration and low motor RPM.

### Joint 2 position control

`joint2_position_control.py` uses STEP BCM 17, DIR BCM 27, and the Joint 2
AS5600 on multiplexer port 2. Move Joint 2 manually to the desired zero, press
Enter to capture it as session `0°`, then enter target joint angles:

```bash
python joint2_position_control.py
```

For every move, the controller asks for a target angle and a **joint-output
RPM**. Enter one target such as `25`, or a comma-separated sequence such as
`20,-15,40,0`. One RPM applies to the complete sequence, and the controller
stops at each encoder target before continuing. It permanently uses Joint 2's
15:1 reduction and shows the corresponding motor RPM before moving. The default
angle limits are -170 through +170 degrees; use `--min-angle` and `--max-angle`
to override them. Joint 2 uses the normal increasing-angle direction by
default; use `--invert-direction` only if the installed mechanism requires it.
The encoder magnet flags are displayed but do not gate motion; an I2C failure
or Ctrl+C stops STEP output.

## Base motor test

The base driver uses BCM GPIO 24 for PUL/STEP and BCM GPIO 23 for DIR. Run its
standalone constant-speed test with:

```bash
python base_motor_test.py
```

Enter the number of seconds per direction and the motor RPM. After confirmation,
the script runs forward at constant speed, stops for one second, and runs in
reverse for the same duration. This test has no encoder feedback or acceleration
ramp; begin with a short duration and low RPM, and press Ctrl+C to stop.

The base motor's AS5600 is connected through I2C multiplexer port 0. The current
base test is open-loop and does not read it yet.

### Closed-loop base position control

`base_position_control.py` combines the base driver (STEP BCM 24, DIR BCM 23)
with the AS5600 on mux port 0. Move the base manually to the desired zero and
press Enter to capture session `0°`; then enter target base angles. The motor
runs at constant velocity until the encoder reaches or crosses the target.
Enter a single target such as `30`, or a comma-separated sequence such as
`30,-20,0`; the configured velocity applies to the complete sequence.

```bash
python base_position_control.py
```

The defaults are 1 base RPM (6 output degrees/second), the permanent 2:1 base
motor-to-output ratio, the 3,200-pulse-per-motor-revolution driver setting, and
software limits of -170 through +170 degrees. Choose another velocity when
needed:

```bash
python base_position_control.py --velocity 60
```

The controller accepts any successfully returned encoder angle regardless of
AS5600 magnet-status flags and has no elapsed-time deadline. An I2C failure or
Ctrl+C stops STEP output. If the first small target moves away from the target,
rerun with `--invert-direction`.

## AS5600 multiplexer test

The reusable encoder diagnostic detects the I2C multiplexer on bus 1, selects
one port, reads the AS5600 at address `0x36`, and restores the mux's original
control value afterward. Port 0 is the default:

```bash
python as5600_mux_test.py --port 0
```

Use `--samples` and `--interval` to change the sampling behavior:

```bash
python as5600_mux_test.py --port 0 --samples 50 --interval 0.05
```

For a valid magnet position, the status should show `MD=1`, `ML=0`, and
`MH=0`. `ML=1` means the magnetic field is too weak; `MH=1` means it is too
strong.

## Joint 1 position control

`joint1_position_control.py` controls Joint 1 using BCM GPIO 14 for STEP, BCM
GPIO 15 for DIR, and the AS5600 on multiplexer port 1. At startup it displays
the live absolute encoder reading. Manually position the joint, then press Enter
to capture that position as session `0°`. After capture, enter target joint
angles; entering `0` returns to the captured zero. Enter a single target such as
`30`, or a comma-separated sequence such as `30,-20,0`; the configured velocity
applies to the complete sequence.

Run at the default constant joint velocity of 15 degrees/second with:

```bash
python joint1_position_control.py
```

Choose another constant velocity with, for example:

```bash
python joint1_position_control.py --velocity 2
```

The current allowed range is greater than 0 through 60 joint degrees/second.
To run at the maximum:

```bash
python joint1_position_control.py --velocity 60
```

The default software limits are -170 through +170 degrees. Override them only
after verifying the mechanism's safe range. If the first commanded movement
runs away from the target, the controller stops automatically; rerun with
`--invert-direction` after checking the wiring and mechanism.

The controller has no target-move deadline; it continues until the target is
reached/crossed, movement is detected in the wrong direction, an I2C read
fails, or the operator presses Ctrl+C. It displays the AS5600 status byte but
does not block motion based on the magnet-status flags; any successfully
returned angle is accepted. It remains a test controller: it has no
limit switches, emergency-stop input, collision detection, or motor-enable
output. Keep a way to remove motor power immediately during initial tests.

### Joint 1 P-controller tuning

`joint1_p_tuner.py` uses the permanent Joint 1 configuration (STEP BCM 14, DIR
BCM 15, encoder mux port 1, 15:1 ratio). Press Enter to capture session zero,
then enter a P gain and target repeatedly. The velocity command is:

```text
joint velocity (deg/s) = Kp (1/s) * position error (deg)
```

Velocity is capped at 120 degrees/second and quantized in 1-degree/second
steps. The controller stops inside a 0.5-degree tolerance for 0.3 seconds, then
reports elapsed time, final error, maximum overshoot, and peak velocity command.

```bash
python joint1_p_tuner.py --kp 1
```

Start with a small target and low gain, then increase Kp gradually. For example,
test `Kp=0.5`, return to `0°`, then test `Kp=1.0`. There is no elapsed-time
deadline or AS5600 magnet-status gate. An I2C failure or Ctrl+C stops STEP
output. Use `--invert-direction` if the first small target moves the wrong way.
