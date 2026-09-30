# Teensy 4.0 diagnostic firmware

This firmware tests USB serial, CPU execution, a dedicated 64 KiB RAM buffer,
and a 1 kHz IntervalTimer. It does not configure external GPIO pins or drive
motors. Uploading replaces the program previously on the Teensy.

Hardware: Teensy 4.0 connected by USB to Raspberry Pi 5. The Teensy's VIN–VUSB
trace is cut, so it requires external power at VIN and a shared ground. USB
alone will not power this board.

On the Pi, PlatformIO and pyserial are installed in Micromamba's `six_dof` env.

```bash
cd ~/six_dof/teensy_diagnostics
~/.local/bin/micromamba run -n six_dof pio run
~/.local/bin/micromamba run -n six_dof pio run --target upload
~/.local/bin/micromamba run -n six_dof python test_usb.py
```

For the first upload, briefly press the Teensy Program button. Linux USB
permissions must allow the Pi's `mecharm` account to access the device:

```bash
sudo install -m 644 00-six-dof-teensy.rules /etc/udev/rules.d/00-six-dof-teensy.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
```

`test_usb.py` saves `test_results.json` only after every assertion passes.
It checks handshake, four RAM patterns, timer counts over two seconds, 2,000
echo messages, invalid/overlong input, fragmented/batched input, and ten serial
port reopenings. Latency measurements are Python/USB round trips, not guaranteed
real-time deadlines. RAM coverage excludes other memory regions. Port reopening
does not test physical reconnection. Encoders, external pins, power integrity,
motor drivers, and six-axis motion remain separate tests.
