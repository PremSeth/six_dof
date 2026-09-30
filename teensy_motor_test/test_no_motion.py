"""Client calculation tests and read-only USB checks; never sends MOVE."""
import glob
import math
import serial
from motor_test import READY, exchange, motion_parameters

assert motion_parameters(400, 30, 400) == (5000, 30)
assert motion_parameters(200, 60, 400) == (2500, 60)
assert motion_parameters(3200, 60, 3200)[0] == 312
for count, rpm, ppr in [(0, 30, 400), (-1, 30, 400), (10000001, 30, 400),
                        (400, 0, 400), (400, math.nan, 400),
                        (400, math.inf, 400), (400, 30, 0),
                        (400, 100000, 400), (400, .00001, 400)]:
    try:
        motion_parameters(count, rpm, ppr)
    except ValueError:
        pass
    else:
        raise AssertionError((count, rpm, ppr))
print('PASS: motor RPM/pulse calculations and invalid-input rejection')
ports = glob.glob('/dev/serial/by-id/*Teensy*')
assert len(ports) == 1, ports
with serial.Serial(ports[0], 115200, timeout=1, write_timeout=1) as link:
    for _ in range(100):
        assert exchange(link, 'PING') == READY
        state = exchange(link, 'STATUS')
        assert state == 'STATUS state=0 pulses=0 target=0', state
print('PASS: 100 handshake/status exchanges, firmware idle, zero generated pulses')
