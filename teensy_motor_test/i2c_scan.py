"""Read mux/AS5600 detection and angles from Teensy; no motor commands."""
import glob
import time
import serial
import argparse
from motor_test import READY, exchange

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', type=int, choices=[0], help='Scan only mux port 0')
args = parser.parse_args()

ports = glob.glob('/dev/serial/by-id/*Teensy*')
if len(ports) != 1:
    raise SystemExit(f'Expected one Teensy serial device, found {ports}')
with serial.Serial(ports[0], 115200, timeout=2, write_timeout=2) as link:
    if exchange(link, 'PING') != READY:
        raise SystemExit('Unexpected firmware')
    link.write(b'I2C_SCAN0\n' if args.port == 0 else b'I2C_SCAN\n')
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        line = link.readline().decode().strip()
        if line:
            print(line, flush=True)
        if line == 'I2C_DONE':
            break
        if line.startswith('ERROR'):
            raise SystemExit(line)
    else:
        raise SystemExit('Timed out waiting for scan completion')
