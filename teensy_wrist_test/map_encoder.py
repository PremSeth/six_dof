"""Scan, optionally drive a finite slow burst on the selected wrist motor, then rescan."""
import argparse
import glob
import re
import time
import serial


def exchange(link, command):
    link.write((command + '\n').encode())
    reply = link.readline().decode().strip()
    if not reply or reply.startswith('ERROR'):
        raise RuntimeError(reply or 'No serial response')
    return reply


def scan(link):
    link.write(b'I2C_SCAN\n')
    result = {}
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        line = link.readline().decode().strip()
        print(line, flush=True)
        if line == 'I2C_DONE':
            return result
        if 'error' in line.lower() or line.startswith('ERROR'):
            raise RuntimeError(line)
        if line.startswith('MUX ') and not re.search(r'(?:deselect_)?result=0(?:\s|$)', line):
            raise RuntimeError(line)
        match = re.search(r'PORT (\d+) AS5600 ACK raw=(\d+)', line)
        if match:
            result[int(match[1])] = int(match[2])
    raise RuntimeError('Scan timed out')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--move', action='store_true', help='Authorize a finite burst at 200 Hz, DIR0')
    parser.add_argument('--pulses', type=int, default=80, help='Burst size 1–800 (default 80)')
    parser.add_argument('--motor', choices=['23', '01'], default='23',
                        help='STEP/DIR pair; matching firmware must already be loaded')
    args = parser.parse_args()
    if not 1 <= args.pulses <= 800:
        parser.error('--pulses must be 1–800')
    ports = glob.glob('/dev/serial/by-id/*Teensy*')
    if len(ports) != 1:
        raise RuntimeError(f'Expected one Teensy; found {ports}')
    with serial.Serial(ports[0], 115200, timeout=1, write_timeout=1, exclusive=True) as link:
        expected = ('READY SIX_DOF_WRIST_V1 STEP=2 DIR=3 INVERTED=0' if args.motor == '23'
                    else 'READY SIX_DOF_BASE_V1 STEP=0 DIR=1 INVERTED=0')
        if exchange(link, 'PING') != expected:
            raise RuntimeError(f'Wrong firmware; expected {expected}')
        state = exchange(link, 'STATUS')
        if not state.startswith('STATUS ') or 'state=1 ' in state:
            raise RuntimeError('Controller not idle')
        if exchange(link, 'SERVO_STATUS') != 'SERVOS pin14=0 pin15=0':
            raise RuntimeError('Disable servo signals first')
        before = scan(link)
        if not args.move:
            return
        if not before:
            raise RuntimeError('No encoders found; no motion commanded')
        try:
            print(f'Commanding exactly {args.pulses} pulses, 200 Hz, STEP{args.motor[0]}/DIR{args.motor[1]}, DIR0.', flush=True)
            if exchange(link, f'MOVE {args.pulses} 5000 0') != 'OK MOVE':
                raise RuntimeError('Unexpected MOVE response')
            deadline = time.monotonic() + args.pulses / 200 + 3
            while True:
                reply = exchange(link, 'STATUS')
                if not reply.startswith('STATUS '):
                    raise RuntimeError(reply)
                fields = dict(s.split('=') for s in reply.split()[1:])
                if fields['state'] != '1':
                    if fields['state'] != '2' or int(fields['pulses']) != args.pulses:
                        raise RuntimeError(reply)
                    print(reply)
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError('Diagnostic burst did not finish')
                time.sleep(.05)
        finally:
            exchange(link, 'STOP')
        time.sleep(.3)
        after = scan(link)
        for port in sorted(before.keys() | after.keys()):
            if port not in before or port not in after:
                print(f'PORT {port}: missing reading; cannot compare')
                continue
            change = ((after[port] - before[port] + 2048) % 4096 - 2048) * 360 / 4096
            print(f'PORT {port}: {before[port]*360/4096:.2f} -> {after[port]*360/4096:.2f} deg; change {change:+.2f} deg')


if __name__ == '__main__':
    main()
