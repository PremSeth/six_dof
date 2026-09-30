"""Constant commanded BASE velocity, with encoder-based target stopping."""
import argparse
import glob
import math
import select
import sys
import time

import serial
from motor_test import READY, exchange

PPR = 3200
RATIO = 2
TOLERANCE = 0.3


def delta(raw, previous):
    return ((raw - previous + 2048) % 4096 - 2048) * 360 / 4096


def parameters(distance, velocity):
    if not math.isfinite(distance) or not math.isfinite(velocity) or velocity <= 0:
        raise ValueError('Target must be finite; velocity must be positive and finite.')
    period = 1e6 * 360 / (velocity * PPR * RATIO)
    if not 50 <= period <= 1000000:
        raise ValueError('Requested velocity must correspond to 1–20,000 pulses/s.')
    return round(period)


def read_raw(link):
    reply = exchange(link, 'ANGLE_BASE')
    if not reply.startswith('ANGLE_BASE port=0 raw='):
        raise RuntimeError(f'Unexpected encoder reply: {reply}')
    raw = int(reply.rsplit('=', 1)[1])
    if not 0 <= raw < 4096:
        raise RuntimeError('Invalid encoder angle.')
    return raw


class Position:
    def __init__(self, link):
        self.link = link
        self.zero()

    def zero(self):
        self.previous = read_raw(self.link)
        self.angle = 0.0
        self.when = time.monotonic()

    def update(self):
        if time.monotonic() - self.when > 1:
            raise RuntimeError('Encoder tracking gap: restart and set zero again.')
        raw = read_raw(self.link)
        self.angle += delta(raw, self.previous)
        self.previous, self.when = raw, time.monotonic()
        return self.angle

    def prompt(self, text):
        # Keep tracking through idle prompts, including 0/360-degree wraps.
        print(text, end='', flush=True)
        while not select.select([sys.stdin], [], [], .02)[0]:
            self.update()
        value = sys.stdin.readline()
        if not value:
            raise EOFError
        self.update()
        return value.strip()


def move(link, position, target, velocity, positive_dir):
    start = position.update()
    distance = target - start
    period = parameters(distance, velocity)
    if abs(distance) <= TOLERANCE:
        print('Already within target tolerance.')
        return
    sign = 1 if distance > 0 else -1
    direction = positive_dir if sign > 0 else 1 - positive_dir
    try:
        if exchange(link, f'RUN {period} {direction}') != 'OK RUN':
            raise RuntimeError('Unexpected RUN response.')
        while True:
            angle = position.update()
            print(f'\rBASE {angle:+.2f}° → {target:+.2f}°', end='', flush=True)
            if sign * (angle - start) < -1:
                raise RuntimeError('Wrong direction. Restart with the other --positive-dir value.')
            if sign * (target - angle) <= TOLERANCE:
                break
            reply = exchange(link, 'STATUS')
            if not reply.startswith('STATUS '):
                raise RuntimeError(f'Unexpected status: {reply}')
            state = dict(field.split('=') for field in reply.split()[1:])
            if state['state'] != '1':
                raise RuntimeError('Motion interrupted before target.')
            time.sleep(.01)
    finally:
        exchange(link, 'STOP')
        print()
    print(f'Stopped at {position.update():+.2f}°; no automatic correction.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port')
    parser.add_argument('--positive-dir', type=int, choices=[0, 1], default=1,
                        help='DIR level that increases encoder angle (default 1)')
    parser.add_argument('--check', action='store_true', help='Read-only encoder/link check')
    args = parser.parse_args()
    ports = [args.port] if args.port else glob.glob('/dev/serial/by-id/*Teensy*')
    if len(ports) != 1:
        parser.error(f'Expected one Teensy; found {ports}. Check USB/power.')
    with serial.Serial(ports[0], 115200, timeout=.5, write_timeout=.5, exclusive=True) as link:
        if exchange(link, 'PING') != READY:
            raise RuntimeError('Unexpected firmware.')
        if exchange(link, 'CAPS') != 'CAPS RUN ANGLE_BASE_PORT0':
            raise RuntimeError('Upload the firmware with continuous RUN support first.')
        status = exchange(link, 'STATUS')
        print(status)
        if 'state=1 ' in status:
            raise RuntimeError('Motor already running. Stop the other client first.')
        print(f'Port 0 absolute angle: {read_raw(link) * 360 / 4096:.2f}°')
        if args.check:
            return
        print('BASE STEP=2 DIR=3 direct TB6600; 3200 pulses/rev, 2:1; output-mounted encoder.')
        print('Velocity is JOINT degrees/s. Constant pulse rate, no acceleration ramp.')
        print('No pulse budget: a stall/frozen encoder will NOT automatically stop motion.')
        print('Start with a small move at 2°/s to verify direction. Ctrl+C stops pulses.')
        position = Position(link)
        try:
            position.prompt('Press Enter to make the CURRENT position 0°: ')
            position.zero()
            while True:
                text = position.prompt('Target degrees [z: set zero, q: quit]: ').lower()
                if text == 'q':
                    break
                if text == 'z':
                    position.zero()
                    print('Current position set to 0°.')
                    continue
                try:
                    target = float(text)
                    velocity = float(position.prompt('Joint velocity °/s [2]: ') or '2')
                    parameters(target - position.angle, velocity)
                except ValueError as error:
                    print(error)
                    continue
                if position.prompt('Enter to move, or type cancel: '):
                    continue
                move(link, position, target, velocity, args.positive_dir)
        except (KeyboardInterrupt, EOFError):
            print('\nStopping.')
        finally:
            try:
                exchange(link, 'STOP')
            except Exception:
                print('STOP not acknowledged; firmware link watchdog remains active.')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, serial.SerialException) as error:
        raise SystemExit(str(error))
