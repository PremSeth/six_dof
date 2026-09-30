"""Interactive goBILDA 300-degree servo test: Teensy pins 14 and 15."""
import argparse
import glob
import math
import select
import sys
import serial
from motor_test import READY, exchange


def angle_to_pulse(angle):
    if not math.isfinite(angle) or not 0 <= angle <= 300:
        raise ValueError('Angle must be 0–300 degrees.')
    return round(500 + angle * 2000 / 300)


def status(link):
    reply = exchange(link, 'SERVO_STATUS')
    if not reply.startswith('SERVOS pin14='):
        raise RuntimeError('Servo firmware missing or invalid status.')
    return reply


def prompt(link, text):
    print(text, end='', flush=True)
    while not select.select([sys.stdin], [], [], .2)[0]:
        status(link)
    line = sys.stdin.readline()
    if not line:
        raise EOFError
    status(link)
    return line.strip().lower()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port')
    parser.add_argument('--check', action='store_true', help='Status only; no servo commands')
    args = parser.parse_args()
    ports = [args.port] if args.port else glob.glob('/dev/serial/by-id/*Teensy*')
    if len(ports) != 1:
        parser.error(f'Expected one Teensy; found {ports}')
    with serial.Serial(ports[0], 115200, timeout=.5, write_timeout=.5, exclusive=True) as link:
        if exchange(link, 'PING') != READY:
            raise RuntimeError('Unexpected firmware.')
        motor = exchange(link, 'STATUS')
        print(motor)
        if 'state=1 ' in motor:
            raise RuntimeError('Stop the stepper test before testing servos.')
        print(status(link))
        if args.check:
            return
        print('Commands: 14 <angle>, 15 <angle>, off 14, off 15, off, q')
        print('0–300° = 500–2500 us; 150° is center. No sweep or ramp.')
        print('First move may be abrupt. Keep linkage clear and support the arm.')
        print('Off/quit/Ctrl+C disables signals, NOT supply power; holding may be lost.')
        try:
            while True:
                parts = prompt(link, 'servo> ').split()
                if parts == ['q']:
                    break
                if parts == ['off']:
                    print(exchange(link, 'SERVOS_OFF'))
                    continue
                try:
                    if len(parts) != 2:
                        raise ValueError('Use: 14 150, 15 150, off 14, off 15, off, or q.')
                    if parts[0] == 'off':
                        pin, pulse = int(parts[1]), 0
                    else:
                        pin, pulse = int(parts[0]), angle_to_pulse(float(parts[1]))
                    if pin not in (14, 15):
                        raise ValueError('Use Teensy pin 14 or 15.')
                except ValueError as error:
                    print(error)
                    continue
                if pulse and prompt(link, f'Pin {pin}: {pulse} us. Enter to move, or type cancel: '):
                    continue
                if exchange(link, f'SERVO {pin} {pulse}') != 'OK SERVO':
                    raise RuntimeError('Unexpected servo response.')
                print(status(link))
        except (KeyboardInterrupt, EOFError):
            print('\nStopping servo signals.')
        finally:
            try:
                exchange(link, 'SERVOS_OFF')
            except Exception:
                print('No OFF acknowledgment; firmware disables signals after 1.5s without heartbeat.')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, serial.SerialException) as error:
        raise SystemExit(str(error))
