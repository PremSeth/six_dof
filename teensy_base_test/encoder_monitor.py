"""Read-only encoder direction monitor for the shared mux port-0 test setup."""
import argparse
import glob
import time
import serial
from motor_test import READY, exchange
from j2_velocity_test import read_raw, delta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='USB serial device override')
    parser.add_argument('--samples', type=int, default=0, help='Stop after N readings; 0 runs until Ctrl+C')
    args = parser.parse_args()
    if args.samples < 0:
        parser.error('--samples must be nonnegative')
    ports = [args.port] if args.port else glob.glob('/dev/serial/by-id/*Teensy*')
    if len(ports) != 1:
        parser.error(f'Expected one Teensy; found {ports}')
    with serial.Serial(ports[0], 115200, timeout=.5, write_timeout=.5, exclusive=True) as link:
        if exchange(link, 'PING') != READY:
            raise RuntimeError('Expected the shared base/J2 test firmware.')
        state = exchange(link, 'STATUS')
        if not state.startswith('STATUS ') or 'state=1 ' in state:
            raise RuntimeError('Stop the motor test before running this monitor.')
        previous = read_raw(link)
        relative = 0.0
        count = 0
        last = time.monotonic()
        print('Encoder port 0. Start = 0 degrees relative. Move slowly by hand.')
        print('Positive change = increasing encoder angle; negative = decreasing.')
        print('Handles 0/360 wrap. No motor commands. Ctrl+C exits.')
        print(' raw   absolute    from start    change / sample')
        try:
            while not args.samples or count < args.samples:
                if time.monotonic() - last > 1:
                    raise RuntimeError('Sampling interrupted; restart to establish a fresh baseline.')
                raw = read_raw(link)
                change = delta(raw, previous)
                relative += change
                print(f'{raw:4d}   {raw * 360 / 4096:8.2f}°   {relative:+10.2f}°   {change:+8.2f}°', flush=True)
                previous = raw
                last = time.monotonic()
                count += 1
                time.sleep(.1)
        except KeyboardInterrupt:
            print('\nMonitor stopped.')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, serial.SerialException) as error:
        raise SystemExit(str(error))
