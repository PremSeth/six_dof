"""Stationary paired-encoder stress check; sends no motor commands."""
import argparse
from collections import Counter
import glob
import time
import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=1000)
    args = parser.parse_args()
    if not 1 <= args.samples <= 10000:
        parser.error('--samples must be 1–10000')
    ports = glob.glob('/dev/serial/by-id/*Teensy*')
    if len(ports) != 1:
        raise SystemExit(f'Expected one Teensy; found {ports}')
    with serial.Serial(ports[0],115200,timeout=.5,write_timeout=.5,exclusive=True) as link:
        def ask(command):
            link.write((command+'\n').encode())
            reply = link.readline().decode().strip()
            if not reply:
                raise RuntimeError('No reply; ending diagnostic')
            return reply
        if ask('WRIST_CAPS') != 'WRIST_FB_V1 A=0,1,1 B=2,3,2 MAX_HZ=2000':
            raise RuntimeError('Unexpected firmware')
        state = ask('STATUS')
        if not state.startswith('STATUS ') or 'state=1 ' in state:
            raise RuntimeError('Stop all motor tests first')
        if ask('SERVO_STATUS') != 'SERVOS pin14=0 pin15=0':
            raise RuntimeError('Disable servo signals first')
        print('Initial:',state,flush=True)
        failures = Counter()
        good = 0
        started = time.monotonic()
        for i in range(args.samples):
            reply = ask('WRIST_READ')
            if reply.startswith('WRIST rawA='):
                fields = dict(item.split('=') for item in reply.split()[1:])
                if not all(0 <= int(fields[key]) < 4096 for key in ('rawA','rawB')):
                    raise RuntimeError('Invalid encoder values')
                good += 1
            elif reply.startswith('ERROR wrist_encoder_read'):
                failures[reply] += 1
            else:
                raise RuntimeError(f'Unexpected reply: {reply}')
            if (i+1) % 100 == 0:
                print(f'{i+1} reads: {good} successful, {sum(failures.values())} failed',flush=True)
            time.sleep(.05)  # Match the slower wrist-controller polling.
        print(f'RESULT: {good}/{args.samples} successful in {time.monotonic()-started:.2f}s')
        for error,count in failures.items():
            print(f'{count}x {error}')
        print('Final:',ask('STATUS'))
        if failures:
            raise SystemExit(1)


if __name__ == '__main__':
    main()
