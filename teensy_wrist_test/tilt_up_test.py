"""One finite synchronized tilt-up test; scan-only unless --move is passed."""
import argparse
import glob
import time
import serial
from map_encoder import exchange, scan

READY = 'READY SIX_DOF_WRIST_V1 STEP=2 DIR=3 INVERTED=0'
CAPS = 'PAIR A_STEP=0 A_DIR=1 B_STEP=2 B_DIR=3 MAX=800'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--move', action='store_true', help='400 pulses per motor at 200 Hz; A DIR0 / B DIR1')
    args = parser.parse_args()
    ports = glob.glob('/dev/serial/by-id/*Teensy*')
    if len(ports) != 1:
        raise RuntimeError(f'Expected one Teensy; found {ports}')
    with serial.Serial(ports[0],115200,timeout=1,write_timeout=1,exclusive=True) as link:
        if exchange(link,'PING') != READY or exchange(link,'PAIR_CAPS') != CAPS:
            raise RuntimeError('Wrong firmware or no paired-motor support')
        state = exchange(link,'STATUS')
        if not state.startswith('STATUS ') or 'state=1 ' in state:
            raise RuntimeError('Controller not idle')
        if exchange(link,'SERVO_STATUS') != 'SERVOS pin14=0 pin15=0':
            raise RuntimeError('Disable servo signals first')
        before = scan(link)
        if not {1,2}.issubset(before):
            raise RuntimeError('Both wrist encoders must respond before this diagnostic')
        if not args.move:
            return
        try:
            print('TILT UP TEST: 400 pulses EACH, 200 Hz, A DIR0 / B DIR1.',flush=True)
            if exchange(link,'PAIR 400 5000 0 1') != 'OK PAIR':
                raise RuntimeError('PAIR not acknowledged')
            deadline = time.monotonic() + 5
            while True:
                reply = exchange(link,'STATUS')
                if not reply.startswith('STATUS '):
                    raise RuntimeError(reply)
                fields = dict(x.split('=') for x in reply.split()[1:])
                if fields['state'] != '1':
                    if fields['state'] != '2' or fields['pulses'] != '400':
                        raise RuntimeError(reply)
                    print('Completed 400 synchronized pulse pairs.')
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError('Test completion timeout')
                time.sleep(.05)
        finally:
            exchange(link,'STOP')
        time.sleep(.3)
        after = scan(link)
        for port in sorted(before.keys() | after.keys()):
            if port not in before or port not in after:
                print(f'PORT {port}: missing reading')
                continue
            change=((after[port]-before[port]+2048)%4096-2048)*360/4096
            print(f'PORT {port}: change {change:+.2f} deg')


if __name__ == '__main__':
    main()
