"""Hold STEP 23 high for at most 10 seconds for a meter measurement."""
import glob
import time
import serial
from motor_test import READY, exchange

ports = glob.glob('/dev/serial/by-id/*Teensy*')
if len(ports) != 1:
    raise SystemExit(f'Expected one Teensy: {ports}')
with serial.Serial(ports[0], 115200, timeout=1, write_timeout=1) as link:
    if exchange(link, 'PING') != READY:
        raise SystemExit('Wrong firmware')
    print('Black probe: signal ground; red probe: PUL-. This may produce one motor step.', flush=True)
    try:
        for remaining in (3, 2, 1):
            print(f'Starts in {remaining}...', flush=True)
            time.sleep(1)
        assert exchange(link, 'HOLD_STEP') == 'OK HOLD_STEP 10s'
        print('STEP HIGH NOW: PUL- should drop near ground for 10 seconds.', flush=True)
        deadline = time.monotonic() + 12
        while True:
            reply = exchange(link, 'STATUS')
            fields = dict(f.split('=') for f in reply.split()[1:])
            if fields['state'] != '1':
                assert fields['state'] == '2', reply
                print('Timed hold finished; STEP is LOW again. ' + reply, flush=True)
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Hold did not finish on schedule')
            time.sleep(.1)
    finally:
        print(exchange(link, 'STOP'), flush=True)
