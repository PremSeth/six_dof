"""Open-loop J2 motor test. Teensy 0=PUL, 1=DIR; direct driver inputs."""
import argparse
import glob
import math
import time
import serial

READY = 'READY SIX_DOF_J2_V1 STEP=0 DIR=1 INVERTED=0'

def exchange(link, command):
    link.write((command + '\n').encode('ascii'))
    reply = link.readline().decode('ascii').rstrip('\r\n')
    if not reply:
        raise RuntimeError('Teensy did not respond; its link watchdog stops pulses.')
    if reply.startswith('ERROR'):
        raise RuntimeError(reply)
    return reply

def motion_parameters(pulses, rpm, ppr):
    if not 1 <= pulses <= 10000000:
        raise ValueError('Pulse count must be 1–10,000,000.')
    if not math.isfinite(rpm) or rpm <= 0 or ppr <= 0:
        raise ValueError('RPM and pulses/revolution must be positive and finite.')
    period = round(60000000 / (rpm * ppr))
    if not 50 <= period <= 1000000:
        raise ValueError('Requested pulse rate must be within 1–20,000 Hz.')
    return period, 60000000 / (period * ppr)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='Defaults to the unique Teensy by-id serial path')
    parser.add_argument('--ppr', type=int, default=3200, help='Driver pulses per motor revolution (default 3200)')
    parser.add_argument('--check', action='store_true', help='Verify firmware and status only; no movement')
    args = parser.parse_args()
    if args.ppr <= 0:
        parser.error('--ppr must be positive')
    ports = [args.port] if args.port else glob.glob('/dev/serial/by-id/*Teensy*')
    if len(ports) != 1:
        parser.error(f'Expected one Teensy; found {ports}. Use --port.')
    with serial.Serial(ports[0], 115200, timeout=1, write_timeout=1) as link:
        if exchange(link, 'PING') != READY:
            raise RuntimeError('Wrong firmware: upload teensy_base_test first.')
        print(exchange(link, 'STATUS'))
        if args.check:
            return
        print(f'Teensy PUL=0, DIR=1, direct driver; {args.ppr} driver pulses/motor revolution.')
        print('Open loop, constant RPM with no acceleration ramp. Start slowly.')
        print('Each test is one direction only. Ctrl+C stops pulses; q exits.')
        try:
            while True:
                raw = input('Pulse count [q to exit]: ').strip()
                if raw.lower() == 'q':
                    break
                try:
                    pulses = int(raw)
                    rpm = float(input('Motor shaft RPM [30]: ').strip() or '30')
                    period, actual_rpm = motion_parameters(pulses, rpm, args.ppr)
                    direction = input('Teensy DIR level [0 or 1, default 0]: ').strip() or '0'
                    if direction not in ('0', '1'):
                        raise ValueError('DIR must be 0 or 1.')
                except ValueError as error:
                    print(error)
                    continue
                print(f'{pulses} pulses; theoretical motor rotation {pulses / args.ppr * 360:.2f} degrees; '
                      f'{actual_rpm:.3f} RPM; approximately {pulses * period / 1e6:.2f} seconds.')
                if input('Press Enter to move, or type cancel: ').strip():
                    continue
                if exchange(link, f'MOVE {pulses} {period} {direction}') != 'OK MOVE':
                    raise RuntimeError('Unexpected MOVE response')
                while True:
                    reply = exchange(link, 'STATUS')
                    if not reply.startswith('STATUS '):
                        raise RuntimeError(f'Unexpected response: {reply}')
                    state = dict(field.split('=') for field in reply.split()[1:])
                    print(f"\rGenerated {state['pulses']}/{state['target']} pulses", end='', flush=True)
                    if state['state'] != '1':
                        print()
                        if state['state'] != '2' or int(state['pulses']) != pulses:
                            raise RuntimeError(f'Motion interrupted: {reply}')
                        print('Pulse generation complete; actual rotation is not encoder-verified.')
                        break
                    time.sleep(.1)
        except (KeyboardInterrupt, EOFError):
            print('\nStopping.')
        finally:
            try:
                exchange(link, 'STOP')
            except Exception:
                print('STOP could not be acknowledged; firmware link watchdog remains active.')

if __name__ == '__main__':
    main()
