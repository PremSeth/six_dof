"""Encoder-target differential wrist test. Angles are pre-differential output degrees."""
import argparse
import glob
import math
import time
import serial
from map_encoder import exchange

CAPS = 'WRIST_FB_V1 A=0,1,1 B=2,3,2 MAX_HZ=2000'
PULSES_PER_REV = 3200
STEPS_PER_DEGREE = PULSES_PER_REV * 10 / 360
MAX_SPEED = 22.5  # Keep existing physical speed limit when changing microstepping.
BLIND_STEP_LIMIT = 88  # 0.99 gearbox-output degrees at 3200 ppr and 10:1.
TOLERANCE = .18
SYNC_BAND = .3
MODES = {'up': (-1, 1), 'down': (1, -1), 'left': (-1, -1), 'right': (1, 1)}
EST_CAPS = f'WRIST_EST_V1 GRACE_MS=250 BLIND_STEPS={BLIND_STEP_LIMIT}'
MAX_DROPOUT = .25


def poll_interval(speed):
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError('Speed must be positive and finite.')
    # At most 0.2 degrees nominal travel between reads: 20Hz at2,100Hz at20.
    return min(.05, .2 / speed)


def step_delta(current, previous):
    return ((current - previous + 2**31) % 2**32) - 2**31


class AxisEstimate:
    """Step-based observer, not a Kalman filter. Fresh encoder wins."""
    def __init__(self, raw, steps, now):
        self.raw = raw
        self.steps = steps
        self.measured = self.angle = 0.0
        self.fresh_at = now
        self.uncertainty = 0.0  # Heuristic allowance, not a statistical bound.
        self.verify_started = None
        self.verify_good = 0
        self.disagreement = 0.0

    @property
    def verifying(self):
        return self.verify_started is not None

    def update(self, valid, raw, steps, now):
        advance = step_delta(steps, self.steps) / STEPS_PER_DEGREE
        self.angle += advance
        self.steps = steps
        if self.verifying and now - self.verify_started >= MAX_DROPOUT:
            raise RuntimeError(f'Encoder disagreement persisted: {self.disagreement:+.2f} degrees; both motors stopped.')
        if now - self.fresh_at >= MAX_DROPOUT:
            raise RuntimeError('No accepted encoder reading for 250 ms; stopping both motors.')
        if valid:
            measured = self.measured + wrap_delta(raw, self.raw)
            if abs(measured - self.angle) > 2:
                self.disagreement = measured - self.angle
                if not self.verifying:
                    self.verify_started = now
                self.verify_good = 0
                # Reject this sample; keep moving using pulse prediction.
                return self.angle
            self.raw, self.measured, self.angle = raw, measured, measured
            self.fresh_at = now
            self.uncertainty = 0.0
            if self.verifying:
                self.verify_good += 1
                if self.verify_good >= 2:
                    self.verify_started = None
                    self.verify_good = 0
        else:
            self.verify_good = 0
            self.uncertainty += abs(advance) + .02
            if self.uncertainty >= 1:
                raise RuntimeError('Prediction allowance exceeded; stopping both motors.')
        return self.angle


def telemetry(link):
    reply = exchange(link, 'WRIST_EST')
    if not reply.startswith('EST '):
        raise RuntimeError(f'Unexpected telemetry: {reply}')
    fields = dict(item.split('=') for item in reply.split()[1:])
    data = []
    for axis in ('A','B'):
        valid, raw, steps = (int(fields[key+axis]) for key in ('valid','raw','steps'))
        if valid not in (0,1) or not 0 <= raw < 4096 or not 0 <= steps < 2**32:
            raise RuntimeError('Invalid estimator telemetry')
        data.append((bool(valid),raw,steps))
    state = int(fields['state'])
    if state not in range(6):
        raise RuntimeError('Invalid firmware state')
    return data, state


def initial_sample(link):
    # Do not start any motor without two fresh encoder baselines.
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        data, state = telemetry(link)
        if state == 1:
            raise RuntimeError('Motor already moving; cannot set a baseline.')
        if all(item[0] for item in data):
            return data
        time.sleep(.05)
    raise RuntimeError('Could not establish two fresh encoder baselines; no move started.')


def wrap_delta(raw, previous):
    return ((raw - previous + 2048) % 4096 - 2048) * 360 / 4096


def targets(mode, degrees):
    if mode not in MODES or not math.isfinite(degrees) or not 0 < degrees <= 30:
        raise ValueError('Use up/down/left/right and 0 < degrees <= 30 per test move.')
    return tuple(sign * degrees for sign in MODES[mode])


def parse_command(text):
    parts = text.strip().lower().split()
    if len(parts) == 3 and parts[0] == 'axes':
        goal = (float(parts[1]), float(parts[2]))
        if not all(math.isfinite(x) and abs(x) <= 30 for x in goal):
            raise ValueError('Each axis target must be finite and between -30 and +30 degrees.')
        return goal
    if len(parts) == 2 and parts[0] == 'roll':
        amount = float(parts[1])
        if not math.isfinite(amount) or abs(amount) > 30:
            raise ValueError('Roll amount must be finite and between -30 and +30 output degrees.')
        # Positive means left twist from the user's front-facing viewpoint.
        return (-amount, -amount)
    if len(parts) == 2:
        return targets(parts[0],float(parts[1]))
    raise ValueError('Use roll 3, roll -3, axes -3 5, up/down/left/right 3, or q.')


def rates_for(position, goal, speed):
    if not math.isfinite(speed) or not 0 < speed <= MAX_SPEED:
        raise ValueError('Speed must be positive and at most 22.5 gearbox-output degrees/s.')
    if not all(math.isfinite(x) for x in (*position, *goal)):
        raise ValueError('Angles must be finite.')
    errors = [g - p for g, p in zip(goal, position)]
    active = [i for i in range(2) if abs(goal[i]) > TOLERANCE]
    progress = {i: position[i] / goal[i] for i in active}
    slowest = min(progress.values(), default=0)
    rates = []
    for i, error in enumerate(errors):
        if abs(error) <= TOLERANCE:
            rates.append(0)
            continue
        velocity = min(speed, 3 * abs(error))
        # Pause a leading gearbox, not the lagging gearbox taking up backlash.
        # Permit overshoot correction independently toward the final target.
        if i in progress and error * goal[i] > 0:
            lead_degrees = max(0, progress[i] - slowest) * abs(goal[i])
            velocity *= max(0, 1 - lead_degrees / SYNC_BAND)
        hz = min(2000, round(velocity * STEPS_PER_DEGREE))
        rates.append(hz if error > 0 else -hz)
    return tuple(rates)


def read_pair(link):
    reply = exchange(link, 'WRIST_READ')
    if not reply.startswith('WRIST rawA='):
        raise RuntimeError(reply)
    fields = dict(item.split('=') for item in reply.split()[1:])
    result = (int(fields['rawA']), int(fields['rawB']))
    if not all(0 <= raw <= 4095 for raw in result):
        raise RuntimeError('Invalid encoder readings')
    return result


def move(link, goal, speed):
    rates_for((0, 0), goal, speed)
    interval = poll_interval(speed)
    sample = initial_sample(link)
    now = time.monotonic()
    estimates = [AxisEstimate(raw, steps, now) for valid,raw,steps in sample]
    position = [0.0, 0.0]
    last = now
    settled = None
    last_display = 0
    last_rates = None
    try:
        while True:
            cycle_started = time.monotonic()
            if time.monotonic() - last >= MAX_DROPOUT:
                raise RuntimeError('Encoder sampling gap; both motors stopped.')
            data, state = telemetry(link)
            if state in (4,5):
                raise RuntimeError('Firmware stopped: connection lost or encoder prediction grace expired.')
            if state != 1 and last_rates is not None and any(last_rates):
                raise RuntimeError('Motion was interrupted; not restarting automatically.')
            last = time.monotonic()
            position = []
            for i,(est,reading) in enumerate(zip(estimates,data)):
                try:
                    position.append(est.update(*reading,last))
                except RuntimeError as error:
                    raise RuntimeError(f'Encoder port {i+1}: {error}') from error
            for i in range(2):
                if goal[i] * position[i] < 0 and abs(position[i]) > 1:
                    raise RuntimeError(f'Wrong-way movement on encoder port {i+1}.')
                if abs(position[i]) > abs(goal[i]) + 2:
                    raise RuntimeError(f'Excess travel on encoder port {i+1}.')
            verifying = any(est.verifying for est in estimates)
            rates = rates_for(position, goal, speed)
            if rates != last_rates:
                if exchange(link, f'WRIST_VEL {rates[0]} {rates[1]}') != 'OK WRIST_VEL':
                    raise RuntimeError('Rate command not acknowledged')
                last_rates = rates
            fresh = not verifying and all(reading[0] for reading in data)
            reached = fresh and all(abs(g-p) <= TOLERANCE for g,p in zip(goal,position))
            if reached:
                if settled is None:
                    settled = last
                if last - settled >= .15:
                    break
            else:
                settled = None
            if last - last_display > .1:
                print(f'\rPort1 {position[0]:+.2f}/{goal[0]:+.2f}° | '
                      f'Port2 {position[1]:+.2f}/{goal[1]:+.2f}° | Hz {rates} | '
                      f'{"PREDICT/RECHECK" if verifying else "ENCODER" if fresh else "PREDICT"}   ', end='', flush=True)
                last_display = last
            time.sleep(max(0, interval - (time.monotonic() - cycle_started)))
    finally:
        exchange(link, 'STOP')
        print()
    print(f'Encoder targets reached: port1 {position[0]:+.2f}°, port2 {position[1]:+.2f}°.')


def parse_sequence(text):
    commands = text.split(';')
    if not 1 <= len(commands) <= 50 or any(not cmd.strip() for cmd in commands):
        raise ValueError('Provide 1–50 motions separated by semicolons, with no empty entries.')
    # Validate everything before accepting any item or sending a motor command.
    return [(cmd.strip().lower(), parse_command(cmd)) for cmd in commands]


def preview(sequence):
    total = [0.,0.]
    print('Sequence: incremental gearbox-output degrees; nominal cumulative totals shown.')
    for i,(label,goal) in enumerate(sequence,1):
        total = [a+b for a,b in zip(total,goal)]
        print(f'{i}. {label}: A {goal[0]:+.2f}°, B {goal[1]:+.2f}° '
              f'(cumulative A {total[0]:+.2f}°, B {total[1]:+.2f}°)')
    print('Each move starts from the preceding measured endpoint. No calibrated travel limits.')


def run_sequence(link, sequence, speed):
    # A failure/interrupt propagates immediately: never skip ahead or auto-replay.
    try:
        for i,(label,goal) in enumerate(sequence,1):
            print(f'\nMotion {i}/{len(sequence)}: {label}',flush=True)
            move(link,goal,speed)
    finally:
        try:
            exchange(link,'STOP')
        except Exception:
            print('Sequence STOP not acknowledged; firmware watchdog remains active.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Read both encoders without motor commands')
    parser.add_argument('--speed', type=float, default=2, help='Maximum requested gearbox-output deg/s (default 2)')
    args = parser.parse_args()
    rates_for((0,0), (1,1), args.speed)
    ports = glob.glob('/dev/serial/by-id/*Teensy*')
    if len(ports) != 1:
        raise RuntimeError(f'Expected one Teensy; found {ports}')
    with serial.Serial(ports[0],115200,timeout=.3,write_timeout=.3,exclusive=True) as link:
        if exchange(link,'WRIST_CAPS') != CAPS:
            raise RuntimeError('Upload the encoder-feedback wrist firmware first.')
        if exchange(link,'EST_CAPS') != EST_CAPS:
            raise RuntimeError('Upload the pulse-prediction wrist firmware first.')
        state = exchange(link,'STATUS')
        if not state.startswith('STATUS ') or 'state=1 ' in state:
            raise RuntimeError('Stop the other motor test first.')
        if exchange(link,'SERVO_STATUS') != 'SERVOS pin14=0 pin15=0':
            raise RuntimeError('Disable servo signals before wrist testing.')
        raw = [reading[1] for reading in initial_sample(link)]
        print(f'Absolute angles: port1 {raw[0]*360/4096:.2f}°, port2 {raw[1]*360/4096:.2f}°')
        if args.check:
            return
        print('Commands: up 3, down 3, left 3, right 3, roll 3, roll -3, axes -3 5, q.')
        print('Queue: add up 3; roll 3 | queue (review) | clear | run.')
        print('Or enter a whole sequence directly: up 3; roll 3; down 3')
        print('Roll + = left, roll - = right, viewed from the front.')
        print('axes A B: signed incremental encoder angles, A=port1/STEP0, B=port2/STEP2.')
        print('An axis target of 0 holds that encoder near its starting angle during the move.')
        print('Degrees are incremental rotation of EACH gearbox output, not calibrated wrist degrees.')
        print('Encoder-target control with leading-side slowdown and small overshoot correction.')
        print('Brief failed reads use generated-step prediction; 250 ms / ~1° blind-travel limit.')
        print(f'Feedback target {1/poll_interval(args.speed):.0f} Hz for this speed; actual timing depends on I/O.')
        print('Large angle jumps pause BOTH motors; two consistent readings allow resume, persistent disagreement stops.')
        print('Fresh encoders required for completion; prediction is not measured position.')
        print('No stall timeout or pulse budget. If it stalls, Ctrl+C. Keep the wrist clear.')
        pending = []
        try:
            while True:
                text = input('wrist> ').strip().lower()
                if text == 'q':
                    break
                if text == 'queue':
                    preview(pending) if pending else print('Queue is empty.')
                    continue
                if text == 'clear':
                    pending.clear()
                    print('Queue cleared.')
                    continue
                if text.startswith('add '):
                    try:
                        added = parse_sequence(text[4:])
                        if len(pending) + len(added) > 50:
                            raise ValueError('Queue is limited to 50 motions.')
                    except ValueError as error:
                        print(error)
                        continue
                    pending.extend(added)
                    print(f'Added {len(added)} motion(s); {len(pending)} queued. Nothing moved.')
                    continue
                if text == 'run':
                    if not pending:
                        print('Queue is empty. Use add <motion>.')
                        continue
                    sequence = list(pending)
                    preview(sequence)
                    if input('Type run to execute this sequence, or anything else to cancel: ').strip().lower() != 'run':
                        continue
                    pending.clear()  # Never leave an interrupted sequence ready to replay.
                    run_sequence(link,sequence,args.speed)
                    continue
                try:
                    sequence = parse_sequence(text)
                except ValueError as error:
                    print(error)
                    continue
                if len(sequence) > 1:
                    if pending:
                        print('Queue already contains motions: run or clear it before an inline sequence.')
                        continue
                    preview(sequence)
                    if input('Type run to execute this sequence, or anything else to cancel: ').strip().lower() == 'run':
                        run_sequence(link,sequence,args.speed)
                    continue
                if pending:
                    print('Queue contains motions. Use add to append, run to execute, or clear first.')
                    continue
                goal = sequence[0][1]
                print(f'Encoder targets relative to NOW: port1 {goal[0]:+.2f}°, port2 {goal[1]:+.2f}°')
                if input('Enter to move, or type cancel: ').strip():
                    continue
                move(link,goal,args.speed)
        except (KeyboardInterrupt,EOFError):
            print('\nStopping both motors.')
        finally:
            try:
                exchange(link,'STOP')
            except Exception:
                print('STOP not acknowledged; firmware watchdog remains active.')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError,ValueError,serial.SerialException) as error:
        raise SystemExit(str(error))
