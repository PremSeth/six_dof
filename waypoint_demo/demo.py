"""Joint-target jogging, measured waypoints and coordinated seven-coordinate replay."""
import argparse
import json
import math
import select
import sys
import time
import glob
from pathlib import Path
import serial

NAMES=['base','j1','j2','wa','wb']
PORTS=[0,1,3,5,4]
SPD=[3200*r/360 for r in (2,15,15,10,10)]
KP=[3,2,2,1.2,1.2]
SPEED=[20,7,15,10,10]  # Stepper axes only; servo targets are sent directly.
ACCEL=[30,20,50,2,2]
MAX_HZ=[math.floor(v*s) for v,s in zip(SPEED,SPD)]
READY='READY WAYPOINT_V1 STEP=2,23,0,4,6 DIR=3,22,1,5,7 ENC=0,1,3,5,4 POS=1,0,1,1,1 HZ=355,933,2000,888,888 SERVO=15,14'
TOL=.5
GRACE=.1
JUMP_CONFIRM=12.  # Encoder-to-encoder jump, NOT disagreement with pulses.
CONFIRM_SPREAD=4.
J1_ZONE=(80.,100.)
J1_BACKLASH=10.+TOL  # 10 degrees plus encoder/endpoint tolerance.

def tracking_allowance(axis,reference,measured):
    low,high=J1_ZONE
    crosses=min(reference,measured)<=high and max(reference,measured)>=low
    return J1_BACKLASH if axis==1 and crosses else 4.

def travel_allowance(axis,start,target,measured):
    low,high=J1_ZONE
    crosses=min(start,target)<=high and max(start,target)>=low
    near=low-J1_BACKLASH<=measured<=high+J1_BACKLASH
    return J1_BACKLASH if axis==1 and crosses and near else 2.

def delta(a,b):return ((a-b+2048)%4096-2048)*360/4096
def count_delta(a,b):return ((a-b+2**31)%2**32)-2**31
def finite(value):
    value=float(value)
    if not math.isfinite(value):raise ValueError('Angle must be finite')
    return value
def servo_pulse(angle):
    angle=finite(angle)
    if not 0<=angle<=300:raise ValueError('Servo angle must be 0–300°; check mechanical clearance')
    return round(500+angle*2000/300)
def servo_angle(pulse):return (pulse-500)*300/2000
def shape(u):
    u=max(0,min(1,u))
    return 10*u**3-15*u**4+6*u**5,30*u*u*(1-u)**2
def duration(start,target,scale=1):
    if not 0<scale<=1:raise ValueError('Scale must be >0 and <=1')
    if len(start)!=7 or len(target)!=7:raise ValueError('Need five encoders and two servo angles')
    distance=[abs(finite(b)-finite(a)) for a,b in zip(start,target)]
    return max([.5]+[1.875*d/(v*scale) for d,v in zip(distance,SPEED)]+
               [math.sqrt(5.774*d/(a*scale)) for d,a in zip(distance,ACCEL)])
def exchange(link,command):
    link.write((command+'\n').encode())
    reply=link.readline().decode().strip()
    if not reply or reply.startswith('ERROR'):raise RuntimeError(reply or 'No Teensy reply')
    return reply
def stop(link):
    if exchange(link,'STOP')!='OK STOP':raise RuntimeError('STOP not acknowledged')
def off(link):
    if exchange(link,'OFF')!='OK OFF':raise RuntimeError('OFF not acknowledged')
def send(link,rates,servos):
    if len(rates)!=5 or len(servos)!=2:raise ValueError('Invalid command dimensions')
    if any(not isinstance(r,int) or abs(r)>cap for r,cap in zip(rates,MAX_HZ)):
        raise ValueError('Pulse rate outside limits')
    pulses=[0 if a is None else servo_pulse(a) for a in servos]
    if exchange(link,'SET '+' '.join(map(str,[*rates,*pulses])))!='OK SET':
        raise RuntimeError('Motion update not acknowledged')
    return [None if p==0 else servo_angle(p) for p in pulses]
def read(link):
    reply=exchange(link,'READ')
    if not reply.startswith('FRAME '):raise RuntimeError(reply)
    try:
        d=dict(x.split('=',1) for x in reply.split()[1:])
        if int(d['fault']):raise RuntimeError(f'Firmware fault {d["fault"]}; stopped')
        values=[tuple(int(d[k+str(i)]) for k in ('v','r','s')) for i in range(5)]
        for valid,raw,steps in values:
            if valid not in (0,1) or not 0<=raw<4096 or not 0<=steps<2**32:raise ValueError()
        servo=[int(d['u'+str(i)]) for i in range(2)]
        if any(p!=0 and not 500<=p<=2500 for p in servo):raise ValueError()
        moving=int(d['moving'])
        if moving not in (0,1):raise ValueError()
    except (ValueError,KeyError) as e:raise RuntimeError('Malformed firmware frame') from e
    return values,[None if p==0 else servo_angle(p) for p in servo],bool(moving)

class Tracker:
    def __init__(self,link):
        self.link=link;deadline=time.monotonic()+2
        while True:
            values,servos,moving=read(link)
            if moving:raise RuntimeError('Motor already moving; close other client first')
            if all(v for v,r,s in values):break
            if time.monotonic()>deadline:raise RuntimeError('All five encoders required: '+str(dict(zip(NAMES,[bool(v) for v,r,s in values]))))
            time.sleep(.02)
        self.raw=[r for v,r,s in values];self.steps=[s for v,r,s in values]
        self.pos=[0.]*5;self.measured=[0.]*5;self.good=[True]*5
        self.when=time.monotonic();self.fresh=[self.when]*5
        self.servos=servos;self.motion=False
    def update(self):
        values,servos,moving=read(self.link)
        frames=[values]
        # Backlash and motor lag do not invalidate encoder feedback. Only an
        # unusually large single-sample jump gets two immediate confirmation
        # reads. A consistent new position is accepted, even far from prediction.
        if any(v and abs(delta(r,self.raw[i]))>JUMP_CONFIRM for i,(v,r,s) in enumerate(values)):
            for _ in range(2):
                values,servos,moving=read(self.link);frames.append(values)
        now=time.monotonic()
        if now-self.when>.5:raise RuntimeError('Tracking gap: restart and re-zero')
        self.when=now;self.servos=servos
        for i in range(5):
            samples=sorted((delta(frame[i][1],self.raw[i]),frame[i][1],frame[i][2])
                           for frame in frames if frame[i][0])
            chosen=None
            if len(frames)==1 and samples:chosen=samples[0]
            elif len(samples)>=2:
                middle=len(samples)//2
                # Use an actual median reading, with at least one agreeing peer.
                candidate=samples[middle]
                if any(abs(candidate[0]-sample[0])<=CONFIRM_SPREAD for j,sample in enumerate(samples) if j!=middle):
                    chosen=candidate
            if chosen is not None:
                change,raw,steps=chosen
                measured=self.measured[i]+change
                self.pos[i]=self.measured[i]=measured;self.raw[i]=raw;self.steps[i]=steps
                self.fresh[i]=now;self.good[i]=True
            else:
                reason='I2C read failed' if not samples else 'inconsistent encoder confirmation reads'
                if self.good[i]:print(f'\n{NAMES[i]}: {reason}; briefly holding last measured angle')
                self.good[i]=False;self.pos[i]=self.measured[i]
                if now-self.fresh[i]>=GRACE:raise RuntimeError(f'{NAMES[i]} {reason} for 100ms; stopped')
        return self.pos[:]
    def prompt(self,text):
        print(text,end='',flush=True)
        while not select.select([sys.stdin],[],[],.02)[0]:self.update()
        line=sys.stdin.readline()
        if not line:raise EOFError
        self.update();return line.strip().lower()
    def zero(self):
        self.update()
        if not all(self.good):raise ValueError('Fresh encoder readings required for zero')
        self.pos=[0.]*5;self.measured=[0.]*5
    def pose(self):
        self.update()
        if not all(self.good):raise ValueError('Fresh encoder readings required')
        if any(s is None for s in self.servos):raise ValueError('Command both j6 and claw before saving/replaying')
        return self.pos[:]+self.servos[:]

class MotionStopped(Exception):pass
def console_stop():
    if select.select([sys.stdin],[],[],0)[0]:
        sys.stdin.readline();raise MotionStopped('Keyboard stop; sequence aborted')

def wrist_coordinates(values):
    a,b=values[3:5]
    return (b-a)/2,-(a+b)/2  # nominal up-positive pitch, right-positive roll

def jog_target(current,axis,angle):
    angle=finite(angle);target=current[:];active=[False]*5
    if axis in NAMES:
        i=NAMES.index(axis);target[i]=angle;active[i]=True
    elif axis in ('pitch','roll'):
        pitch,roll=wrist_coordinates(current)
        if axis=='pitch':pitch=angle
        else:roll=angle
        target[3:5]=[-pitch-roll,pitch-roll];active[3:5]=[True,True]
    else:raise ValueError('Use base, j1, j2, wa, wb, pitch, roll, j6 or claw')
    return target,active

def move(link,tracker,target,active=None,scale=1,interrupt=console_stop):
    # active=None is coordinated replay. A mask is a direct P joint jog.
    replay=active is None
    if not 0<scale<=1:raise ValueError('Scale must be >0 and <=1')
    if len(target)!=7:raise ValueError('Need seven target coordinates')
    for value in target[:5]:finite(value)
    for value in target[5:]:
        if value is not None:servo_pulse(value)
    if replay and any(v is None for v in target):raise ValueError('Replay requires both servo angles')
    start=tracker.update()+tracker.servos[:]
    if not all(tracker.good):raise ValueError('Fresh encoders required before moving')
    if replay and any(v is None for v in start):raise ValueError('Command both servos before replay')
    if not replay and (len(active)!=5 or not any(active)):raise ValueError('No joint selected')
    active=[True]*5 if replay else list(active)
    seconds=duration(start,target,scale) if replay else 0
    if replay:print(f'Coordinated segment: reference duration {seconds:.2f}s; all joints settle at end')
    previous=[0.]*5;begun=last=time.monotonic();settled=None;display=0
    try:
        tracker.motion=True
        while True:
            interrupt();pos=tracker.update();now=time.monotonic();dt=now-last;last=now;elapsed=now-begun
            if dt>.25:raise RuntimeError('Control-loop gap; stopped')
            s,ds=shape(elapsed/seconds) if replay else (1,0)
            rates=[]
            for i in range(5):
                if not active[i]:rates.append(0);continue
                ref=start[i]+(target[i]-start[i])*s
                error=ref-pos[i]
                allowed=tracking_allowance(i,ref,pos[i])
                if replay and abs(error)>allowed:raise RuntimeError(f'{NAMES[i]} trajectory tracking error {error:+.2f}° exceeds {allowed:g}°')
                envelope=travel_allowance(i,start[i],target[i],pos[i])
                if not min(start[i],target[i])-envelope<=pos[i]<=max(start[i],target[i])+envelope:
                    raise RuntimeError(f'{NAMES[i]} outside travel envelope: angle={pos[i]:+.2f} start={start[i]:+.2f} target={target[i]:+.2f}')
                ff=(target[i]-start[i])*ds/seconds if replay and elapsed<seconds else 0
                requested=max(-SPEED[i]*scale,min(SPEED[i]*scale,ff+KP[i]*error))
                if (not replay or elapsed>=seconds) and abs(target[i]-pos[i])<=TOL:requested=0
                velocity=max(previous[i]-ACCEL[i]*scale*dt,min(previous[i]+ACCEL[i]*scale*dt,requested))
                previous[i]=velocity
                cap=min(MAX_HZ[i],math.floor(SPEED[i]*scale*SPD[i]))
                rates.append(max(-cap,min(cap,round(velocity*SPD[i]))))
            servos=target[5:] if replay else tracker.servos[:]
            # Unexpected loss of servo signals must not silently re-enable them.
            if replay and any(v is None for v in tracker.servos):raise RuntimeError('Servo signals were lost; reinitialize explicitly')
            tracker.servos=send(link,rates,servos)
            reached=(not replay or elapsed>=seconds) and all(tracker.good[i] and abs(target[i]-pos[i])<=TOL for i in range(5) if active[i]) and not any(rates)
            if reached:
                if settled is None:settled=now
                if now-settled>=.25:break
            else:settled=None
            if replay and elapsed>seconds+15:raise RuntimeError('Trajectory failed to settle within 15s after reference')
            if now-display>.2:
                print('\r'+' | '.join(f'{n} {a:+.1f}' for n,a in zip(NAMES,pos)),end='',flush=True);display=now
            time.sleep(max(0,.02-(time.monotonic()-now)))
    finally:
        tracker.motion=False;stop(link);print()
    print('Reached. Step pulses stopped; servos hold last command (not measured).')

def set_servo(link,tracker,index,angle,interrupt=console_stop,scale=1):
    servo_pulse(angle)
    # No host-side speed limit, interpolation or delay, even at reduced scale.
    try:
        interrupt();tracker.update()
        values=tracker.servos[:];values[index]=angle
        tracker.servos=send(link,[0]*5,values)
    finally:stop(link)
    print(f'{("j6","claw")[index]} command = {tracker.servos[index]:.2f}°; no position feedback')

def pose_number(text):
    if not text.isascii() or not text.isdigit() or not 0<=int(text)<=99:raise ValueError('Pose number must be 0–99')
    return int(text)

class Book:
    def __init__(self):self.zeroed=False;self.poses={};self.queue=[]
    def zero(self,tracker):
        if self.poses:raise ValueError('Cannot change zero after saving poses; restart to redefine')
        tracker.zero();self.zeroed=True
    def save(self,number,tracker):
        number=pose_number(number)
        if not self.zeroed:raise ValueError('Use zero first')
        if number in self.poses:raise ValueError('Pose already exists; use a new number')
        self.poses[number]=tracker.pose()
    def targets(self,numbers):
        if not numbers:raise ValueError('Specify at least one saved pose')
        result=[]
        for value in numbers:
            key=pose_number(value)
            if key not in self.poses:raise ValueError(f'Pose {key} not saved')
            result.append(key)
        return result
    def write(self,path):
        # Audit only: never auto-load session-relative coordinates after reboot.
        data={'axis_order':NAMES+['j6','claw'],'encoder_ports':PORTS,'servo_pins':[15,14],
              'note':'Session-relative encoders; commanded servo angles. NOT auto-loadable.',
              'poses':self.poses,'queue':self.queue}
        temp=path.with_suffix('.tmp');temp.write_text(json.dumps(data,indent=2));temp.replace(path)

HELP='''zero          set current five encoder positions to zero (before saving)
base 10       move only base to +10°; also j1, j2, wa, wb
pitch 5       nominal differential pitch; roll 5 = nominal rightward roll
j6 150        command sixth-axis servo on pin15 (absolute 0–300°)
claw 100      command claw on pin14 (absolute 0–300°)
save 0        save measured five encoder angles + both servo commands
0             go to saved pose 0 (confirmation required)
queue 0 1 2 0 replace demo sequence with these saved pose numbers
run           run queued sequence, stopping at every waypoint
scale 0.5     half-speed/acceleration for steppers only; default1
w / list      current angles / saved poses and sequence
off           stop pulses AND disable servo signals (holding may be lost)
q             exit; servo signals disabled. Enter during a move stops sequence.
Servo motion is unmeasured. No IK, collision avoidance or calibrated joint limits.'''

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true',help='Read-only status/encoders; no motion commands')
    args=parser.parse_args()
    ports=glob.glob('/dev/serial/by-id/*Teensy*')
    if len(ports)!=1:raise RuntimeError(f'Expected one Teensy, found {ports}')
    with serial.Serial(ports[0],115200,timeout=.4,write_timeout=.4,exclusive=True) as link:
        if exchange(link,'PING')!=READY:raise RuntimeError('Upload waypoint_demo firmware first')
        tracker=Tracker(link)
        print('Absolute encoders:',dict(zip(NAMES,[r*360/4096 for r in tracker.raw])))
        print('Servo COMMAND angles (j6,claw):',tracker.servos)
        if args.check:return
        book=Book();scale=1.;log=Path(__file__).with_name('last_taught_poses.json')
        print(HELP)
        print('First servo command may jump from unknown position. Keep full path clear; support arm before off/quit.')
        print('Waypoint audit saved locally; restart requires zero and new poses. No automatic resume.')
        print('J1 backlash zone80–100° allows10° +0.5° tolerance; zero must use your original reference where upright=90°.')
        try:
            while True:
                parts=tracker.prompt('arm> ').split()
                if not parts:continue
                try:
                    if parts==['q']:break
                    if parts==['help']:print(HELP);continue
                    if parts==['zero'] or parts==['z']:book.zero(tracker);print('Encoder zero set; servo commands unchanged');continue
                    if parts==['off']:off(link);tracker.servos=[None,None];print('Servo signals OFF; no holding guarantee');continue
                    if parts==['w']:
                        print(dict(zip(NAMES,tracker.pos)),'nominal pitch/roll',wrist_coordinates(tracker.pos),'j6/claw commands',tracker.servos);continue
                    if parts==['list']:print(json.dumps({'poses':book.poses,'queue':book.queue},indent=2));continue
                    if parts[0]=='scale' and len(parts)==2:
                        value=finite(parts[1])
                        if not 0<value<=1:raise ValueError('Scale must be >0 and <=1')
                        scale=value;continue
                    if parts[0]=='save' and len(parts)==2:
                        book.save(parts[1],tracker);book.write(log);print('Saved',parts[1]);continue
                    if parts[0]=='queue':book.queue=book.targets(parts[1:]);book.write(log);print('Sequence:',book.queue);continue
                    if parts==['run'] or (len(parts)==1 and parts[0].isdigit()):
                        keys=book.targets([str(n) for n in book.queue] if parts==['run'] else parts)
                        tracker.pose()  # Require current trustworthy encoders + initialized servos.
                        if tracker.prompt(f'Run poses {keys}? Enter = go; anything else cancels: '):continue
                        for key in keys:print('Pose',key);move(link,tracker,book.poses[key],scale=scale)
                        continue
                    if len(parts)==2:
                        axis,angle=parts[0],finite(parts[1])
                        if axis in ('j6','claw'):
                            servo_pulse(angle);index=0 if axis=='j6' else 1
                            warning=' FIRST COMMAND MAY JUMP.' if tracker.servos[index] is None else ''
                            if tracker.prompt(f'{axis} to {angle}°?{warning} Enter = go; anything else cancels: '):continue
                            set_servo(link,tracker,index,angle,scale=scale);continue
                        if not book.zeroed:raise ValueError('Use zero before joint jogging')
                        target,active=jog_target(tracker.pos,axis,angle)
                        if tracker.prompt(f'{axis} to {angle:+g}°? Enter = go; anything else cancels: '):continue
                        # Unselected joints are not driven; use latest servo commands.
                        move(link,tracker,target+tracker.servos[:],active,scale);continue
                    raise ValueError('Unknown command; type help')
                except MotionStopped as e:print(e)
                except ValueError as e:print(e)
        finally:
            try:off(link)
            except Exception:print('OFF not acknowledged; firmware watchdogs remain active')

if __name__=='__main__':
    try:main()
    except (KeyboardInterrupt,EOFError):print('\nStopped; servo signals disabled or watchdog pending.')
    except (RuntimeError,serial.SerialException) as e:raise SystemExit(str(e))
