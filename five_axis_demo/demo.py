"""Five-axis encoder teach/replay. Shared quintic reference; no Cartesian IK."""
import argparse,glob,json,math,select,sys,time
from pathlib import Path
import serial

NAMES=['base','J1','J2','wrist_A','wrist_B']
PORTS=[0,1,3,5,4]
SPD=[3200*r/360 for r in [2,15,15,10,10]]
SPEED_CAPS=[5,5,10,10,10]
MAX_HZ=[math.floor(v*s) for v,s in zip(SPEED_CAPS,SPD)]
READY='READY FIVE_AXIS_V2 STEP=2,23,0,4,6 DIR=3,22,1,5,7 ENC=0,1,3,5,4 POS=1,0,1,1,1 HZ=88,666,1333,888,888'
TOL=.35
def delta(a,b):return ((a-b+2048)%4096-2048)*360/4096
def count_delta(a,b):return ((a-b+2**31)%2**32)-2**31
def exchange(link,command):
    link.write((command+'\n').encode());reply=link.readline().decode().strip()
    if not reply or reply.startswith('ERROR'):raise RuntimeError(reply or 'No serial response')
    return reply
def read(link):
    reply=exchange(link,'READ')
    if not reply.startswith('FRAME '):raise RuntimeError(reply)
    d=dict(x.split('=',1) for x in reply.split()[1:])
    if int(d['fault']):raise RuntimeError(f'Firmware stop fault={d["fault"]} (1=link,2=feedback,3=command)')
    result=[]
    for i in range(5):
        valid,raw,steps=[int(d[k+str(i)]) for k in ('v','r','s')]
        if valid not in (0,1) or not 0<=raw<4096 or not 0<=steps<2**32:raise RuntimeError('Malformed telemetry')
        result.append((bool(valid),raw,steps))
    return result,bool(int(d['moving']))
def shape(u):
    u=max(0,min(1,u));return 10*u**3-15*u**4+6*u**5,30*u*u*(1-u)**2
def speed_limits(speed=None):
    requested=SPEED_CAPS if speed is None else [speed]*5 if isinstance(speed,(float,int)) else list(speed)
    if len(requested)!=5 or not all(math.isfinite(v) and v>0 for v in requested):raise ValueError('Need five positive finite speed limits')
    return [min(v,c) for v,c in zip(requested,SPEED_CAPS)]
def duration(start,target,speed,accel,minimum=3):
    limits=speed_limits(speed)
    if len(start)!=5 or len(target)!=5 or not all(math.isfinite(x) for x in [*start,*target,accel,minimum]) or min(accel,minimum)<=0:raise ValueError('Nonfinite or nonpositive trajectory parameter')
    distances=[abs(b-a) for a,b in zip(start,target)]
    return max([minimum]+[1.875*d/v for d,v in zip(distances,limits)]+[math.sqrt(5.774*d/accel) for d in distances])
class Tracker:
    def __init__(self,link):
        self.link=link;deadline=time.monotonic()+2
        while True:
            sample,moving=read(link)
            if moving:raise RuntimeError('Controller moving; close other client first')
            if all(v for v,r,s in sample):break
            if time.monotonic()>deadline:raise RuntimeError('Need all five encoders: '+str({NAMES[i]:v for i,(v,r,s) in enumerate(sample)}))
            time.sleep(.03)
        self.raw=[r for v,r,s in sample];self.steps=[s for v,r,s in sample]
        self.measured=[0.]*5;self.pos=[0.]*5;self.fresh=[time.monotonic()]*5;self.good=[True]*5
    def update(self,motion=False):
        sample,moving=read(self.link);now=time.monotonic()
        for i,(valid,raw,steps) in enumerate(sample):
            predicted=self.pos[i]+count_delta(steps,self.steps[i])/SPD[i]
            self.steps[i]=steps;measured=self.measured[i]+delta(raw,self.raw[i])
            # Reject isolated implausible jumps without pausing the motors.
            accepted=valid and (not motion or abs(measured-predicted)<=5)
            if accepted:
                self.raw[i]=raw;self.measured[i]=measured;self.pos[i]=measured;self.fresh[i]=now
            else:self.pos[i]=predicted
            self.good[i]=accepted
            if now-self.fresh[i]>.3:raise RuntimeError(f'{NAMES[i]} encoder mux{PORTS[i]} missing/unusable for 300ms; stopped')
        return self.pos[:]
    def prompt(self,text):
        print(text,end='',flush=True)
        while not select.select([sys.stdin],[],[],.02)[0]:self.update()
        line=sys.stdin.readline()
        if not line:raise EOFError
        self.update();return line.strip()
    def zero(self):
        self.update()
        if not all(self.good):raise RuntimeError('Cannot zero without fresh feedback')
        self.measured=[0.]*5;self.pos=[0.]*5
class MotionStopped(Exception):pass
def console_stop():
    if select.select([sys.stdin],[],[],0)[0]:
        text=sys.stdin.readline()
        if not text:raise EOFError
        raise MotionStopped('Keyboard stop. Queued numbers are not replayed.')

class PoseBook:
    def __init__(self):self.poses={};self.mode='TEACH'
    def number(self,key,tracker):
        if not key.isascii() or not key.isdigit() or not 0<=int(key)<=99:raise ValueError('Use pose numbers 0–99')
        key=str(int(key))
        if self.mode=='REPLAY':
            if key not in self.poses:raise ValueError('That pose has not been saved')
            return 'move',key,self.poses[key][:]
        if key in self.poses:raise ValueError('Pose already saved; use a new number. Existing poses are not overwritten.')
        if not all(tracker.good):raise ValueError('Fresh encoder feedback required before saving')
        if not self.poses:
            if key!='0':raise ValueError('First type 0 at home')
            tracker.zero()
        self.poses[key]=tracker.pos[:]
        return 'saved',key,self.poses[key][:]
    def arm(self):
        if '0' not in self.poses:raise ValueError('Teach home pose 0 first')
        self.mode='REPLAY'
    def disarm(self):self.mode='TEACH'

def replay(link,tracker,target,speed,accel,minimum,interrupt=None):
    limits=speed_limits(speed)
    start=tracker.update();seconds=duration(start,target,speed,accel,minimum)
    if not all(tracker.good):raise RuntimeError('Fresh feedback required to start')
    print(f'Shared smooth reference: {seconds:.1f}s minimum; completion waits for all five encoders.')
    begun=last=time.monotonic();previous=[0.]*5;settled=None;display=0
    try:
        while True:
            if interrupt:interrupt()
            pos=tracker.update(motion=True);now=time.monotonic();dt=now-last;last=now;t=now-begun
            if dt>.25:raise RuntimeError('Control loop interrupted; stopped')
            s,ds=shape(t/seconds);rates=[]
            for i in range(5):
                reference=start[i]+(target[i]-start[i])*s
                ff=(target[i]-start[i])*ds/seconds if t<seconds else 0
                error=reference-pos[i]
                if abs(error)>4:raise RuntimeError(f'{NAMES[i]} tracking error >4°; stop and check direction/scaling/load')
                if not min(start[i],target[i])-2<=pos[i]<=max(start[i],target[i])+2:raise RuntimeError(f'{NAMES[i]} moved outside demo travel envelope')
                requested=max(-limits[i],min(limits[i],ff+1.2*error))
                if t>=seconds and abs(target[i]-pos[i])<=TOL:requested=0
                velocity=max(previous[i]-accel*dt,min(previous[i]+accel*dt,requested))
                previous[i]=velocity;rates.append(max(-MAX_HZ[i],min(MAX_HZ[i],round(velocity*SPD[i]))))
            if exchange(link,'VEL '+' '.join(map(str,rates)))!='OK VEL':raise RuntimeError('Velocity update failed')
            reached=t>=seconds and all(tracker.good) and all(abs(a-b)<=TOL for a,b in zip(pos,target))
            if reached:
                if settled is None:settled=now
                if now-settled>=.25:break
            else:settled=None
            if t>seconds+15:raise RuntimeError('Did not settle; check mechanics/calibration before retry')
            if now-display>.2:
                print('\r'+' | '.join(f'{n} {x:+.1f}/{g:+.1f}' for n,x,g in zip(NAMES,pos,target)),end='',flush=True);display=now
            time.sleep(max(0,.02-(time.monotonic()-now)))
    finally:
        exchange(link,'STOP');print()
    print('Pose reached. Pulses stopped; driver holding current remains.')
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true');parser.add_argument('--speed',type=float,default=None,help='Optional slower common ceiling; never exceeds per-axis caps')
    parser.add_argument('--accel',type=float,default=2);args=parser.parse_args()
    try:limits=speed_limits(args.speed)
    except ValueError as e:parser.error(str(e))
    if not 0<args.accel<=3:parser.error('Acceleration must be >0 and <=3 deg/s²')
    ports=glob.glob('/dev/serial/by-id/*Teensy*')
    if len(ports)!=1:raise RuntimeError(f'Expected one Teensy: {ports}')
    with serial.Serial(ports[0],115200,timeout=.2,write_timeout=.2,exclusive=True) as link:
        if exchange(link,'PING')!=READY:raise RuntimeError('Wrong five-axis firmware')
        tracker=Tracker(link)
        print('Absolute encoder degrees:',dict(zip(NAMES,[r*360/4096 for r in tracker.raw])))
        if args.check:return
        book=PoseBook();log=Path(__file__).with_name('last_taught_poses.json')
        print('TEACH: numbers SAVE poses. First 0 = home, then 1, 2, etc. No movement.')
        print('Support gravity-loaded links BEFORE disabling motor power. Keep Pi/Teensy/encoders powered.')
        print('When drives are ON and path clear: r = REPLAY. Then numbers MOVE immediately.')
        print('t = return to TEACH; l = list; w = angles; q = quit. Enter during motion stops/disarms.')
        print('No collision avoidance. Caps deg/s:',dict(zip(NAMES,limits)))
        print('Poses are session-only; saved JSON is an audit log, never auto-loaded. No per-move angle cap: check full path clearance.')
        try:
            while True:
                command=tracker.prompt(f'{book.mode} ({"numbers SAVE" if book.mode=="TEACH" else "numbers MOVE"})> ').strip().lower()
                if not command:continue
                try:
                    if command=='q':break
                    if command in ('w','where'):print(dict(zip(NAMES,tracker.pos)));continue
                    if command in ('l','list'):print(json.dumps(book.poses,indent=2));continue
                    if command=='t':exchange(link,'STOP');book.disarm();print('TEACH: numbers save. Drive holding current is not disabled.');continue
                    if command=='r':book.arm();print('REPLAY ARMED: numbers now MOVE. Keep hands/path clear. t disarms.');continue
                    if command.isdigit():
                        action,key,target=book.number(command,tracker)
                        if action=='saved':
                            log.write_text(json.dumps({'axis_order':NAMES,'encoder_ports':PORTS,'note':'session-relative; not safe to auto-reload','poses':book.poses},indent=2))
                            print('Saved pose',key);continue
                        print('Moving to pose',key)
                        replay(link,tracker,target,limits,args.accel,3,interrupt=console_stop);continue
                    print('Use 0,1,2,...; r replay; t teach; l list; w angles; q quit.')
                except MotionStopped as e:book.disarm();print(e,'Back in TEACH mode.')
                except ValueError as e:print(e)
        finally:exchange(link,'STOP')
if __name__=='__main__':
    try:main()
    except (KeyboardInterrupt,EOFError):print('\nStopped.')
    except (RuntimeError,serial.SerialException) as e:raise SystemExit(str(e))
