"""Single-axis position P tuning for base/J1/J2. No wrist/servo control."""
import argparse
import glob
import math
import select
import sys
import time
import serial

READY='READY AXIS_P_V1 STEP=2,23,0 DIR=3,22,1 ENC=0,1,3 POS=1,0,1 HZ=88,666,1333'
AXES={'base':(0,2,5,0),'j1':(1,15,5,1),'j2':(2,15,10,3)}
TOL=.35
FILTER_GRACE=.1
FILTER_MARGIN=.75

def step_delta(a,b):return ((a-b+2**31)%2**32)-2**31

def positive(value):
    value=float(value)
    if not math.isfinite(value) or value<=0:raise ValueError('Value must be positive and finite')
    return value

def exchange(link,command):
    link.write((command+'\n').encode())
    reply=link.readline().decode().strip()
    if not reply or reply.startswith('ERROR'):raise RuntimeError(reply or 'No Teensy reply')
    return reply

def stop(link):
    if exchange(link,'STOP')!='OK STOP':raise RuntimeError('STOP not acknowledged')

def read(link):
    reply=exchange(link,'READ')
    if not reply.startswith('FRAME '):raise RuntimeError(reply)
    fields=dict(item.split('=',1) for item in reply.split()[1:])
    raw,steps,hz,fault=[int(fields[k]) for k in ('raw','steps','hz','fault')]
    if fault:raise RuntimeError(f'Firmware stop fault={fault} (1=link, 2=encoder, 3=command)')
    if not 0<=raw<4096 or not 0<=steps<2**32:raise RuntimeError('Invalid encoder frame')
    return raw,hz,steps

class Position:
    def __init__(self,link,spd=3200*15/360):
        self.link=link;self.spd=spd;self.raw,hz,self.steps=read(link)
        if hz:raise RuntimeError('Motor already running')
        self.angle=0.;self.when=time.monotonic()
        self.measured=0.;self.fresh=self.when;self.good=True;self.motion=False
        self.rejected=0;self.last_rejection='none'
    def update(self):
        raw,_,steps=read(self.link);now=time.monotonic()
        if now-self.when>.5:raise RuntimeError('Tracking gap: restart and set zero again')
        self.when=now
        measured=self.measured+((raw-self.raw+2048)%4096-2048)*360/4096
        predicted=self.measured+step_delta(steps,self.steps)/self.spd
        # During motion compare with pulses since the LAST ACCEPTED reading.
        # Idle allows hand-positioning up to 180 deg/s plus quantization margin.
        margin=FILTER_MARGIN if self.motion else FILTER_MARGIN+180*(now-self.fresh)
        if not self.good and now-self.fresh>=FILTER_GRACE:
            raise RuntimeError('Encoder filter: no accepted feedback for 100ms; '+self.last_rejection)
        if abs(measured-predicted)<=margin:
            self.angle=self.measured=measured;self.raw=raw;self.steps=steps
            self.fresh=now;self.good=True
        else:
            self.last_rejection=f'raw={raw} candidate={measured:+.3f}° predicted={predicted:+.3f}° difference={measured-predicted:+.3f}°'
            if self.good:print('\nRejected encoder jump: '+self.last_rejection,flush=True)
            self.good=False;self.rejected+=1
            if now-self.fresh>=FILTER_GRACE:
                raise RuntimeError('Encoder filter: no accepted feedback for 100ms; '+self.last_rejection)
            self.angle=predicted
        return self.angle
    def prompt(self,text):
        print(text,end='',flush=True)
        while not select.select([sys.stdin],[],[],.03)[0]:self.update()
        line=sys.stdin.readline()
        if not line:raise EOFError
        self.update();return line.strip().lower()
    def zero(self):
        self.update()
        if not self.good:raise ValueError('Wait for a valid encoder reading before setting zero')
        self.angle=self.measured=0.

class MotionStopped(Exception):pass
def keyboard_stop():
    if select.select([sys.stdin],[],[],0)[0]:
        sys.stdin.readline();raise MotionStopped('Keyboard stop')

def velocity(error,kp,cap,previous,accel,dt):
    # Begin braking before P alone would slow down. Reserve 20% deceleration
    # and at least two nominal control periods for sampling/serial latency.
    # Aim inside the tolerance band, not at its edge (avoids rounding stalls).
    distance=max(0.,abs(error)-TOL/2)
    braking_accel=.8*accel
    reaction=max(.04,dt)
    braking_speed=math.sqrt((braking_accel*reaction)**2+2*braking_accel*distance)-braking_accel*reaction
    requested=0 if abs(error)<=TOL else math.copysign(min(cap,kp*abs(error),braking_speed),error)
    return max(previous-accel*dt,min(previous+accel*dt,requested))

def move(link,pos,target,kp,cap,accel,spd,interrupt=keyboard_stop):
    if not math.isfinite(target):raise ValueError('Target must be finite')
    start=pos.update();last=time.monotonic();previous=0.;settled=None;display=0
    if not getattr(pos,'good',True):raise ValueError('Fresh encoder reading required before moving')
    try:
        pos.motion=True
        while True:
            interrupt()
            angle=pos.update();now=time.monotonic();dt=now-last;last=now
            if dt>.25:raise RuntimeError('Control loop gap; stopped')
            if not min(start,target)-2<=angle<=max(start,target)+2:
                raise RuntimeError(f'Outside target travel envelope: angle={angle:+.3f}° start={start:+.3f}° target={target:+.3f}° feedback={"encoder" if getattr(pos,"good",True) else "pulse prediction"}')
            error=target-angle
            previous=velocity(error,kp,cap,previous,accel,dt)
            hz=max(-math.floor(cap*spd),min(math.floor(cap*spd),round(previous*spd)))
            if exchange(link,f'VEL {hz}')!='OK VEL':raise RuntimeError('Velocity update failed')
            if abs(error)<=TOL and hz==0 and getattr(pos,'good',True):
                if settled is None:settled=now
                if now-settled>=.25:break
            else:settled=None
            if now-display>.1:
                print(f'\rangle {angle:+.2f}° | target {target:+.2f}° | error {error:+.2f}° | velocity {hz/spd:+.2f}°/s | Kp {kp:g}',end='',flush=True)
                display=now
            time.sleep(max(0,.02-(time.monotonic()-now)))
    finally:
        pos.motion=False
        stop(link);print()
    print(f'Reached {angle:+.2f}°. Pulses stopped; holding current remains.')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('axis',choices=AXES)
    parser.add_argument('--kp',type=float,default=1.)
    parser.add_argument('--speed',type=float,default=2.)
    parser.add_argument('--accel',type=float,default=2.,help='Acceleration/deceleration in deg/s² (default 2, maximum 10)')
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args();index,ratio,maximum,port=AXES[args.axis]
    try:kp=positive(args.kp);cap=positive(args.speed);accel=positive(args.accel)
    except ValueError as e:parser.error(str(e))
    if cap>maximum:parser.error(f'{args.axis} speed ceiling is {maximum}°/s')
    if accel>10:parser.error('Acceleration ceiling is 10°/s²')
    spd=3200*ratio/360
    ports=glob.glob('/dev/serial/by-id/*Teensy*')
    if len(ports)!=1:raise RuntimeError(f'Expected one Teensy; found {ports}')
    with serial.Serial(ports[0],115200,timeout=.4,write_timeout=.4,exclusive=True) as link:
        if exchange(link,'PING')!=READY:raise RuntimeError('Upload axis_p_tuner firmware first')
        if exchange(link,f'SELECT {index}')!=f'OK SELECT {index}':raise RuntimeError('Axis selection failed')
        pos=Position(link,spd)
        print(f'{args.axis}: encoder mux{port}, absolute {pos.raw*360/4096:.2f}°, ratio {ratio}:1, 3200 pulses/motor rev')
        if args.check:return
        print('Position P with distance-aware braking and speed/acceleration limits; no D term.')
        print('z: set zero; number: target degrees; k 1.2: gain; v 2: speed cap; w: position; q: quit')
        print('Enter during motion stops. Other axes receive NO step pulses. Support gravity-loaded links.')
        print('No automatic stall timeout or physical joint limits. Stay present; keep stop/power accessible.')
        zeroed=False
        try:
            while True:
                cmd=pos.prompt(f'{args.axis} Kp={kp:g} speed={cap:g}> ')
                try:
                    if cmd=='q':break
                    if cmd=='z':pos.zero();zeroed=True;print('Current position = 0°');continue
                    if cmd=='w':print(f'{pos.angle:+.2f}°'+(' relative to zero' if zeroed else ' since startup (set z first)'));continue
                    if cmd.startswith('k '):kp=positive(cmd[2:]);continue
                    if cmd.startswith('v '):
                        proposed=positive(cmd[2:])
                        if proposed>maximum:raise ValueError(f'Maximum {maximum}°/s')
                        cap=proposed;continue
                    if not cmd:continue
                    target=float(cmd)
                    if not zeroed:raise ValueError('Set zero with z before moving')
                    if not math.isfinite(target):raise ValueError('Target must be finite')
                    if pos.prompt(f'Move ONLY {args.axis} to {target:+g}°? Enter = go; anything else cancels: '):continue
                    move(link,pos,target,kp,cap,accel,spd)
                except (ValueError,MotionStopped) as e:print(e)
        finally:stop(link)

if __name__=='__main__':
    try:main()
    except (KeyboardInterrupt,EOFError):print('\nStopped.')
    except (RuntimeError,serial.SerialException) as e:raise SystemExit(str(e))
