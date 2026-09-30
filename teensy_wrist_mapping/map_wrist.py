"""Bounded open-loop mapping bursts; encoder reads only before and after motion."""
import argparse,glob,json,time
from pathlib import Path
import serial

MAX_PULSES=2667
READY=f'READY WRIST_MAP_V1 A=4,5 B=6,7 ENC=4,5 MAX={MAX_PULSES}'
def exchange(link,cmd):
    link.write((cmd+'\n').encode());reply=link.readline().decode().strip()
    if not reply or reply.startswith('ERROR'):raise RuntimeError(reply or 'No serial response')
    return reply
def fields(reply,prefix):
    if not reply.startswith(prefix+' '):raise RuntimeError(reply)
    return dict(part.split('=',1) for part in reply.split()[1:])
def read_pair(link):
    for attempt in range(3):
        r=fields(exchange(link,'ENCODERS'),'ENC')
        if r['valid4']=='1' and r['valid5']=='1':
            values=[int(r['raw4']),int(r['raw5'])]
            if not all(0<=v<4096 for v in values):raise RuntimeError('Invalid raw angle')
            return values
        time.sleep(.05)
    raise RuntimeError(f'Encoder read failed: {r}; no automatic motion retry')
def delta(a,b):return ((a-b+2048)%4096-2048)*360/4096
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--move',action='store_true');p.add_argument('--motor',choices=['A','B'],default='A')
    p.add_argument('--pulses',type=int,default=400);p.add_argument('--dir',type=int,choices=[0,1],default=0)
    args=p.parse_args()
    if not 1<=args.pulses<=MAX_PULSES:p.error(f'Use 1–{MAX_PULSES} pulses')
    ports=glob.glob('/dev/serial/by-id/*Teensy*')
    if len(ports)!=1:raise RuntimeError(f'Expected one Teensy: {ports}')
    with serial.Serial(ports[0],115200,timeout=1,write_timeout=1,exclusive=True) as link:
        if exchange(link,'PING')!=READY:raise RuntimeError('Wrong mapping firmware')
        status=fields(exchange(link,'STATUS'),'STATUS')
        if status['state']=='1':raise RuntimeError('Motor already moving')
        before=read_pair(link);print('Before:',dict(zip([4,5],[r*360/4096 for r in before])),flush=True)
        if not args.move:return
        i=0 if args.motor=='A' else 1
        print(f'Motor {args.motor}: STEP{4+2*i}/DIR{5+2*i}, DIR{args.dir}, {args.pulses} pulses at200Hz; nominal {args.pulses*360/32000:.2f} gearbox degrees.',flush=True)
        try:
            if exchange(link,f'MOVE {i} {args.pulses} 5000 {args.dir}')!='OK MOVE':raise RuntimeError('MOVE not acknowledged')
            deadline=time.monotonic()+args.pulses/200+3
            while True:
                s=fields(exchange(link,'STATUS'),'STATUS')
                if s['state']!='1':
                    if s['state']!='2' or int(s['pulses'])!=args.pulses:raise RuntimeError(f'Interrupted: {s}')
                    print('Completed:',s,flush=True);break
                if time.monotonic()>deadline:raise RuntimeError('Burst timeout')
                time.sleep(.05)
        finally:exchange(link,'STOP')
        time.sleep(.3);after=read_pair(link)
        result={'motor':args.motor,'dir':args.dir,'pulses':args.pulses,'before_raw':before,'after_raw':after,'delta_deg':dict(zip(['4','5'],[delta(a,b) for a,b in zip(after,before)]))}
        print(json.dumps(result),flush=True)
        with Path(__file__).with_name('mapping_results.jsonl').open('a') as log:log.write(json.dumps(result)+'\n')
if __name__=='__main__':main()
