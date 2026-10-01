"""Offline tests: no real serial devices or motors."""
import io
import math
import json
import tempfile
import unittest
from pathlib import Path
from contextlib import redirect_stdout
from unittest.mock import patch
import demo as d

class Plant:
    def __init__(self,efficiency=None):
        self.clock=0.;self.last=0.;self.angles=[0.]*5;self.pulses=[0.]*5
        self.rates=[0]*5;self.servos=[150.,100.];self.commands=[]
        self.efficiency=efficiency or [1]*5
    def sample(self,link):
        dt=self.clock-self.last;self.last=self.clock
        for i in range(5):
            pulses=self.rates[i]*dt;self.pulses[i]+=pulses
            self.angles[i]+=pulses/d.SPD[i]*self.efficiency[i]
        values=[(1,(1000+round(a*4096/360))%4096,round(s)%2**32) for a,s in zip(self.angles,self.pulses)]
        return values,self.servos[:],any(self.rates)
    def exchange(self,link,cmd):
        self.commands.append(cmd)
        if cmd=='STOP':self.rates=[0]*5;return 'OK STOP'
        if cmd=='OFF':self.rates=[0]*5;self.servos=[None,None];return 'OK OFF'
        if cmd.startswith('SET '):
            values=list(map(int,cmd.split()[1:]));self.rates=values[:5]
            for i,p in enumerate(values[5:]):
                if p:self.servos[i]=d.servo_angle(p)
            return 'OK SET'
        raise AssertionError(cmd)
    def sleep(self,dt):
        self.clock+=max(.001,dt)
        if self.clock>180:raise RuntimeError('Simulation did not finish')

class Tests(unittest.TestCase):
    def test_j1_zone_allowance_is_local_and_bidirectional(self):
        for reference,measured in ((85,95),(95,85),(100,110),(80,70)):
            self.assertEqual(d.tracking_allowance(1,reference,measured),10.5)
            self.assertLessEqual(abs(reference-measured),d.tracking_allowance(1,reference,measured))
        self.assertGreater(abs(90-102),d.tracking_allowance(1,90,102))
        for reference,measured in ((20,30),(120,110)):
            self.assertEqual(d.tracking_allowance(1,reference,measured),4)
        for axis in (0,2,3,4):
            self.assertEqual(d.tracking_allowance(axis,90,100),4)
            self.assertEqual(d.travel_allowance(axis,0,100,110),2)
        self.assertEqual(d.travel_allowance(1,0,100,110),10.5)
        self.assertEqual(d.travel_allowance(1,100,80,70),10.5)
        self.assertEqual(d.travel_allowance(1,0,20,30),2)
        self.assertEqual(d.travel_allowance(1,0,100,120),2)
        self.assertEqual(d.TOL,.5)  # Endpoint accuracy is NOT relaxed.

    def test_mapping_and_tuned_limits(self):
        self.assertEqual(d.PORTS,[0,1,3,5,4])
        self.assertEqual(d.KP[:3],[3,2,2])
        self.assertEqual(d.SPEED[:3],[20,7,15])
        self.assertEqual(d.ACCEL[:3],[30,20,50])
        self.assertEqual(d.MAX_HZ,[355,933,2000,888,888])
        self.assertTrue(d.READY.endswith('SERVO=15,14'))
    def test_servo_mapping_and_limits(self):
        for angle,pulse in ((0,500),(150,1500),(300,2500)):
            self.assertEqual(d.servo_pulse(angle),pulse)
            self.assertEqual(d.servo_angle(pulse),angle)
        for bad in (-1,301,math.inf,math.nan):
            with self.assertRaises(ValueError):d.servo_pulse(bad)
        with patch.object(d,'exchange',return_value='OK SET') as ex:
            d.send(None,[0]*5,[150,100])
            ex.assert_called_once_with(None,'SET 0 0 0 0 0 1500 1167')
        with patch.object(d,'exchange') as ex,self.assertRaises(ValueError):d.send(None,[356,0,0,0,0],[None,None])
        ex.assert_not_called()
    def test_zero_servo_angle_is_not_off(self):
        with patch.object(d,'exchange',return_value='OK SET') as ex:
            self.assertEqual(d.send(None,[0]*5,[0,None]),[0,None])
            ex.assert_called_once_with(None,'SET 0 0 0 0 0 500 0')
    def test_jog_and_differential_mapping(self):
        current=[1,2,3,4,5]
        target,active=d.jog_target(current,'j1',10)
        self.assertEqual(target,[1,10,3,4,5]);self.assertEqual(active,[False,True,False,False,False])
        target,active=d.jog_target([0]*5,'pitch',5)
        self.assertEqual(target[3:],[-5,5]);self.assertEqual(d.wrist_coordinates(target),(5,0))
        target,active=d.jog_target(target,'roll',3)
        self.assertEqual(target[3:],[-8,2]);self.assertEqual(d.wrist_coordinates(target),(5,3))
        with self.assertRaises(ValueError):d.jog_target(current,'j7',1)
    def test_profile_limits_five_stepper_coordinates(self):
        start=[0]*5+[150,100];target=[90,-30,45,10,-10,210,160]
        for scale in (1,.5):
            T=d.duration(start,target,scale)
            for j in range(101):
                u=j/100;s,ds=d.shape(u)
                for i,(a,b) in enumerate(zip(start[:5],target[:5])):
                    self.assertLessEqual(abs((b-a)*ds/T),d.SPEED[i]*scale+1e-8)
                    self.assertLessEqual(abs((b-a)*(60*u-180*u*u+120*u**3)/T**2),d.ACCEL[i]*scale+1e-8)
        self.assertEqual(d.shape(0),(0,0));self.assertEqual(d.shape(1),(1,0))
        self.assertEqual(d.duration(start,start),d.duration(start,start[:5]+[0,300]))
    def test_book_requires_zero_fresh_data_and_servos(self):
        class Tracker:
            def zero(self):pass
            def pose(self):return [1,2,3,4,5,150,100]
        b=d.Book();t=Tracker()
        with self.assertRaises(ValueError):b.save('0',t)
        b.zero(t);b.save('0',t)
        self.assertEqual(b.poses[0],[1,2,3,4,5,150,100])
        with self.assertRaises(ValueError):b.save('0',t)
        with self.assertRaises(ValueError):b.zero(t)
        with self.assertRaises(ValueError):b.targets(['1'])
        with self.assertRaises(ValueError):b.targets([])
        self.assertEqual(b.targets(['0','0']),[0,0])
        for bad in ('-1','100','x'):
            with self.assertRaises(ValueError):d.pose_number(bad)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'poses.json';b.write(path)
            data=json.loads(path.read_text());self.assertEqual(data['servo_pins'],[15,14])
            self.assertEqual(data['poses']['0'],[1,2,3,4,5,150,100])
        self.assertEqual(d.Book().poses,{})  # No auto-load between sessions.
    def test_coordinated_replay_and_return(self):
        for efficiency in ([1]*5,[1,1,1,.5,.5]):
            plant=Plant(efficiency)
            with patch.object(d,'read',side_effect=plant.sample),patch.object(d,'exchange',side_effect=plant.exchange),patch.object(d.time,'monotonic',side_effect=lambda:plant.clock),patch.object(d.time,'sleep',side_effect=plant.sleep),redirect_stdout(io.StringIO()):
                tracker=d.Tracker(None)
                for target in ([5,-3,4,-2,2,160,110],[0]*5+[150,100]):
                    d.move(None,tracker,target,interrupt=lambda:None)
                    self.assertTrue(all(abs(a-b)<=d.TOL+.1 for a,b in zip(plant.angles,target[:5])))
                    self.assertTrue(all(abs(a-b)<.1 for a,b in zip(plant.servos,target[5:])))
            rows=[list(map(int,c.split()[1:])) for c in plant.commands if c.startswith('SET')]
            self.assertTrue(any(all(r!=0 for r in row[:5]) for row in rows))
            self.assertTrue(all(abs(r)<=cap for row in rows for r,cap in zip(row,d.MAX_HZ)))
            self.assertEqual(plant.commands[-1],'STOP')
            first=list(map(int,next(c for c in plant.commands if c.startswith('SET')).split()[1:]))
            self.assertEqual(first[5:],[d.servo_pulse(160),d.servo_pulse(110)])
    def test_only_selected_joint_receives_pulses(self):
        for selected in range(5):
            plant=Plant()
            with patch.object(d,'read',side_effect=plant.sample),patch.object(d,'exchange',side_effect=plant.exchange),patch.object(d.time,'monotonic',side_effect=lambda:plant.clock),patch.object(d.time,'sleep',side_effect=plant.sleep),redirect_stdout(io.StringIO()):
                tracker=d.Tracker(None);target=[0]*5+[150,100];target[selected]=3
                active=[i==selected for i in range(5)]
                d.move(None,tracker,target,active,interrupt=lambda:None)
            for row in (list(map(int,c.split()[1:])) for c in plant.commands if c.startswith('SET')):
                self.assertTrue(all(row[i]==0 for i in range(5) if i!=selected))
            self.assertLessEqual(abs(plant.angles[selected]-3),d.TOL+.1)
    def test_servo_jog_moves_no_steppers(self):
        for initial in ([150.,100.],[None,None]):
            plant=Plant();plant.servos=initial[:]
            with patch.object(d,'read',side_effect=plant.sample),patch.object(d,'exchange',side_effect=plant.exchange),patch.object(d.time,'monotonic',side_effect=lambda:plant.clock),patch.object(d.time,'sleep',side_effect=plant.sleep),redirect_stdout(io.StringIO()):
                tracker=d.Tracker(None);d.set_servo(None,tracker,0,160,interrupt=lambda:None)
            self.assertLess(abs(plant.servos[0]-160),.1)
            if initial[1] is None:self.assertIsNone(plant.servos[1])
            else:self.assertLess(abs(plant.servos[1]-initial[1]),.1)
            self.assertTrue(all(not any(map(int,c.split()[1:6])) for c in plant.commands if c.startswith('SET')))
            self.assertEqual(len([c for c in plant.commands if c.startswith('SET')]),1)
            self.assertEqual(plant.clock,0)  # No servo ramp or software delay.
    def test_outlier_recovery_and_persistent_failure(self):
        good=[(1,1000,0)]*5;bad=[(1,0,0)]*5
        with patch.object(d,'read',side_effect=[(good,[150,100],False),(bad,[150,100],True),(good,[150,100],False),(good,[150,100],False)]),patch.object(d.time,'monotonic',side_effect=[0,0,.02]),redirect_stdout(io.StringIO()):
            t=d.Tracker(None);t.motion=True;t.update()
            self.assertTrue(all(t.good));self.assertEqual(t.pos,[0]*5)
        missing=[(0,0,0)]*5
        with patch.object(d,'read',side_effect=[(good,[150,100],False),(missing,[150,100],False),(missing,[150,100],False)]),patch.object(d.time,'monotonic',side_effect=[0,0,.02,.12]),redirect_stdout(io.StringIO()):
            t=d.Tracker(None);t.motion=True;t.update()
            with self.assertRaises(RuntimeError):t.update()
    def test_backlash_lag_and_reversals_are_not_feedback_loss(self):
        # Pulses imply substantial travel; encoder sticks, then moves +/-8deg.
        raws=[1000,1000,1091,1000,909,1000]
        frames=[([(1,raw,idx*2000)]*5,[150,100],idx!=0) for idx,raw in enumerate(raws)]
        with patch.object(d,'read',side_effect=frames) as reader,patch.object(d.time,'monotonic',side_effect=[0,0,.02,.04,.06,.08,.1]):
            t=d.Tracker(None);t.motion=True
            for raw in raws[1:]:
                t.update();self.assertTrue(all(t.good))
                self.assertTrue(all(abs(p-d.delta(raw,1000))<1e-9 for p in t.pos))
            self.assertEqual(reader.call_count,len(raws))  # No extra polling for ordinary backlash.
    def test_large_consistent_change_is_accepted(self):
        good=[(1,1000,0)]*5;shifted=[(1,1200,0)]*5
        with patch.object(d,'read',side_effect=[(good,[150,100],False)]+[(shifted,[150,100],True)]*3),patch.object(d.time,'monotonic',side_effect=[0,0,.02]):
            t=d.Tracker(None);t.motion=True;t.update()
            self.assertTrue(all(t.good));self.assertAlmostEqual(t.pos[0],200*360/4096)
    def test_missing_reads_hold_measurement_not_pulse_prediction(self):
        good=[(1,1000,0)]*5;missing=[(0,0,2000)]*5
        with patch.object(d,'read',side_effect=[(good,[150,100],False),(missing,[150,100],True),(good,[150,100],False)]),patch.object(d.time,'monotonic',side_effect=[0,0,.02,.04]),redirect_stdout(io.StringIO()):
            t=d.Tracker(None);t.motion=True;t.update()
            self.assertEqual(t.pos,[0]*5);self.assertFalse(any(t.good))
            t.update();self.assertTrue(all(t.good))
    def test_no_save_when_servo_disabled(self):
        plant=Plant();plant.servos=[150,None]
        with patch.object(d,'read',side_effect=plant.sample):
            t=d.Tracker(None)
            with self.assertRaises(ValueError):t.pose()
    def test_stop_on_interrupt_and_error(self):
        for err in (d.MotionStopped('stop'),KeyboardInterrupt(),RuntimeError('read error')):
            plant=Plant()
            with patch.object(d,'read',side_effect=plant.sample),patch.object(d,'exchange',side_effect=plant.exchange),redirect_stdout(io.StringIO()):
                t=d.Tracker(None)
                def interrupt():raise err
                with self.assertRaises(type(err)):d.move(None,t,[1]*5+[150,100],interrupt=interrupt)
            self.assertEqual(plant.commands,['STOP'])
    def test_bad_targets_do_not_send(self):
        with patch.object(d,'exchange') as ex:
            for target in ([1]*5+[None,100],[1]*5+[301,100],[math.nan]*5+[150,100]):
                with self.assertRaises(ValueError):d.move(None,None,target)
            ex.assert_not_called()

    def test_ui_saves_measured_not_target_and_aborts_sequence(self):
        class Link:
            def __enter__(self):return self
            def __exit__(self,*args):pass
        class Tracker:
            raw=[1000]*5;pos=[0.]*5;servos=[150.,100.];good=[True]*5
            def update(self):return self.pos[:]
            def zero(self):self.pos=[0.]*5
            def pose(self):return self.pos[:]+self.servos[:]
            def prompt(self,text):return next(inputs)
        tracker=Tracker();book=d.Book();sent=[]
        inputs=iter(['zero','save 0','j1 10','','save 1','queue 0 1','run','','q'])
        def move(link,t,target,active=None,scale=1):
            if active is None:raise d.MotionStopped('test stop')
            t.pos=target[:5];t.pos[1]=9.75
        def exchange(link,cmd):
            sent.append(cmd)
            return {'PING':d.READY,'OFF':'OK OFF'}[cmd]
        with patch.object(d.sys,'argv',['demo.py']),patch.object(d.glob,'glob',return_value=['fake']),patch.object(d.serial,'Serial',return_value=Link()),patch.object(d,'exchange',side_effect=exchange),patch.object(d,'Tracker',return_value=tracker),patch.object(d,'Book',return_value=book),patch.object(book,'write'),patch.object(d,'move',side_effect=move) as mover,redirect_stdout(io.StringIO()):
            d.main()
        self.assertEqual(book.poses[1][1],9.75)
        self.assertEqual(book.poses[1][5:],[150,100])
        self.assertEqual(mover.call_count,2)  # one jog, first queued pose; no second pose
        self.assertEqual(sent,['PING','OFF'])

if __name__=='__main__':unittest.main()
