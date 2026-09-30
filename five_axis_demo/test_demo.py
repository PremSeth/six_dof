"""Offline trajectory, pulse scaling and simulated motor tests."""
import io,math,unittest
from contextlib import redirect_stdout
from unittest.mock import patch
import demo as d

class Tests(unittest.TestCase):
    def test_independent_gains(self):
        self.assertEqual(d.gains(),[1.2]*5)
        original=d.gains()
        self.assertEqual(d.change_gain(original,'J1','0.8'),[1.2,.8,1.2,1.2,1.2])
        self.assertEqual(original,[1.2]*5)
        for axis in ('base','j1','j2','wrist_a','wrist_b'):
            updated=d.change_gain(original,axis,'2')
            self.assertEqual(updated[[n.lower() for n in d.NAMES].index(axis)],2)
        for bad in (0,-1,math.inf,math.nan):
            with self.assertRaises(ValueError):d.gains([bad]*5)
        with self.assertRaises(ValueError):d.gains([1]*4)
        with self.assertRaises(ValueError):d.change_gain(original,'pitch','1')

    def test_p_correction_sign_gain_and_saturation(self):
        self.assertEqual(d.corrected_velocity(0,2,.5,5),1)
        self.assertEqual(d.corrected_velocity(0,-2,1.5,5),-3)
        self.assertEqual(d.corrected_velocity(1,2,1.5,5),4)
        self.assertEqual(d.corrected_velocity(0,0,100,5),0)
        for sign in (-1,1):
            self.assertEqual(d.corrected_velocity(0,sign*10,100,5),sign*5)
    def test_requested_per_axis_caps(self):
        self.assertEqual(d.speed_limits(),[5,5,10,10,10])
        self.assertEqual(d.speed_limits(8),[5,5,8,8,8])
        self.assertEqual(d.MAX_HZ,[88,666,1333,888,888])
        self.assertTrue(all(h/s<=v for h,s,v in zip(d.MAX_HZ,d.SPD,d.SPEED_CAPS)))
        target=[15]*5;T=d.duration([0]*5,target,None,2)
        for n in range(101):
            _,v=d.shape(n/100)
            self.assertTrue(all(abs(g*v/T)<=cap+1e-8 for g,cap in zip(target,d.SPEED_CAPS)))

    def test_numbered_teach_and_replay(self):
        class Tracker:
            good=[True]*5
            pos=[1,2,3,4,5]
            def zero(self):self.pos=[0]*5
        tracker=Tracker();book=d.PoseBook()
        with self.assertRaises(ValueError):book.arm()
        with self.assertRaises(ValueError):book.number('1',tracker)
        self.assertEqual(book.number('0',tracker),('saved','0',[0]*5))
        tracker.pos=[1,2,3,4,5]
        self.assertEqual(book.number('1',tracker),('saved','1',[1,2,3,4,5]))
        with self.assertRaises(ValueError):book.number('0',tracker)
        book.arm()
        self.assertEqual(book.number('0',tracker),('move','0',[0]*5))
        self.assertEqual(book.number('1',tracker),('move','1',[1,2,3,4,5]))
        with self.assertRaises(ValueError):book.number('2',tracker)
        book.disarm();self.assertEqual(book.mode,'TEACH')
        self.assertEqual(book.number('2',tracker)[0],'saved')

    def test_keyboard_stop_sends_stop(self):
        class Tracker:
            good=[True]*5
            def update(self,motion=False):return [0]*5
        def interrupt():raise d.MotionStopped('Enter pressed')
        with patch.object(d,'exchange',return_value='OK STOP') as ex,redirect_stdout(io.StringIO()),self.assertRaises(d.MotionStopped):
            d.replay(None,Tracker(),[1]*5,None,2,3,interrupt=interrupt)
        ex.assert_called_once_with(None,'STOP')

    def test_axis_configuration(self):
        self.assertEqual(d.PORTS,[0,1,3,5,4])
        self.assertEqual(d.SPD,[3200*r/360 for r in [2,15,15,10,10]])
    def test_smooth_endpoints(self):
        self.assertEqual(d.shape(0),(0,0));self.assertEqual(d.shape(1),(1,0))
        s=[d.shape(i/100)[0] for i in range(101)]
        self.assertTrue(all(a<=b for a,b in zip(s,s[1:])))
    def test_shared_duration_limits(self):
        target=[15,2,-3,5,-5];T=d.duration([0]*5,target,2,2)
        for i in range(101):
            u=i/100;s,v=d.shape(u)
            for goal in target:
                self.assertLessEqual(abs(goal*v/T),2+1e-8)
                self.assertLessEqual(abs(goal*(60*u-180*u*u+120*u**3)/T**2),2+1e-8)
        for bad in (float('nan'),float('inf'),0,-1):
            with self.assertRaises(ValueError):d.duration([0]*5,target,bad,2)
    def test_larger_poses_keep_speed_and_acceleration_limits(self):
        target=[90,-45,30,-60,60]
        T=d.duration([0]*5,target,None,2)
        self.assertGreater(T,0)
        for n in range(101):
            u=n/100;_,v=d.shape(u)
            for g,cap in zip(target,d.SPEED_CAPS):
                self.assertLessEqual(abs(g*v/T),cap+1e-8)
                self.assertLessEqual(abs(g*(60*u-180*u*u+120*u**3)/T**2),2+1e-8)
    def test_wrap(self):
        self.assertAlmostEqual(d.delta(0,4095),360/4096)
        self.assertEqual(d.count_delta(0,2**32-1),1)
        self.assertEqual(d.count_delta(2**32-1,0),-1)
    def test_replay_converges_with_half_speed_wrists(self):
        for efficiency,kp in (([1]*5,None),([1,1,1,.5,.5],None),([1,1,1,.5,.5],[.8,1,1.4,1.6,2])):
            clock=[0.];rates=[0]*5;pos=[0.]*5;last=[0.];commands=[]
            class FakeTracker:
                good=[True]*5
                def update(self,motion=False):
                    dt=clock[0]-last[0];last[0]=clock[0]
                    for i in range(5):pos[i]+=rates[i]/d.SPD[i]*dt*efficiency[i]
                    return pos[:]
            def exchange(link,cmd):
                commands.append(cmd)
                if cmd=='STOP':rates[:]=[0]*5;return 'OK STOP'
                rates[:]=list(map(int,cmd.split()[1:]));return 'OK VEL'
            goal=[3,-3,2,-5,5]
            with patch.object(d.time,'monotonic',side_effect=lambda:clock[0]),patch.object(d.time,'sleep',side_effect=lambda t:clock.__setitem__(0,clock[0]+max(t,.001))),patch.object(d,'exchange',side_effect=exchange),redirect_stdout(io.StringIO()):
                d.replay(None,FakeTracker(),goal,2,2,3,kp=kp)
            self.assertEqual(commands[-1],'STOP')
            self.assertTrue(all(abs(a-b)<=d.TOL for a,b in zip(pos,goal)),pos)
            vectors=[list(map(int,c.split()[1:])) for c in commands if c.startswith('VEL')]
            self.assertTrue(any(all(v!=0 for v in row) for row in vectors))
            self.assertTrue(all(abs(v)<=round(2*d.SPD[i]) for row in vectors for i,v in enumerate(row)))
    def test_error_and_interrupt_stop_all(self):
        class FakeTracker:
            good=[True]*5
            def __init__(self,error):self.n=0;self.error=error
            def update(self,motion=False):
                self.n+=1
                if self.n>1:raise self.error
                return [0]*5
        for err in (RuntimeError('feedback failure'),KeyboardInterrupt()):
            with patch.object(d,'exchange',return_value='OK STOP') as ex,redirect_stdout(io.StringIO()),self.assertRaises(type(err)):
                d.replay(None,FakeTracker(err),[1]*5,2,2,3)
            ex.assert_called_once_with(None,'STOP')
    def test_tracker_tolerates_one_bad_frame(self):
        fresh=[(True,1000,0)]*5;bad=[(False,0,1)]*5;recovered=[(True,1001,1)]*5
        with patch.object(d,'read',side_effect=[(fresh,False),(bad,True),(recovered,True)]),patch.object(d.time,'monotonic',return_value=1):
            t=d.Tracker(None);t.update(motion=True)
            self.assertFalse(any(t.good));self.assertGreater(t.pos[0],0)
            t.update(motion=True);self.assertTrue(all(t.good))
    def test_tracker_stops_sustained_feedback_loss(self):
        fresh=[(True,1000,0)]*5;bad=[(False,0,0)]*5
        with patch.object(d,'read',side_effect=[(fresh,False),(bad,True)]),patch.object(d.time,'monotonic',side_effect=[0,0,.31]):
            t=d.Tracker(None)
            with self.assertRaisesRegex(RuntimeError,'300ms'):t.update(motion=True)

if __name__=='__main__':unittest.main()
