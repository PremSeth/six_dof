"""Offline tests. No serial device is opened or physical motor moved."""
import io
import itertools
import math
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch
import tune as t

class Tests(unittest.TestCase):
    def test_mapping(self):
        self.assertEqual(t.AXES,{'base':(0,2,5,0),'j1':(1,15,5,1),'j2':(2,15,10,3)})
    def test_positive_parameters(self):
        self.assertEqual(t.positive('1.2'),1.2)
        for value in ('nan','inf','0','-1','bad'):
            with self.assertRaises(ValueError):t.positive(value)
    def test_p_gain_sign_deadband_caps_and_acceleration(self):
        self.assertEqual(t.velocity(2,1,5,2,3,.02),2)
        self.assertEqual(t.velocity(-2,1,5,-2,3,.02),-2)
        self.assertEqual(t.velocity(2,.5,5,1,3,.02),1)
        self.assertEqual(t.velocity(.1,5,5,0,3,1),0)
        self.assertEqual(t.velocity(100,100,5,0,3,.02),.06)
        self.assertEqual(t.velocity(100,100,5,5,3,.02),5)

    def test_long_moves_at_acceleration_fifty(self):
        # Quantized pulse speeds, both signs, all axes, high gains and jitter.
        for _,ratio,cap,_ in t.AXES.values():
            spd=3200*ratio/360
            for target in (-90,-30,30,90):
                for kp,accel in itertools.product((.8,1.5,5),(50,)):
                    angle=previous=0.;peak=0.;settled=0
                    for tick in range(15000):
                        dt=(.02,.025,.015)[tick%3]
                        error=target-angle
                        velocity=t.velocity(error,kp,cap,previous,accel,dt)
                        self.assertLessEqual(abs(velocity-previous),accel*dt+1e-9)
                        self.assertLessEqual(abs(velocity),cap+1e-9)
                        previous=velocity
                        hz=max(-math.floor(cap*spd),min(math.floor(cap*spd),round(velocity*spd)))
                        angle+=hz/spd*dt
                        peak=max(peak,(angle-target)*(1 if target>0 else -1))
                        settled=settled+1 if abs(target-angle)<=t.TOL and hz==0 else 0
                        if settled>=20:break
                    else:self.fail(f'Failed to settle target={target}, kp={kp}, cap={cap}')
                    self.assertLess(peak,.5,(target,kp,cap,peak))

    def test_no_distance_braking_and_slew_limit_retained(self):
        self.assertEqual(t.velocity(20,1,10,10,2,.02),10)
        self.assertEqual(t.velocity(-20,1,10,-10,2,.02),-10)
        for accel in (2,10,50):
            self.assertAlmostEqual(t.velocity(30,1,10,0,accel,.02),accel*.02)
            self.assertAlmostEqual(t.velocity(-30,1,10,0,accel,.02),-accel*.02)
            self.assertAlmostEqual(t.velocity(0,1,10,10,accel,.02),10-accel*.02)
    def test_wrap(self):
        with patch.object(t,'read',side_effect=[(4095,0,0),(0,0,0),(4095,0,0)]),patch.object(t.time,'monotonic',return_value=1):
            pos=t.Position(None)
            self.assertAlmostEqual(pos.update(),360/4096)
            self.assertAlmostEqual(pos.update(),0)
    def test_firmware_fault_and_bad_frame(self):
        for reply in ('FRAME raw=1 steps=0 hz=0 fault=2','FRAME raw=4096 steps=0 hz=0 fault=0','wrong'):
            with patch.object(t,'exchange',return_value=reply),self.assertRaises(RuntimeError):t.read(None)
    def test_error_and_keyboard_stop(self):
        class Position:
            def update(self):return 0
        for error in (t.MotionStopped('stop'),KeyboardInterrupt(),RuntimeError('bus')):
            def interrupt():raise error
            with patch.object(t,'stop') as stop,redirect_stdout(io.StringIO()),self.assertRaises(type(error)):
                t.move(None,Position(),3,1,2,2,3200*15/360,interrupt)
            stop.assert_called_once_with(None)
    def test_read_error_stops(self):
        class Position:
            calls=0
            def update(self):
                self.calls+=1
                if self.calls>1:raise RuntimeError('encoder failure')
                return 0
        with patch.object(t,'stop') as stop,redirect_stdout(io.StringIO()),self.assertRaises(RuntimeError):
            t.move(None,Position(),3,1,2,2,3200*15/360,lambda:None)
        stop.assert_called_once_with(None)
    def test_all_axes_converge_both_directions(self):
        for _,ratio,cap,_ in t.AXES.values():
            for target in (-3,3):
                for kp in (.5,1.2,2):
                    clock=[0.];last=[0.];angle=[0.];rate=[0];commands=[];spd=3200*ratio/360
                    class Position:
                        def update(self):
                            dt=clock[0]-last[0];last[0]=clock[0]
                            angle[0]+=rate[0]/spd*dt
                            return angle[0]
                    def exchange(link,cmd):
                        commands.append(cmd)
                        if cmd=='STOP':rate[0]=0;return 'OK STOP'
                        self.assertTrue(cmd.startswith('VEL '))
                        rate[0]=int(cmd.split()[1]);self.assertLessEqual(abs(rate[0]),math.floor(cap*spd))
                        return 'OK VEL'
                    def interrupt():
                        if clock[0]>60:raise RuntimeError('Simulation failed to converge')
                    with patch.object(t,'exchange',side_effect=exchange),patch.object(t.time,'monotonic',side_effect=lambda:clock[0]),patch.object(t.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+max(.001,dt))),redirect_stdout(io.StringIO()):
                        t.move(None,Position(),target,kp,cap,2,spd,interrupt)
                    self.assertLessEqual(abs(target-angle[0]),t.TOL)
                    self.assertEqual(commands[-1],'STOP')
    def test_zero_gate_and_gain_changes_issue_no_velocity(self):
        class Link:
            def __enter__(self):return self
            def __exit__(self,*args):pass
        class Position:
            raw=1000;angle=0
            def __init__(self,link,spd):pass
            def prompt(self,text):return next(commands)
            def zero(self):pass
        commands=iter(['5','k 0.8','v 1','z','q'])
        sent=[]
        def exchange(link,cmd):
            sent.append(cmd)
            return {'PING':t.READY,'SELECT 2':'OK SELECT 2','STOP':'OK STOP'}[cmd]
        with patch.object(t.sys,'argv',['tune.py','j2','--accel','50']),patch.object(t.glob,'glob',return_value=['fake']),patch.object(t.serial,'Serial',return_value=Link()),patch.object(t,'Position',Position),patch.object(t,'exchange',side_effect=exchange),redirect_stdout(io.StringIO()):
            t.main()
        self.assertEqual(sent,['PING','SELECT 2','STOP'])

    def test_acceleration_above_fifty_rejected_before_serial(self):
        with patch.object(t.sys,'argv',['tune.py','j2','--accel','50.1']),patch.object(t.serial,'Serial') as serial,patch.object(t.sys,'stderr',io.StringIO()),self.assertRaises(SystemExit) as error:
            t.main()
        self.assertEqual(error.exception.code,2)
        serial.assert_not_called()

    def test_single_zero_glitch_rejected_then_recovers(self):
        samples=[(1000,0,0),(999,-100,2**32-12),(0,-100,2**32-24),(998,-100,2**32-24)]
        with patch.object(t,'read',side_effect=samples),patch.object(t.time,'monotonic',side_effect=[0,.02,.04,.06]),redirect_stdout(io.StringIO()):
            p=t.Position(None);p.motion=True
            self.assertAlmostEqual(p.update(),-360/4096)
            estimated=p.update()
            self.assertFalse(p.good);self.assertEqual(p.raw,999)
            self.assertLess(abs(estimated),.3)
            self.assertAlmostEqual(p.update(),-2*360/4096)
            self.assertTrue(p.good);self.assertEqual(p.rejected,1)

    def test_persistent_bad_angles_stop(self):
        with patch.object(t,'read',side_effect=[(1000,0,0),(0,0,0),(0,0,0)]),patch.object(t.time,'monotonic',side_effect=[0,.02,.12]),redirect_stdout(io.StringIO()):
            p=t.Position(None);p.motion=True;p.update()
            with self.assertRaisesRegex(RuntimeError,'100ms'):p.update()

    def test_zero_and_target_start_require_real_feedback(self):
        with patch.object(t,'read',side_effect=[(1000,0,0),(0,0,0)]),patch.object(t.time,'monotonic',side_effect=[0,.001]),redirect_stdout(io.StringIO()):
            p=t.Position(None)
            with self.assertRaises(ValueError):p.zero()
        class BadPosition:
            good=False
            def update(self):return 0
        with patch.object(t,'exchange') as exchange,self.assertRaises(ValueError):
            t.move(None,BadPosition(),1,1,2,2,100)
        exchange.assert_not_called()

    def test_real_gradual_overshoot_is_not_filtered_out(self):
        samples=[(1000-i,0,0) for i in range(31)]
        with patch.object(t,'read',side_effect=samples),patch.object(t.time,'monotonic',side_effect=[i*.02 for i in range(31)]):
            p=t.Position(None);p.motion=True
            for _ in range(30):p.update();self.assertTrue(p.good)
            self.assertLess(p.angle,-2)
        self.assertEqual(t.step_delta(0,2**32-1),1)
        self.assertEqual(t.step_delta(2**32-1,0),-1)

    def test_move_survives_injected_zero_angle(self):
        clock=[0.];last=[0.];angle=[0.];rate=[0];injected=[False];commands=[]
        spd=3200*15/360
        def sensor(link):
            dt=clock[0]-last[0];last[0]=clock[0]
            angle[0]+=rate[0]/spd*dt
            raw=(1000+round(angle[0]*4096/360))%4096
            if clock[0]>.8 and not injected[0]:raw=0;injected[0]=True
            return raw,rate[0],round(angle[0]*spd)%2**32
        def exchange(link,cmd):
            commands.append(cmd)
            if cmd=='STOP':rate[0]=0;return 'OK STOP'
            rate[0]=int(cmd.split()[1]);return 'OK VEL'
        def interrupt():
            if clock[0]>30:raise RuntimeError('Did not converge')
        with patch.object(t,'read',side_effect=sensor),patch.object(t,'exchange',side_effect=exchange),patch.object(t.time,'monotonic',side_effect=lambda:clock[0]),patch.object(t.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+max(dt,.001))),redirect_stdout(io.StringIO()):
            pos=t.Position(None,spd)
            t.move(None,pos,-3,1,2,2,spd,interrupt)
        self.assertTrue(injected[0]);self.assertEqual(pos.rejected,1)
        self.assertTrue(pos.good);self.assertLessEqual(abs(angle[0]+3),t.TOL+.1)
        self.assertEqual(commands[-1],'STOP')

if __name__=='__main__':unittest.main()
