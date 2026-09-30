"""Offline controller tests; no serial hardware accessed."""
import math
import unittest
from unittest.mock import patch
import wrist_position_test as w


class Tests(unittest.TestCase):
    def test_sequence_validation(self):
        self.assertEqual(w.parse_sequence('up 3; roll -2; axes 0 1'),
                         [('up 3',(-3,3)),('roll -2',(2,2)),('axes 0 1',(0,1))])
        for text in ('','up 3;',';up 3','up 3;;down 3','up 3; bad 2',';'.join(['up 1']*51)):
            with self.assertRaises(ValueError): w.parse_sequence(text)

    def test_sequence_order_and_stop(self):
        sequence=w.parse_sequence('up 3; roll -2; axes 0 1')
        with patch.object(w,'move') as move,patch.object(w,'exchange') as exchange:
            w.run_sequence('link',sequence,5)
        self.assertEqual([c.args for c in move.call_args_list],
                         [('link',(-3,3),5),('link',(2,2),5),('link',(0,1),5)])
        exchange.assert_called_once_with('link','STOP')

    def test_sequence_does_not_advance_after_fault_or_interrupt(self):
        sequence=w.parse_sequence('up 3; roll 3; down 3')
        for error in (RuntimeError('encoder fault'),KeyboardInterrupt()):
            with patch.object(w,'move',side_effect=[None,error,None]) as move, \
                 patch.object(w,'exchange') as exchange,self.assertRaises(type(error)):
                w.run_sequence(None,sequence,5)
            self.assertEqual(move.call_count,2)
            exchange.assert_called_once_with(None,'STOP')

    def test_roll_and_independent_commands(self):
        self.assertEqual(w.parse_command('roll 3'),(-3,-3))
        self.assertEqual(w.parse_command('roll -3'),(3,3))
        self.assertEqual(w.parse_command('axes -3 5'),(-3,5))
        self.assertEqual(w.parse_command('axes 0 3'),(0,3))
        self.assertEqual(w.parse_command('up 3'),(-3,3))
        self.assertEqual(w.parse_command('axes 0 0'),(0,0))
        for text in ('roll nan','axes 0 inf','axes -31 2','roll 31','axes 3','axes 1 2 3','roll 2 3','up -3'):
            with self.assertRaises(ValueError): w.parse_command(text)

    def test_unequal_and_zero_axis_targets_converge(self):
        for goal in ((-3,5),(0,3),(-3,0),(0,0)):
            p=[0.,0.]
            for tick in range(6000):
                rate=w.rates_for(p,goal,5)
                p[0]+=rate[0]/w.STEPS_PER_DEGREE*.01
                p[1]+=rate[1]/w.STEPS_PER_DEGREE*.01*.6
                if all(abs(g-x)<=w.TOLERANCE for g,x in zip(goal,p)): break
            else: self.fail(f'Unequal targets did not converge: {goal}, {p}')
            for i,g in enumerate(goal):
                if g==0: self.assertEqual(p[i],0)

    def test_step_counter_wrap(self):
        self.assertEqual(w.step_delta(0,2**32-1),1)
        self.assertEqual(w.step_delta(2**32-1,0),-1)

    def test_prediction_and_fresh_correction(self):
        axis=w.AxisEstimate(4090,0,0)
        self.assertAlmostEqual(axis.update(False,0,1,.05),1/w.STEPS_PER_DEGREE)
        # Encoder crosses wrap; recovery is actual displacement, not double-counted prediction.
        self.assertAlmostEqual(axis.update(True,2,4,.1),8*360/4096)
        self.assertEqual(axis.uncertainty,0)
        reverse=w.AxisEstimate(100,0,0)
        self.assertLess(reverse.update(False,0,2**32-1,.05),0)

    def test_prediction_bounds(self):
        axis=w.AxisEstimate(0,0,0)
        with self.assertRaises(RuntimeError): axis.update(False,0,1,.25)
        axis=w.AxisEstimate(0,0,0)
        with self.assertRaises(RuntimeError): axis.update(False,0,100,.05)
        axis=w.AxisEstimate(0,0,0)
        axis.update(True,100,0,.05)
        self.assertTrue(axis.verifying)
        with self.assertRaises(RuntimeError): axis.update(True,100,0,.3)

    def test_speed_polling(self):
        self.assertEqual(w.poll_interval(2),.05)
        self.assertEqual(w.poll_interval(20),.01)
        # Normal cycle plus 5ms I/O margin stays below the calibrated guard.
        for speed in (2,5,10,20,22.5):
            self.assertLess(math.ceil((w.poll_interval(speed)+.005)*speed*w.STEPS_PER_DEGREE),w.BLIND_STEP_LIMIT)

    def test_twenty_deg_speed_with_two_missed_samples(self):
        axis=w.AxisEstimate(2000,0,0)
        last_good_count=0
        for n in range(1,101):
            now=n*.01
            steps=round(20*w.STEPS_PER_DEGREE*now)
            raw=(2000+round(20*now*4096/360))%4096
            valid=n%20 not in (5,6)
            # Firmware should not exhaust its calibrated pulse allowance.
            self.assertLess(steps-last_good_count,w.BLIND_STEP_LIMIT)
            axis.update(valid,raw,steps,now)
            self.assertFalse(axis.verifying)
            if valid: last_good_count=steps
        self.assertAlmostEqual(axis.angle,20,delta=.1)

    def test_outlier_requires_two_confirmations(self):
        axis=w.AxisEstimate(1000,0,0)
        axis.update(True,1100,0,.02)
        self.assertTrue(axis.verifying)
        self.assertEqual(axis.raw,1000)
        axis.update(False,0,0,.04)
        self.assertTrue(axis.verifying)
        axis.update(True,1000,0,.06)
        self.assertTrue(axis.verifying)
        axis.update(True,1000,0,.08)
        self.assertFalse(axis.verifying)
        self.assertEqual(axis.angle,0)

    def test_outlier_does_not_pause_motion(self):
        clock=[1.0]; calls=[]; samples=[0]
        def sample(link):
            samples[0]+=1
            n=samples[0]
            if n==1: return [(True,0,0),(True,0,0)],3
            if n==2: return [(True,100,0),(True,0,0)],1
            if n in (3,4): return [(True,0,0),(True,0,0)],1
            return [(True,4096-6,2**32-3),(True,6,3)],1 if n==5 else 3
        def exchange(link,cmd):
            if cmd == 'WRIST_VEL 0 0':
                self.assertGreaterEqual(samples[0],5, 'Outlier must not pause motion')
            calls.append(cmd)
            return 'OK STOP' if cmd=='STOP' else 'OK WRIST_VEL'
        with patch.object(w,'initial_sample',return_value=[(True,0,0),(True,0,0)]), \
             patch.object(w,'telemetry',side_effect=sample), \
             patch.object(w,'exchange',side_effect=exchange), \
             patch.object(w.time,'monotonic',side_effect=lambda:clock[0]), \
             patch.object(w.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            w.move(None,(-.5,.5),20)
        hz = round(1.5 * w.STEPS_PER_DEGREE)
        self.assertEqual(calls,[f'WRIST_VEL {-hz} {hz}','WRIST_VEL 0 0','STOP'])
        self.assertEqual(calls[-1],'STOP')

    def test_move_requires_fresh_completion(self):
        clock=[1.0]
        calls=[]
        samples=[0]
        def sample(link):
            samples[0]+=1
            if samples[0]==1: return [(True,0,0),(True,0,0)],3
            steps = round(.5 * w.STEPS_PER_DEGREE)
            if samples[0]==2: return [(False,0,2**32-steps),(False,0,steps)],1
            return [(True,4096-6,2**32-steps),(True,6,steps)],3
        def exchange(link,command):
            calls.append(command)
            return 'OK STOP' if command=='STOP' else 'OK WRIST_VEL'
        with patch.object(w,'initial_sample',return_value=[(True,0,0),(True,0,0)]), \
             patch.object(w,'telemetry',side_effect=sample), \
             patch.object(w,'exchange',side_effect=exchange), \
             patch.object(w.time,'monotonic',side_effect=lambda:clock[0]), \
             patch.object(w.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            w.move(None,(-.5,.5),2)
        self.assertGreaterEqual(samples[0],6)
        hz = round(1.5 * w.STEPS_PER_DEGREE)
        self.assertIn(f'WRIST_VEL {-hz} {hz}',calls)
        self.assertEqual(calls[-1],'STOP')

    def test_move_stops_on_firmware_grace_fault(self):
        calls=[]
        with patch.object(w,'initial_sample',return_value=[(True,0,0),(True,0,0)]), \
             patch.object(w,'telemetry',return_value=([(False,0,0),(False,0,0)],5)), \
             patch.object(w,'exchange',side_effect=lambda link,cmd:calls.append(cmd)), \
             self.assertRaises(RuntimeError):
            w.move(None,(-3,3),2)
        self.assertEqual(calls,['STOP'])

    def test_modes(self):
        self.assertEqual(w.targets('up',3),(-3,3))
        self.assertEqual(w.targets('down',3),(3,-3))
        self.assertEqual(w.targets('left',3),(-3,-3))
        self.assertEqual(w.targets('right',3),(3,3))

    def test_wrap(self):
        self.assertAlmostEqual(w.wrap_delta(0,4095),360/4096)
        self.assertAlmostEqual(w.wrap_delta(4095,0),-360/4096)

    def test_leader_and_target(self):
        a,b=w.rates_for((-1,0),(-3,3),2)
        self.assertEqual(a,0)
        self.assertGreater(b,0)
        self.assertEqual(w.rates_for((-3,3),(-3,3),2),(0,0))
        self.assertGreater(w.rates_for((-3.5,3),(-3,3),2)[0],0)

    def test_bad_inputs(self):
        for amount in (0,-1,31,math.nan,math.inf):
            with self.assertRaises(ValueError): w.targets('up',amount)
        for speed in (0,-1,23,math.nan,math.inf):
            with self.assertRaises(ValueError): w.rates_for((0,0),(-3,3),speed)

    def test_unequal_plant_and_backlash(self):
        # B waits for 0.4s of initial backlash, then moves at half A's rate.
        for mode in w.MODES:
            goal=w.targets(mode,3)
            p=[0.,0.]
            max_skew=0
            for tick in range(4000):
                rates=w.rates_for(p,goal,2)
                p[0]+=rates[0]/w.STEPS_PER_DEGREE*.01
                if tick>=40: p[1]+=rates[1]/w.STEPS_PER_DEGREE*.01*.5
                max_skew=max(max_skew,abs(p[0]/goal[0]-p[1]/goal[1])*3)
                if all(abs(g-x)<=w.TOLERANCE for g,x in zip(goal,p)): break
            else: self.fail(f'{mode} did not converge: {p}')
            self.assertLessEqual(max_skew,w.SYNC_BAND+.03)


if __name__ == '__main__': unittest.main()
