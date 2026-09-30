"""Offline tests: no serial device is opened and no hardware is moved."""
import math
import unittest
from unittest.mock import patch
import base_velocity_test as j1


class FakePosition:
    def __init__(self, angles):
        self.angles = iter(angles)

    def update(self):
        return next(self.angles)


class Tests(unittest.TestCase):
    def test_base_pins_do_not_redirect_j2(self):
        import j2_velocity_test as j2
        self.assertEqual(j1.READY,'READY SIX_DOF_BASE_V1 STEP=2 DIR=3 INVERTED=0')
        self.assertEqual(j2.READY,'READY SIX_DOF_BASE_V1 STEP=0 DIR=1 INVERTED=0')

    def test_wrap(self):
        self.assertAlmostEqual(j1.delta(0, 4095), 360 / 4096)
        self.assertAlmostEqual(j1.delta(4095, 0), -360 / 4096)

    def test_rate(self):
        self.assertEqual(j1.parameters(10, 2), 28125)
        self.assertEqual(j1.parameters(-10, 120), 469)
        self.assertEqual(j1.parameters(1000000, 2), 28125)
        for velocity in (0, -1, math.nan, math.inf, .001, 1126):
            with self.assertRaises(ValueError):
                j1.parameters(10, velocity)
        with self.assertRaises(ValueError):
            j1.parameters(math.nan, 2)

    def run_move(self, angles, target, status='1'):
        commands = []
        def exchange(link, command):
            commands.append(command)
            if command.startswith('RUN '):
                return 'OK RUN'
            if command == 'STOP':
                return 'OK STOP'
            return f'STATUS state={status} pulses=1 target=2'
        return commands, exchange

    def test_forward_and_reverse(self):
        for target, angles, direction in [(1, [0, .5, 1.1, 1.1], 0),
                                           (-1, [0, -.5, -1.1, -1.1], 1)]:
            commands, exchange = self.run_move(angles, target)
            with patch.object(j1, 'exchange', exchange), patch.object(j1.time, 'sleep'):
                j1.move(None, FakePosition(angles), target, 2, 0)
            self.assertTrue(commands[0].endswith(f' {direction}'))
            self.assertEqual(commands[0], f'RUN 28125 {direction}')
            self.assertEqual(commands[-1], 'STOP')

    def test_wrong_direction_and_interruption(self):
        for angles, status in [([0, -2], '1'), ([0, .1], '2')]:
            commands, exchange = self.run_move(angles, 5, status)
            with patch.object(j1, 'exchange', exchange), self.assertRaises(RuntimeError):
                j1.move(None, FakePosition(angles), 5, 2, 0)
            self.assertEqual(commands[-1], 'STOP')

    def test_read_failure_stops(self):
        commands, exchange = self.run_move([], 5)
        position = FakePosition([0])
        with patch.object(j1, 'exchange', exchange), self.assertRaises(StopIteration):
            j1.move(None, position, 5, 2, 0)
        self.assertEqual(commands[-1], 'STOP')


if __name__ == '__main__':
    unittest.main()
