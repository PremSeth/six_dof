"""Offline boundary checks; serial device never opened."""
import unittest
from unittest.mock import patch
import map_wrist as m

class Tests(unittest.TestCase):
    def test_limit_and_scaling(self):
        self.assertEqual(m.MAX_PULSES,2667)
        self.assertAlmostEqual(m.MAX_PULSES*360/32000,30.00375)
        self.assertAlmostEqual(m.MAX_PULSES/200,13.335)

    def test_reject_out_of_range_before_hardware(self):
        for n in (0,-1,2668):
            with patch('sys.argv',['map_wrist.py','--move','--pulses',str(n)]),patch.object(m.serial,'Serial') as serial:
                with self.assertRaises(SystemExit) as error:m.main()
                self.assertEqual(error.exception.code,2);serial.assert_not_called()

    def test_accept_limit_before_device_discovery(self):
        with patch('sys.argv',['map_wrist.py','--pulses','2667']),patch.object(m.glob,'glob',return_value=[]):
            with self.assertRaisesRegex(RuntimeError,'Expected one Teensy'):m.main()

if __name__=='__main__':unittest.main()
