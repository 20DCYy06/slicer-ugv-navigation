"""Body-mode configuration only; no ROS or device access."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from control_core import load_config
CONFIG=Path(__file__).resolve().parents[2]/'ugv_bringup/config'
class BodyConfigTests(unittest.TestCase):
    def load_body(self, **updates):
        c=yaml.safe_load((CONFIG/'slicer_driver_wheel_speed_reference.yaml').read_text())
        c.update(control_mode='body_speed',**updates)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'cfg.yaml';p.write_text(yaml.safe_dump(c));return load_config(p)
    def test_body_mode_defaults_and_independent_polarity(self):
        c=self.load_body()
        self.assertEqual(c['control_mode'],'body_speed')
        self.assertEqual(c['body_feedback_timeout_s'],.30)
        self.assertEqual(c['body_imu_timeout_ms'],100)
        self.assertEqual(c['pwm_signs'],[-1,1,1,1])
    def test_body_mode_rejects_missing_signs_and_unenforceable_pwm_cap(self):
        for update in ({'pwm_signs':None},{'encoder_signs':None},{'ros_driver_pwm_limit_permille':100}):
            with self.subTest(update=update), self.assertRaises(ValueError):self.load_body(**update)
    def test_body_mode_rejects_invalid_feedback_timeouts(self):
        for update in ({'body_feedback_timeout_s':0},{'body_feedback_timeout_s':float('nan')},{'body_feedback_timeout_s':.6},{'body_imu_timeout_ms':0},{'body_imu_timeout_ms':101}):
            with self.subTest(update=update), self.assertRaises(ValueError):self.load_body(**update)
    def test_original_default_and_wheel_modes_remain_explicit(self):
        self.assertEqual(load_config(CONFIG/'slicer_driver.yaml')['control_mode'],'pwm_reference')
        self.assertEqual(load_config(CONFIG/'slicer_driver_wheel_speed_reference.yaml')['control_mode'],'wheel_speed')
if __name__=='__main__':unittest.main()
