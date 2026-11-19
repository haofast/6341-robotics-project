"""Hardware-free regression checks: python -m unittest discover -s vision-service -p 'test_trace_perimeter.py'."""
import contextlib
import importlib.util
import io
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

spec = importlib.util.spec_from_file_location("trace_perimeter", Path(__file__).with_name("trace_perimeter.py"))
trace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trace)


class Model:
    def forward_kinematics(self, joints):
        pose = np.eye(4)
        pose[:3, 3] = np.asarray(joints)[:3] / 1000
        return pose


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Robot:
    is_calibrated = True
    is_connected = False

    def __init__(self, clock, final_error=0):
        self.clock = clock
        self.commanded = False
        self.final_error = final_error

    def connect(self, **kwargs):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def get_observation(self):
        q = np.zeros(5)
        if self.commanded and self.clock.now >= 1:
            q[0] = self.final_error
        return {name + ".pos": value for name, value in zip(trace.JOINTS, q)}

    def send_action(self, action):
        self.commanded = True
        return action


class TraceTests(unittest.TestCase):
    def run_endpoint(self, error):
        clock = Clock()
        robot = Robot(clock, error)
        with patch.object(trace, "follower", return_value=robot), \
             patch.object(trace.time, "monotonic", clock.monotonic), \
             patch.object(trace.time, "sleep", clock.sleep), \
             patch.object(trace, "release_after_failure"), \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            if error:
                with self.assertRaisesRegex(RuntimeError, "Endpoint settling timed out"):
                    trace.trace([(1, np.zeros(3), np.zeros(5))], np.zeros(5), Model(), "", "")
            else:
                trace.trace([(1, np.zeros(3), np.zeros(5))], np.zeros(5), Model(), "", "")
        self.assertGreaterEqual(clock.now, trace.ENDPOINT_SETTLE_S)
        self.assertFalse(robot.is_connected)

    def test_endpoint_waits_even_when_first_read_is_perfect(self):
        self.run_endpoint(0)

    def test_final_read_decides_not_initial_pass(self):
        self.run_endpoint(2)

    def test_mapping_reads_two_references_without_commands(self):
        board = {"columns": 6, "square_size_mm": 38.1,
                 "top_down_corners_px": [[0, 0], [100, 0]]}
        observation = {name + ".pos": 0 for name in trace.JOINTS}
        with patch.object(trace, "read_pose", return_value=observation) as read, \
             patch.object(trace, "follower") as follower, \
             patch("builtins.input", return_value=""), contextlib.redirect_stdout(io.StringIO()):
            trace.verify_mapping(board, np.zeros((4, 5)), Model(), "port", "id")
        self.assertEqual(read.call_count, 2)
        follower.assert_not_called()


if __name__ == "__main__":
    unittest.main()
