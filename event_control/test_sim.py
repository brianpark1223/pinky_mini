"""Launcher tests without ROS: python3 -m unittest discover -s event_control -p test_sim.py."""
import importlib.util
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


class SimulationLauncherTests(unittest.TestCase):
    def launch(self, extra):
        calls = []
        controller = SimpleNamespace(main=lambda args: calls.append(args))
        with patch.dict(sys.modules, fms_controller=controller):
            spec = importlib.util.spec_from_file_location(
                'sim_launcher_test', Path(__file__).with_name('fms_sim.py'))
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            with patch.object(sys, 'argv', ['fms_sim.py'] + extra):
                module.main()
        self.assertEqual(len(calls), 1)
        return calls[0]

    @staticmethod
    def ros_overrides(args):
        in_ros, overrides = False, []
        for index, token in enumerate(args):
            if token == '--ros-args':
                in_ros = True
            elif token == '--':
                in_ros = False
            elif in_ros and token == '-p':
                overrides.append(args[index + 1])
        return overrides

    def test_default_is_simulation(self):
        self.assertEqual(self.ros_overrides(self.launch([])), ['dry_run:=true'])

    def test_options_preserved_with_simulation_override_last(self):
        extra = ['--ros-args', '-p', 'docking:=true', '-p', 'dry_run:=false']
        result = self.launch(extra)
        self.assertEqual(result[1:1 + len(extra)], extra)
        self.assertEqual(self.ros_overrides(result),
                         ['docking:=true', 'dry_run:=false', 'dry_run:=true'])

    def test_closed_ros_block_cannot_disable_simulation(self):
        result = self.launch(['--ros-args', '-p', 'dry_run:=false', '--'])
        self.assertEqual(self.ros_overrides(result), ['dry_run:=false', 'dry_run:=true'])


if __name__ == '__main__':
    unittest.main()
