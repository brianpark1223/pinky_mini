"""Simulation run of the 2-robot FMS: nothing moves.

Same controller as fms_controller.py, started with dry_run:=true:
each trip "arrives" after 2 seconds, docking (if on) finishes after 2 seconds,
no Nav2 goals are sent. Use it with delivery_gui.py to try the buttons and the flow.

    python3 fms_sim.py
    python3 fms_sim.py --ros-args -p dry_run_travel:=5.0     # slower virtual trips
    python3 fms_sim.py --ros-args -p docking:=true           # include the DOCKING step

Real mode: fms_controller.py with measured coordinate files, then /delivery/arm.
See README.md for the required configuration.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fms_controller  # noqa: E402


def main():
    extra = sys.argv[1:]
    # End an open ROS argument block before adding the final simulation override.
    # A user-supplied '--' must not turn dry_run:=true into a non-ROS argument.
    if '--ros-args' in extra:
        last_ros = len(extra) - 1 - extra[::-1].index('--ros-args')
        if '--' not in extra[last_ros + 1:]:
            extra = extra + ['--']
    args = [sys.argv[0]] + extra + ['--ros-args', '-p', 'dry_run:=true']
    fms_controller.main(args)


if __name__ == '__main__':
    main()
