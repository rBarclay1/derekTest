#!/usr/bin/env python3
"""Find Derek's ROS environment, then run the stationary camera search."""
from pathlib import Path
from run_approach_trial import run_ros_script

if __name__ == '__main__':
    run_ros_script(Path(__file__).resolve().with_name('camera_search.py'), timeout=75)
