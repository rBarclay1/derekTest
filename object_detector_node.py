#!/usr/bin/env python3
"""
Standalone detector for topic/HSV checks. Prefer:
  ros2 launch jetrover_retrieve detect_only.launch.py
after copying jetrover_retrieve onto the robot.
"""

import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent / 'jetrover_retrieve'
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from jetrover_retrieve.object_detector_node import main

if __name__ == '__main__':
    main()
