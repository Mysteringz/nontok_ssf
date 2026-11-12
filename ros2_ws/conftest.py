"""Lets the tests run without ROS: puts every package folder on the import path.

Run from ros2_ws/:  python3 -m pytest
"""
import pathlib
import sys

for package in sorted((pathlib.Path(__file__).parent / 'src').iterdir()):
    if (package / 'package.xml').exists():
        sys.path.insert(0, str(package))
