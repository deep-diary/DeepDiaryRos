#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

print("Python version: {}".format(sys.version))
print("Current directory: {}".format(os.getcwd()))
print("Script location: {}".format(os.path.abspath(__file__)))

# 修复导入路径问题 - 添加 scripts 目录到 Python 路径
current_dir = os.path.dirname(os.path.abspath(__file__))
scripts_dir = os.path.abspath(os.path.join(current_dir, '../../'))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)
    print("\nAdded to Python path: {}".format(scripts_dir))

print("\nPython path after fix:")
for p in sys.path:
    print(" - {}".format(p))

print("\nTrying to import serial_comm...")
try:
    import serial_comm
    print("SUCCESS: imported from {}".format(serial_comm.__file__))
except ImportError as e:
    print("FAILED: {}".format(e)) 