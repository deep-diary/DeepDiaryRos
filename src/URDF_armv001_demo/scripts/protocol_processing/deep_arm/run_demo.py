#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Deep Arm 演示程序运行器
"""

import os
import sys

# 添加包路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))

# 导入并运行演示
from protocol_processing.deep_arm.demo import main

if __name__ == "__main__":
    main() 