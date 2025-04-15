#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
串口通信包，提供标准化的上下位机通信功能
"""

from .serial_manager import SerialManager
from .serial_utils import list_available_ports, get_port_by_hwid, get_port_by_description
from .serial_config import load_config
from .serial_exceptions import *

__all__ = [
    # 类
    'SerialManager',
    
    # 工具函数
    'list_available_ports',
    'get_port_by_hwid',
    'get_port_by_description',
    'load_config',
    
    # 异常类
    'SerialCommException',
    'ConfigError',
    'ConnectionError',
    'ReadError',
    'WriteError',
    'TimeoutError',
    'PortNotFoundError',
    'CallbackError'
]

# 包版本
__version__ = '1.0.0' 