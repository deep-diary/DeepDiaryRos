#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
协议处理包，用于处理不同产品的串口协议
作为串口通信层和应用层之间的桥梁
"""

# 使用本地导入 - Python 2.7 兼容
from protocol_processing.base import ProtocolProcessor, ProtocolCommand, ProtocolResponse
from protocol_processing.protocol_exceptions import *

# 导入产品协议
from protocol_processing.deep_arm.protocol import DeepArmProtocol
# 可以在此处添加其他产品协议的导入

__all__ = [
    # 基类
    'ProtocolProcessor',
    'ProtocolCommand',
    'ProtocolResponse',
    
    # 实现类
    'DeepArmProtocol',
    
    # 异常类
    'ProtocolError',
    'PacketError',
    'ChecksumError',
    'CommandError',
    'ResponseError',
    'TimeoutError',
    'ConfigError',
]

# 包版本
__version__ = '0.1.0'

# protocol_processing package 