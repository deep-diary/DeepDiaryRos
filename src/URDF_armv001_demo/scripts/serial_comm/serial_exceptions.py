#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
串口通信包的自定义异常类
提供特定类型的错误处理
"""

class SerialCommException(Exception):
    """串口通信包的基础异常类"""
    pass

class ConfigError(SerialCommException):
    """配置错误异常"""
    pass

class ConnectionError(SerialCommException):
    """连接错误异常"""
    pass

class ReadError(SerialCommException):
    """读取错误异常"""
    pass

class WriteError(SerialCommException):
    """写入错误异常"""
    pass

class TimeoutError(SerialCommException):
    """超时错误异常"""
    pass

class PortNotFoundError(SerialCommException):
    """未找到端口异常"""
    pass

class CallbackError(SerialCommException):
    """回调函数执行错误"""
    pass 