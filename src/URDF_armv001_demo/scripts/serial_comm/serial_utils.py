#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
串口通信包的工具函数
提供辅助功能
"""

import os
import sys
import glob
import logging
import time
import serial
import serial.tools.list_ports

def list_available_ports():
    """
    列出系统上所有可用的串口设备
    
    Returns:
        list: 包含串口信息的列表，每个元素是一个字典 {port, desc, hwid}
    """
    try:
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append({
                'port': port.device,
                'desc': port.description,
                'hwid': port.hwid
            })
        return ports
    except Exception as e:
        logging.error("Error listing available serial ports: %s" % e)
        return []

def get_port_by_hwid(hwid):
    """
    通过硬件ID查找串口
    
    Args:
        hwid (str): 硬件ID字符串或其子串
        
    Returns:
        str: 匹配的串口名，如果未找到则返回None
    """
    try:
        for port in serial.tools.list_ports.comports():
            if hwid in port.hwid:
                return port.device
        return None
    except Exception as e:
        logging.error("Error finding port by HWID: %s" % e)
        return None

def get_port_by_description(desc):
    """
    Find port by description
    
    Args:
        desc (str): Description string or its substring
        
    Returns:
        str: The name of the matched port, or None if not found
    """
    try:
        for port in serial.tools.list_ports.comports():
            if desc in port.description:
                return port.device
        return None
    except Exception as e:
        logging.error("Error finding port by description: %s" % e)
        return None

def setup_logger(name, level=logging.INFO, log_file=None):
    """
    设置日志记录器
    
    Args:
        name (str): 日志记录器名称
        level: 日志级别
        log_file (str): 日志文件路径，如果为None则不记录到文件
        
    Returns:
        logging.Logger: 配置好的日志记录器
    """
    # 创建日志记录器
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # 创建格式化器
    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')
    
    # 添加控制台处理器
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # 如果指定了日志文件，添加文件处理器
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    
    return logger

def str_to_bytes(data, encoding='utf-8'):
    """
    将字符串转换为字节，兼容Python 2和3
    
    Args:
        data (str): 输入字符串
        encoding (str): 字符编码
        
    Returns:
        bytes: 转换后的字节
    """
    if sys.version_info[0] >= 3:
        if isinstance(data, str):
            return data.encode(encoding)
        return data
    else:
        if isinstance(data, unicode):
            return data.encode(encoding)
        return data

def bytes_to_str(data, encoding='utf-8'):
    """
    将字节转换为字符串，兼容Python 2和3
    
    Args:
        data (bytes): 输入字节
        encoding (str): 字符编码
        
    Returns:
        str: 转换后的字符串
    """
    if sys.version_info[0] >= 3:
        if isinstance(data, bytes):
            return data.decode(encoding)
        return data
    else:
        return data 