#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
协议处理包的共享工具函数
"""

import struct
import logging
from typing import Dict, Any, List, Union

def calculate_crc16(data: bytes) -> int:
    """
    计算CRC16校验值 (MODBUS)
    
    Args:
        data: 要计算的字节数据
        
    Returns:
        int: CRC16校验值
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc = crc >> 1
    return crc

def calculate_checksum(data: bytes) -> int:
    """
    计算简单校验和
    
    Args:
        data: 要计算的字节数据
        
    Returns:
        int: 校验和（8位）
    """
    return sum(data) & 0xFF

def bytes_to_hex_str(data: bytes) -> str:
    """
    将字节数据转换为十六进制字符串
    
    Args:
        data: 字节数据
        
    Returns:
        str: 十六进制字符串，以空格分隔
    """
    return ' '.join(f'{b:02X}' for b in data)

def setup_logger(name: str, level: int = logging.INFO, log_file: str = None) -> logging.Logger:
    """
    设置日志记录器
    
    Args:
        name: 日志记录器名称
        level: 日志级别
        log_file: 日志文件路径，None则不记录到文件
        
    Returns:
        logging.Logger: 配置好的日志记录器
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(name)s: %(message)s')
    
    # 控制台处理器
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # 文件处理器
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    
    return logger 