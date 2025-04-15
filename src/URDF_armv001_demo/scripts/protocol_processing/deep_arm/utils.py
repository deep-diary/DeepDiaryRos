#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Deep Arm 协议专用工具函数
"""

import struct
from typing import Dict, Any, List, Optional, Union, Tuple

def parse_motor_angle(angle_value: float) -> Dict[str, Any]:
    """
    解析电机角度值
    
    Args:
        angle_value: 原始角度值
        
    Returns:
        Dict: 解析后的数据，包含度数、弧度等信息
    """
    import math
    radians = math.radians(angle_value)
    return {
        'degrees': angle_value,
        'radians': radians,
        'normalized': angle_value % 360
    }

def convert_motor_param(param_name: str, value: Any) -> Any:
    """
    转换电机参数值格式
    
    Args:
        param_name: 参数名称
        value: 原始参数值
        
    Returns:
        Any: 转换后的参数值
    """
    if param_name == 'angle':
        # 确保角度在有效范围内
        return float(value) % 360
    elif param_name == 'speed':
        # 确保速度为正值，限制最大速度
        speed = abs(int(value))
        return min(speed, 1000)  # 假设最大速度为1000
    elif param_name == 'motor_id':
        # 确保电机ID为有效值
        motor_id = int(value)
        return max(1, min(motor_id, 255))  # 假设ID范围为1-255
    
    # 默认不变换
    return value

def analyze_sensor_data(sensor_id: int, value: float) -> Dict[str, Any]:
    """
    分析传感器数据
    
    Args:
        sensor_id: 传感器ID
        value: 传感器原始值
        
    Returns:
        Dict: 分析结果
    """
    result = {'raw_value': value}
    
    # 根据不同传感器类型进行处理
    if 1 <= sensor_id <= 10:  # 假设1-10为位置传感器
        result['type'] = 'position'
        result['unit'] = 'mm'
        result['processed'] = value
    elif 11 <= sensor_id <= 20:  # 假设11-20为温度传感器
        result['type'] = 'temperature'
        result['unit'] = '°C'
        result['processed'] = value
    elif 21 <= sensor_id <= 30:  # 假设21-30为压力传感器
        result['type'] = 'pressure'
        result['unit'] = 'kPa'
        result['processed'] = value
    else:
        result['type'] = 'unknown'
        result['processed'] = value
    
    return result 