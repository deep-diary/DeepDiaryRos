#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
模板协议专用工具函数
"""

import struct
from typing import Dict, Any, Optional, Union

def is_valid_parameter(param_type: str, value: Any) -> bool:
    """
    验证参数值是否符合类型要求
    
    Args:
        param_type: 参数类型
        value: 参数值
        
    Returns:
        bool: 是否有效
    """
    try:
        if param_type == 'uint8':
            return isinstance(value, int) and 0 <= value <= 255
        elif param_type == 'uint16':
            return isinstance(value, int) and 0 <= value <= 65535
        elif param_type == 'float':
            return isinstance(value, (int, float))
        elif param_type == 'bool':
            return isinstance(value, bool) or (isinstance(value, int) and value in (0, 1))
        elif param_type == 'bytes':
            return isinstance(value, bytes)
        else:
            return True  # 未知类型默认有效
    except Exception:
        return False

def format_response_data(response_type: str, data: Dict[str, Any]) -> str:
    """
    格式化响应数据为可读字符串
    
    Args:
        response_type: 响应类型
        data: 响应数据
        
    Returns:
        str: 格式化后的字符串
    """
    if response_type == 'ack':
        status = data.get('status', -1)
        cmd_code = data.get('command_code', 0)
        return f"确认响应 [状态: {status}, 命令: 0x{cmd_code:02X}]"
    elif response_type == 'response1':
        result1 = data.get('result1', 0)
        result2 = data.get('result2', 0.0)
        return f"响应1 [结果1: {result1}, 结果2: {result2:.2f}]"
    elif response_type == 'response2':
        bin_data = data.get('data', b'')
        hex_str = ' '.join(f'{b:02X}' for b in bin_data)
        return f"响应2 [数据: {hex_str}]"
    else:
        return f"{response_type}: {data}" 