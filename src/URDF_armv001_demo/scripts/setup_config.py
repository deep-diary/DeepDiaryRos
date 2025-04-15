#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import yaml
import sys

def ensure_config_exists(config_dir):
    """确保配置文件存在"""
    serial_config_path = os.path.join(config_dir, "serial_config.yaml")
    
    # 检查目录是否存在
    if not os.path.exists(config_dir):
        os.makedirs(config_dir)
        print(f"创建配置目录: {config_dir}")
    
    # 检查配置文件是否存在
    if not os.path.exists(serial_config_path):
        # 创建默认配置
        default_config = {
            'serial': {
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'timeout': 0.1,
                'update_rate': 50
            }
        }
        
        # 写入配置文件
        with open(serial_config_path, 'w') as f:
            yaml.dump(default_config, f, default_flow_style=False)
        
        print(f"创建默认配置文件: {serial_config_path}")
    else:
        print(f"配置文件已存在: {serial_config_path}")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        config_dir = sys.argv[1]
    else:
        # 默认配置目录
        config_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")
    
    ensure_config_exists(config_dir) 