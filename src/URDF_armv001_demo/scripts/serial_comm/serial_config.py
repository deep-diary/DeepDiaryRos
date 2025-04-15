#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
串口通信包的配置模块
负责加载和验证配置
"""

import os
import yaml
import logging
from .serial_exceptions import ConfigError

# 默认配置
DEFAULT_CONFIG = {
    # 串口设置
    'port': '/dev/ttyUSB0',
    'baudrate': 921600,
    'bytesize': 8,
    'parity': 'N',
    'stopbits': 1,
    'timeout': 0.1,
    'write_timeout': 1.0,
    'xonxoff': False,
    'rtscts': False,
    'dsrdtr': False,
    
    # 读取设置
    'read_mode': 'line',
    'line_terminator': '\n',
    'read_length': 1024,
    
    # 处理设置
    'encoding': 'utf-8',
    'auto_reconnect': True,
    'reconnect_delay': 2.0,
    'reconnect_attempts': 5,
    
    # 高级设置
    'buffer_size': 4096,
    'read_chunk_size': 128,
    'flush_on_write': True,
    'thread_sleep': 0.01,
    'line_buffer_size': 100,
    
    # 日志设置
    'log_level': 'info',
    'log_to_file': False,
    'log_file': 'serial_comm.log'
}

# 日志级别映射
LOG_LEVELS = {
    'debug': logging.DEBUG,
    'info': logging.INFO, 
    'warning': logging.WARNING,
    'error': logging.ERROR,
    'critical': logging.CRITICAL
}

def load_config(config_file=None):
    """
    加载配置文件
    
    Args:
        config_file (str): 配置文件路径，None则使用默认配置
        
    Returns:
        dict: 配置字典
    
    Raises:
        ConfigError: 当配置加载或验证失败时
    """
    # 使用默认配置作为基础
    config = DEFAULT_CONFIG.copy()
    
    # If a configuration file is specified, attempt to load it
    if config_file:
        try:
            if not os.path.exists(config_file):
                raise ConfigError("Configuration file not found: %s" % config_file)
                
            with open(config_file, 'r') as f:
                yaml_config = yaml.safe_load(f)
                
            if yaml_config:
                # Update the configuration
                config.update(yaml_config)
        except Exception as e:
            raise ConfigError("Failed to load configuration file: %s" % str(e))
    
    # 验证配置
    validate_config(config)
    
    return config

def validate_config(config):
    """
    验证配置项
    
    Args:
        config (dict): 配置字典
        
    Raises:
        ConfigError: 当配置验证失败时
    """
    try:
        # Check required fields
        required_fields = ['port', 'baudrate', 'bytesize', 'parity', 'stopbits']
        for field in required_fields:
            if field not in config:
                raise ConfigError("Missing required configuration item: %s" % field)
        
        # Validate baudrate is an integer and within a reasonable range
        if not isinstance(config['baudrate'], int) or config['baudrate'] <= 0:
            raise ConfigError("Baudrate must be a positive integer, current value: %s" % config['baudrate'])
        
        # Validate data bits
        if config['bytesize'] not in [5, 6, 7, 8]:
            raise ConfigError("Data bits must be 5, 6, 7, or 8, current value: %s" % config['bytesize'])
        
        # Validate parity
        if config['parity'] not in ['N', 'E', 'O', 'M', 'S']:
            raise ConfigError("Parity must be N, E, O, M, or S, current value: %s" % config['parity'])
        
        # Validate stop bits
        if config['stopbits'] not in [1, 1.5, 2]:
            raise ConfigError("Stop bits must be 1, 1.5, or 2, current value: %s" % config['stopbits'])
        
        # Validate read mode
        if config['read_mode'] not in ['line', 'raw', 'length']:
            raise ConfigError("Read mode must be line, raw, or length, current value: %s" % config['read_mode'])
        
        # Validate log level
        if config['log_level'].lower() not in LOG_LEVELS:
            raise ConfigError("Invalid log level, current value: %s" % config['log_level'])
        
    except Exception as e:
        if isinstance(e, ConfigError):
            raise
        raise ConfigError("Configuration validation failed: %s" % str(e))
    
    return True 