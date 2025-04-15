#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
协议处理器基类
提供协议处理的抽象接口
"""

from __future__ import division, print_function, absolute_import

import os
import yaml
import logging
import sys
from abc import ABCMeta, abstractmethod

# Python 2.7 兼容性处理
PY2 = sys.version_info[0] == 2

# 修改导入语句，使用本地导入
# from protocol_exceptions import ConfigError, PacketError

class ProtocolCommand(object):
    """协议命令基类"""
    
    def __init__(self, command_type, parameters=None):
        """
        初始化命令
        
        Args:
            command_type: 命令类型
            parameters: 命令参数
        """
        self.command_type = command_type
        self.parameters = parameters or {}
    
    def __str__(self):
        # 使用格式化字符串，兼容Python 2.7
        return "{}: {}".format(self.command_type, self.parameters)

class ProtocolResponse(object):
    """协议响应基类"""
    
    def __init__(self, response_type, data=None, success=True, error_msg=None):
        """
        初始化响应
        
        Args:
            response_type: 响应类型
            data: 响应数据
            success: 是否成功
            error_msg: 错误信息
        """
        self.response_type = response_type
        self.data = data or {}
        self.success = success
        self.error_msg = error_msg
    
    def __str__(self):
        # 使用格式化字符串，兼容Python 2.7
        status = "Success" if self.success else "Fail: {}".format(self.error_msg)
        return "{} [{}]: {}".format(self.response_type, status, self.data)

# 纯Python 2.7实现的抽象基类
class ProtocolProcessor(object):
    """协议处理器抽象基类"""
    __metaclass__ = ABCMeta
    
    def __init__(self, config_file=None):
        """
        初始化协议处理器
        
        Args:
            config_file: 配置文件路径，None则使用默认配置文件
        """
        self.config_file = config_file
        self.config = {}
    
    def load_config(self, config_file=None):
        """
        加载配置文件
        
        Args:
            config_file: 配置文件路径，None则使用默认配置文件
            
        Returns:
            dict: 配置字典
            
        Raises:
            ConfigError: 配置错误
        """
        # 使用参数中的配置文件，如果没有则使用初始化时的配置文件
        config_file = config_file or self.config_file
        
        # 如果没有指定配置文件，则使用默认配置
        if not config_file:
            module_dir = os.path.dirname(os.path.abspath(__file__))
            default_config = os.path.join(module_dir, 'config', 'protocol.yaml')
            if os.path.exists(default_config):
                config_file = default_config
        
        # 加载配置文件
        if config_file and os.path.exists(config_file):
            try:
                # 在Python 2.7中不支持encoding参数
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)
                return config
            except Exception as e:
                print("加载配置文件失败: {}".format(str(e)))
        
        # 如果没有配置文件，返回空字典
        return {}
    
    @abstractmethod
    def encode_command(self, command):
        """
        将命令对象编码为字节数据
        
        Args:
            command: 命令对象
            
        Returns:
            bytes: 编码后的字节数据
            
        Raises:
            PacketError: 编码失败时
        """
        pass
    
    @abstractmethod
    def decode_response(self, data):
        """
        将字节数据解码为响应对象
        
        Args:
            data: 字节数据
            
        Returns:
            ProtocolResponse: 解码后的响应对象
            
        Raises:
            PacketError: 解码失败时
        """
        pass
    
    @abstractmethod
    def create_command(self, command_type, **parameters):
        """
        创建命令对象
        
        Args:
            command_type: 命令类型
            **parameters: 命令参数
            
        Returns:
            ProtocolCommand: 命令对象
        """
        pass
    
    @abstractmethod
    def validate_response(self, response, command=None):
        """
        验证响应是否有效
        
        Args:
            response: 响应对象
            command: 对应的命令对象，可选
            
        Returns:
            bool: 是否有效
        """
        pass 