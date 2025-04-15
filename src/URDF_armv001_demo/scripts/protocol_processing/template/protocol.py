#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
模板协议实现
可作为新产品协议开发的起点
"""

import os
import struct
import logging
from typing import Dict, Any, List, Optional, Union, Tuple

from ..base import ProtocolProcessor, ProtocolCommand, ProtocolResponse
from ..utils import calculate_checksum, bytes_to_hex_str
from ..exceptions import PacketError, ChecksumError, CommandError, ResponseError

class TemplateCommand(ProtocolCommand):
    """模板命令类"""
    
    def __init__(self, command_type: str, parameters: Dict[str, Any] = None, command_code: int = None):
        """
        初始化模板命令
        
        Args:
            command_type: 命令类型
            parameters: 命令参数
            command_code: 命令代码
        """
        super().__init__(command_type, parameters)
        self.command_code = command_code

class TemplateResponse(ProtocolResponse):
    """模板响应类"""
    
    def __init__(self, response_type: str, data: Dict[str, Any] = None, 
                 success: bool = True, error_msg: str = None, response_code: int = None):
        """
        初始化模板响应
        
        Args:
            response_type: 响应类型
            data: 响应数据
            success: 是否成功
            error_msg: 错误信息
            response_code: 响应代码
        """
        super().__init__(response_type, data, success, error_msg)
        self.response_code = response_code

class TemplateProtocol(ProtocolProcessor):
    """模板协议处理器"""
    
    def __init__(self, config_file: str = None):
        """
        初始化模板协议处理器
        
        Args:
            config_file: 配置文件路径，None则使用默认配置文件
        """
        # 检查是否提供配置文件，如果没有则使用默认配置
        if config_file is None:
            # 使用当前模块目录下的config.yaml
            module_dir = os.path.dirname(os.path.abspath(__file__))
            config_file = os.path.join(module_dir, 'config.yaml')
        
        super().__init__(config_file)
        
        # 获取通信设置
        self.start_byte = self.config.get('communication', {}).get('start_byte', 0xFA)
        self.end_byte = self.config.get('communication', {}).get('end_byte', 0xFB)
        self.escape_byte = self.config.get('communication', {}).get('escape_byte', 0xFC)
        self.max_packet_size = self.config.get('communication', {}).get('max_packet_size', 256)
        
        # 命令和响应代码映射
        self.command_codes = {}
        self.response_codes = {}
        self.cmd_type_to_code = {}
        self.resp_code_to_type = {}
        
        # 初始化命令代码映射
        for cmd_type, cmd_info in self.config.get('commands', {}).items():
            code = cmd_info.get('code')
            if code is not None:
                self.command_codes[cmd_type] = cmd_info
                self.cmd_type_to_code[cmd_type] = code
        
        # 初始化响应代码映射
        for resp_type, resp_info in self.config.get('responses', {}).items():
            code = resp_info.get('code')
            if code is not None:
                self.response_codes[resp_type] = resp_info
                self.resp_code_to_type[code] = resp_type
        
        # 错误码映射
        self.error_codes = self.config.get('error_codes', {})
        
        self.logger.info(f"模板协议处理器初始化完成，支持 {len(self.command_codes)} 种命令")
    
    def create_command(self, command_type: str, **parameters) -> TemplateCommand:
        """
        创建模板命令对象
        
        Args:
            command_type: 命令类型
            **parameters: 命令参数
            
        Returns:
            TemplateCommand: 命令对象
            
        Raises:
            CommandError: 命令类型未知或参数错误
        """
        if command_type not in self.command_codes:
            raise CommandError(f"未知命令类型: {command_type}")
        
        command_code = self.cmd_type_to_code.get(command_type)
        
        # 验证参数
        cmd_info = self.command_codes[command_type]
        expected_params = {param['name']: param for param in cmd_info.get('parameters', [])}
        
        # 检查必要参数是否提供
        for param_name in expected_params:
            if param_name not in parameters:
                self.logger.warning(f"命令 {command_type} 缺少参数: {param_name}")
        
        return TemplateCommand(command_type, parameters, command_code)
    
    def encode_command(self, command: TemplateCommand) -> bytes:
        """
        将命令对象编码为字节数据
        
        Args:
            command: 命令对象
            
        Returns:
            bytes: 编码后的字节数据
            
        Raises:
            PacketError: 编码失败时
        """
        try:
            if not isinstance(command, TemplateCommand):
                raise PacketError("命令对象类型错误")
            
            if command.command_code is None:
                raise PacketError(f"命令 {command.command_type} 没有有效的命令代码")
            
            # 准备命令数据
            payload = bytearray([command.command_code])
            
            # 添加参数数据
            if command.parameters:
                cmd_info = self.command_codes.get(command.command_type, {})
                param_defs = {param['name']: param for param in cmd_info.get('parameters', [])}
                
                for param_name, param_value in command.parameters.items():
                    if param_name in param_defs:
                        param_def = param_defs[param_name]
                        param_type = param_def.get('type', 'uint8')
                        
                        # 根据不同类型编码
                        if param_type == 'uint8':
                            payload.append(int(param_value) & 0xFF)
                        elif param_type == 'uint16':
                            payload.extend(struct.pack('<H', int(param_value)))
                        elif param_type == 'float':
                            payload.extend(struct.pack('<f', float(param_value)))
                        elif param_type == 'bool':
                            payload.append(1 if param_value else 0)
                        elif param_type == 'bytes' and isinstance(param_value, bytes):
                            # 添加数据长度
                            length = len(param_value)
                            payload.append(length)
                            payload.extend(param_value)
                        # 可以添加更多类型的支持
            
            # 计算简单校验和
            checksum = calculate_checksum(payload)
            payload.append(checksum)
            
            # 组装完整数据包
            packet = bytearray([self.start_byte])
            
            # 转义处理
            for byte in payload:
                if byte in [self.start_byte, self.end_byte, self.escape_byte]:
                    packet.append(self.escape_byte)
                packet.append(byte)
            
            packet.append(self.end_byte)
            
            self.logger.debug(f"编码命令: {command.command_type} -> {bytes_to_hex_str(packet)}")
            return bytes(packet)
            
        except Exception as e:
            raise PacketError(f"编码命令失败: {str(e)}")
    
    def decode_response(self, data: bytes) -> TemplateResponse:
        """
        将字节数据解码为响应对象
        
        Args:
            data: 字节数据
            
        Returns:
            TemplateResponse: 解码后的响应对象
            
        Raises:
            PacketError: 解码失败时
        """
        try:
            if not data or len(data) < 4:  # 最小包长度检查
                raise PacketError("数据包太短")
            
            # 检查起始和结束字节
            if data[0] != self.start_byte or data[-1] != self.end_byte:
                raise PacketError("数据包格式错误，缺少起始或结束标志")
            
            # 提取并去除转义字符
            payload = bytearray()
            i = 1  # 跳过起始字节
            while i < len(data) - 1:  # 跳过结束字节
                if data[i] == self.escape_byte:
                    if i + 1 < len(data) - 1:
                        payload.append(data[i+1])
                        i += 2
                    else:
                        raise PacketError("转义字符后缺少数据")
                else:
                    payload.append(data[i])
                    i += 1
            
            if len(payload) < 2:  # 至少包含响应代码和校验和
                raise PacketError("数据包负载太短")
            
            # 提取校验和
            checksum_received = payload[-1]
            payload = payload[:-1]
            
            # 计算校验和并验证
            checksum_calculated = calculate_checksum(payload)
            if checksum_received != checksum_calculated:
                raise ChecksumError(f"校验和失败: 计算值={checksum_calculated}, 接收值={checksum_received}")
            
            # 提取响应代码
            response_code = payload[0]
            payload = payload[1:]
            
            # 确定响应类型
            response_type = self.resp_code_to_type.get(response_code)
            if not response_type:
                raise ResponseError(f"未知响应代码: 0x{response_code:02X}")
            
            # 解析响应数据
            data_dict = {}
            
            # 特殊处理通用响应
            if response_type == 'ack':
                if len(payload) >= 2:
                    command_code = payload[0]
                    status_code = payload[1]
                    
                    data_dict['command_code'] = command_code
                    data_dict['status'] = status_code
                    
                    # 检查状态码
                    success = (status_code == 0)
                    error_msg = self.error_codes.get(status_code, f"未知错误: 0x{status_code:02X}")
                    
                    return TemplateResponse(response_type, data_dict, success, 
                                          None if success else error_msg, response_code)
            
            # 解析其他类型响应
            resp_info = self.response_codes.get(response_type, {})
            param_defs = resp_info.get('parameters', [])
            
            offset = 0
            for param_def in param_defs:
                param_name = param_def.get('name')
                param_type = param_def.get('type', 'uint8')
                
                if offset >= len(payload):
                    self.logger.warning(f"响应数据不足，无法解析参数 {param_name}")
                    break
                
                # 根据不同类型解码
                if param_type == 'uint8':
                    data_dict[param_name] = payload[offset]
                    offset += 1
                elif param_type == 'uint16':
                    if offset + 2 <= len(payload):
                        data_dict[param_name] = struct.unpack('<H', payload[offset:offset+2])[0]
                        offset += 2
                elif param_type == 'float':
                    if offset + 4 <= len(payload):
                        data_dict[param_name] = struct.unpack('<f', payload[offset:offset+4])[0]
                        offset += 4
                elif param_type == 'bool':
                    data_dict[param_name] = bool(payload[offset])
                    offset += 1
                elif param_type == 'bytes':
                    if offset < len(payload):
                        length = payload[offset]
                        offset += 1
                        if offset + length <= len(payload):
                            data_dict[param_name] = bytes(payload[offset:offset+length])
                            offset += length
                # 可以添加更多类型的支持
            
            self.logger.debug(f"解码响应: {response_type} -> {data_dict}")
            return TemplateResponse(response_type, data_dict, True, None, response_code)
            
        except ChecksumError as e:
            # 重新包装校验和错误，保持原始异常信息
            raise ChecksumError(str(e))
            
        except Exception as e:
            if isinstance(e, PacketError) or isinstance(e, ResponseError):
                raise
            raise PacketError(f"解码响应失败: {str(e)}")
    
    def validate_response(self, response: TemplateResponse, command: TemplateCommand = None) -> bool:
        """
        验证响应是否有效
        
        Args:
            response: 响应对象
            command: 对应的命令对象，可选
            
        Returns:
            bool: 是否有效
        """
        if not isinstance(response, TemplateResponse):
            return False
        
        # 检查响应是否成功
        if not response.success:
            self.logger.warning(f"响应不成功: {response.error_msg}")
            return False
        
        # 如果提供了命令，检查响应是否与命令匹配
        if command:
            # 特殊处理通用响应
            if response.response_type == 'ack':
                if 'command_code' in response.data:
                    if response.data['command_code'] != command.command_code:
                        self.logger.warning(f"响应命令代码不匹配: 期望={command.command_code}, "
                                           f"实际={response.data['command_code']}")
                        return False
        
        return True 