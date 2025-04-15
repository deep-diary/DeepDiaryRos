#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
串口通信包的主管理类
提供串口初始化、读写和事件处理功能
"""

import os
import sys
import time
import threading
import queue
import logging
import serial

from .serial_config import load_config, LOG_LEVELS
from .serial_utils import list_available_ports, setup_logger, str_to_bytes, bytes_to_str
from .serial_exceptions import *

class SerialManager:
    """
    串口管理类
    负责初始化串口连接、读写数据和管理回调函数
    """
    
    def __init__(self, config_file=None, callback=None):
        """
        初始化串口管理器
        
        Args:
            config_file (str): 配置文件路径，None则使用默认配置
            callback (callable): 数据接收回调函数，接收参数为 (data, source)
        """
        # 加载配置
        self.config = load_config(config_file)
        
        # 设置日志
        log_level = LOG_LEVELS.get(self.config['log_level'].lower(), logging.INFO)
        log_file = self.config['log_file'] if self.config['log_to_file'] else None
        self.logger = setup_logger('serial_manager', level=log_level, log_file=log_file)
        
        # 初始化串口和线程变量
        self.serial = None
        self.read_thread = None
        self.is_reading = False
        self.line_buffer = ""
        self.reconnect_count = 0
        
        # 设置回调函数
        self.callback = callback
        
        # 初始化事件回调字典
        self.event_callbacks = {
            'connect': [],      # 连接事件
            'disconnect': [],   # 断开连接事件
            'data': [],         # 数据接收事件
            'error': [],        # 错误事件
            'reconnect': []     # 重连事件
        }
        
        # 如果提供了回调函数，添加到数据事件
        if callback:
            self.on('data', callback)
        
        self.logger.info("Serial manager initialization completed")
        # print("Serial manager initialization completed")

    def list_ports(self):
        """List available ports"""
        ports = list_available_ports()
        if ports:
            self.logger.info("Available ports:")
            for port in ports:
                self.logger.info("  %s" % port)
        else:
            self.logger.warning("No available ports found")
        return ports
    
    def connect(self, port=None):
        """
        连接到串口
        
        Args:
            port (str): 串口设备，None则使用配置中的设备
            
        Returns:
            bool: 是否成功连接
            
        Raises:
            ConnectionError: 连接失败时
        """
        # 如果已连接，先断开
        if self.is_connected():
            self.disconnect()
        
        # 使用传入端口或配置端口
        port = port or self.config['port']
        
        try:
            self.logger.info("Connecting to serial port: %s, baudrate: %s" % (port, self.config['baudrate']))
            
            # Create serial object
            self.serial = serial.Serial(
                port=port,
                baudrate=self.config['baudrate'],
                bytesize=self.config['bytesize'],
                parity=self.config['parity'],
                stopbits=self.config['stopbits'],
                timeout=self.config['timeout'],
                write_timeout=self.config['write_timeout'],
                xonxoff=self.config['xonxoff'],
                rtscts=self.config['rtscts'],
                dsrdtr=self.config['dsrdtr']
            )
            
            # Flush buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # Update port in configuration
            self.config['port'] = port
            
            # Start read thread
            self.start_read_thread()
            
            # Trigger connect event
            self._trigger_event('connect', port)
            
            self.logger.info("Successfully connected to serial port: %s" % port)
            return True
            
        except serial.SerialException as e:
            error_msg = "Failed to connect to serial port %s: %s" % (port, str(e))
            self.logger.error(error_msg)
            
            # Trigger error event
            self._trigger_event('error', {'type': 'connect', 'message': error_msg})
            
            raise ConnectionError(error_msg)
            
        except Exception as e:
            error_msg = "Unknown error occurred during connection: %s" % str(e)
            self.logger.error(error_msg)
            
            # Trigger error event
            self._trigger_event('error', {'type': 'connect', 'message': error_msg})
            
            raise ConnectionError(error_msg)
    
    def disconnect(self):
        """
        断开串口连接
        
        Returns:
            bool: 是否成功断开
        """
        if not self.serial:
            return True
            
        try:
            # 停止读取线程
            self.stop_read_thread()
            
            # 关闭串口
            if self.serial.is_open:
                self.serial.close()
            
            # 重置变量
            self.reconnect_count = 0
            
            # 触发断开连接事件
            self._trigger_event('disconnect', self.config['port'])
            
            self.logger.info("已断开串口连接")
            return True
            
        except Exception as e:
            error_msg = "Error disconnecting: %s" % str(e)
            self.logger.error(error_msg)
            
            # Trigger error event
            self._trigger_event('error', {'type': 'disconnect', 'message': error_msg})
            
            return False
    
    def is_connected(self):
        """
        检查是否已连接
        
        Returns:
            bool: 是否已连接
        """
        return self.serial is not None and self.serial.is_open
    
    def write(self, data, flush=None):
        """
        发送数据到串口
        
        Args:
            data: 要发送的数据，可以是字符串或字节
            flush (bool): 是否刷新缓冲区，None则使用配置
            
        Returns:
            int: 发送的字节数
            
        Raises:
            WriteError: 写入失败时
        """
        if not self.is_connected():
            error_msg = "未连接到串口，无法发送数据"
            self.logger.error(error_msg)
            raise WriteError(error_msg)
        
        try:
            # Convert data to bytes
            byte_data = str_to_bytes(data, self.config['encoding'])
            
            # Write data
            bytes_written = self.serial.write(byte_data)
            
            # Flush buffer
            if flush is None:
                flush = self.config['flush_on_write']
            if flush:
                self.serial.flush()
            
            self.logger.debug("Sent %d bytes" % bytes_written)
            return bytes_written
            
        except serial.SerialTimeoutException:
            error_msg = "Write timeout"
            self.logger.error(error_msg)
            
            # Trigger error event
            self._trigger_event('error', {'type': 'write_timeout', 'message': error_msg})
            
            raise TimeoutError(error_msg)
            
        except serial.SerialException as e:
            error_msg = "Failed to write to serial port: %s" % str(e)
            self.logger.error(error_msg)
            
            # Trigger error event
            self._trigger_event('error', {'type': 'write', 'message': error_msg})
            
            raise WriteError(error_msg)
            
        except Exception as e:
            error_msg = "Unknown error occurred while writing: %s" % str(e)
            self.logger.error(error_msg)
            
            # Trigger error event
            self._trigger_event('error', {'type': 'unknown', 'message': error_msg})
            
            raise WriteError(error_msg)
    
    def write_line(self, data, line_terminator=None):
        """
        发送一行数据，自动添加行结束符
        
        Args:
            data: 要发送的数据，不含行结束符
            line_terminator (str): 行结束符，None则使用配置
            
        Returns:
            int: 发送的字节数
        """
        # Use the line terminator from the configuration if not specified
        if line_terminator is None:
            line_terminator = self.config['line_terminator']
        
        # Combine data and line terminator
        line_data = str(data) + line_terminator
        
        # Send data
        return self.write(line_data)
    
    def flush(self):
        """
        刷新输入和输出缓冲区
        
        Returns:
            bool: 是否成功
        """
        if not self.is_connected():
            return False
        
        try:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            return True
        except Exception as e:
            self.logger.error("Failed to flush buffer: %s" % str(e))
            return False
    
    def on(self, event, callback):
        """
        Registers an event callback function
        
        Args:
            event (str): Event name
            callback (callable): Callback function, receives arguments as (data, source)
            
        Returns:
            bool: Whether registration was successful
        """
        if event not in self.event_callbacks:
            self.logger.warning("Unknown event: %s" % event)
            return False
        
        if not callable(callback):
            self.logger.warning("Callback function is not callable")
            return False
        
        self.event_callbacks[event].append(callback)
        return True
    
    def off(self, event, callback=None):
        """
        移除事件回调函数
        
        Args:
            event (str): 事件名称
            callback (callable): 要移除的回调函数，None则移除所有
            
        Returns:
            bool: 是否成功移除
        """
        if event not in self.event_callbacks:
            return False
        
        if callback is None:
            # 移除所有回调
            self.event_callbacks[event] = []
            return True
        
        # 移除特定回调
        if callback in self.event_callbacks[event]:
            self.event_callbacks[event].remove(callback)
            return True
            
        return False
    
    def start_read_thread(self):
        """
        启动读取线程
        
        Returns:
            bool: 是否成功启动
        """
        if self.is_reading:
            return True
            
        if not self.is_connected():
            self.logger.error("未连接到串口，无法启动读取线程")
            return False
        
        try:
            # 设置标志
            self.is_reading = True
            
            # 创建并启动线程
            self.read_thread = threading.Thread(target=self._read_thread_func)
            self.read_thread.daemon = True
            self.read_thread.start()
            
            self.logger.debug("已启动读取线程")
            return True
            
        except Exception as e:
            self.is_reading = False
            self.logger.error("Failed to start read thread: " + str(e))
            return False
    
    def stop_read_thread(self):
        """
        停止读取线程
        
        Returns:
            bool: 是否成功停止
        """
        # 如果线程未运行，直接返回
        if not self.is_reading or not self.read_thread:
            return True
            
        try:
            # 设置标志
            self.is_reading = False
            
            # 等待线程结束
            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join(1.0)  # 等待1秒
            
            self.read_thread = None
            
            self.logger.debug("已停止读取线程")
            return True
            
        except Exception as e:
            self.logger.error("Failed to stop read thread: " + str(e))
            return False
    
    def _read_thread_func(self):
        """
        读取线程函数
        负责从串口读取数据并触发事件
        """
        self.logger.debug("读取线程开始执行")
        
        read_mode = self.config['read_mode'].lower()
        encoding = self.config['encoding']
        line_term = self.config['line_terminator']
        read_chunk_size = self.config['read_chunk_size']
        thread_sleep = self.config['thread_sleep']
        
        # 清空行缓冲区
        self.line_buffer = ""
        
        while self.is_reading and self.is_connected():
            try:
                if self.serial.in_waiting > 0:
                    if read_mode == 'line':
                        # 按行读取模式
                        self._read_by_line(encoding, line_term)
                    elif read_mode == 'raw':
                        # 原始字节模式
                        self._read_raw(read_chunk_size, encoding)
                    elif read_mode == 'length':
                        # 固定长度模式
                        read_length = self.config['read_length']
                        self._read_by_length(read_length, encoding)
                
                # 小睡一下，减少CPU使用
                time.sleep(thread_sleep)
                
            except serial.SerialException as e:
                error_msg = "读取线程发生串口错误: " + str(e)
                self.logger.error(error_msg)
                
                # 触发错误事件
                self._trigger_event('error', {'type': 'serial', 'message': error_msg})
                
                # 如果配置为自动重连，尝试重连
                if self.config['auto_reconnect']:
                    if self._try_reconnect():
                        continue  # 重连成功，继续循环
                    else:
                        break  # 重连失败，退出循环
                else:
                    break  # 非自动重连，退出循环
                    
            except Exception as e:
                error_msg = "读取线程发生未知错误: " + str(e)
                self.logger.error(error_msg)
                
                # 触发错误事件
                self._trigger_event('error', {'type': 'unknown', 'message': error_msg})
                
                # 短暂暂停以避免错误循环
                time.sleep(0.5)
                
        self.logger.debug("读取线程已退出")
    
    def _read_by_line(self, encoding, line_term):
        """按行读取"""
        try:
            # 使用serial模块的readline函数直接读取一行
            line = self.serial.readline()
            if line:
                # print('line:', line)
                # 如果data 是字符串，则转换为字节数组
                if isinstance(line, str):
                    # print('line is str')
                    line = bytearray(line)
                    # print('line is encoded')    
                # 触发数据事件
                self._trigger_event('data', line)
                
        except UnicodeDecodeError:
            self.logger.warning("Cannot decode byte")
    
    def _read_raw(self, chunk_size, encoding):
        """原始字节读取"""
        # 读取一块数据
        data = self.serial.read(chunk_size)
        
        if data:
            # 触发数据事件，以字节形式
            self._trigger_event('data', data)
            
            # 也尝试解码并触发事件
            try:
                text = data.decode(encoding)
                self._trigger_event('data', text)
            except UnicodeDecodeError:
                pass  # 忽略解码错误
    
    def _read_by_length(self, length, encoding):
        """固定长度读取"""
        # 读取指定长度的数据
        data = self.serial.read(length)
        
        if len(data) == length:
            # 触发数据事件，以字节形式
            self._trigger_event('data', data)
            
            # 也尝试解码并触发事件
            try:
                text = data.decode(encoding)
                self._trigger_event('data', text)
            except UnicodeDecodeError:
                pass  # 忽略解码错误
    
    def _trigger_event(self, event, data):
        """
        触发事件
        
        Args:
            event (str): 事件名称
            data: 事件数据
        """
        if event not in self.event_callbacks:
            return
            
        for callback in self.event_callbacks[event]:
            try:
                callback(data, event)
            except Exception as e:
                self.logger.error("Executing callback function for event %s failed: %s" % (event, str(e)))
    
    def _try_reconnect(self):
        """
        Attempt to reconnect
        
        Returns:
            bool: Whether the reconnect was successful
        """
        if self.reconnect_count >= self.config['reconnect_attempts']:
            self.logger.error("Reconnect attempts (%d) have reached the limit" % self.reconnect_count)
            return False
            
        self.reconnect_count += 1
        reconnect_delay = self.config['reconnect_delay']
        
        self.logger.info("Attempting reconnect (attempt %d), waiting %d seconds..." % (self.reconnect_count, reconnect_delay))
        
        # Trigger reconnect event
        self._trigger_event('reconnect', {
            'attempt': self.reconnect_count,
            'max_attempts': self.config['reconnect_attempts']
        })
        
        # 等待指定时间
        time.sleep(reconnect_delay)
        
        # 尝试关闭串口（如果仍然打开）
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        except:
            pass
        
        # 尝试重新连接
        try:
            self.serial = serial.Serial(
                port=self.config['port'],
                baudrate=self.config['baudrate'],
                bytesize=self.config['bytesize'],
                parity=self.config['parity'],
                stopbits=self.config['stopbits'],
                timeout=self.config['timeout'],
                write_timeout=self.config['write_timeout'],
                xonxoff=self.config['xonxoff'],
                rtscts=self.config['rtscts'],
                dsrdtr=self.config['dsrdtr']
            )
            
            # 刷新缓冲区
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            self.logger.info("Reconnection successful!")
            return True
            
        except Exception as e:
            self.logger.error("Reconnection failed: %s" % str(e))
            return False 