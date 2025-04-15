#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
串口通信包演示程序
展示基本功能和用法
"""

import sys
import os
import time
import threading
import signal

# 添加父目录到搜索路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

# 导入串口通信包
from serial_comm import SerialManager, list_available_ports, ConfigError, ConnectionError

class SerialDemo:
    """串口通信包演示类"""
    
    def __init__(self):
        """初始化演示"""
        self.serial_manager = None
        self.stop_event = threading.Event()
        
        # 注册信号处理函数
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """处理Ctrl+C信号"""
        print("\n收到中断信号，正在退出...")
        self.stop_event.set()
    
    def data_callback(self, data, source):
        """
        数据接收回调函数
        
        Args:
            data: 接收到的数据
            source: 事件源
        """
        if source == 'data':
            # 去除行尾的换行符
            if isinstance(data, str) and data.endswith('\n'):
                data = data.rstrip('\n')
                
            print(f"收到数据: {data}")
    
    def connect_callback(self, data, source):
        """连接事件回调"""
        print(f"已连接到串口: {data}")
    
    def disconnect_callback(self, data, source):
        """断开连接事件回调"""
        print(f"已断开与串口的连接: {data}")
    
    def error_callback(self, data, source):
        """错误事件回调"""
        print(f"发生错误: {data['message']}")
    
    def list_ports(self):
        """列出所有可用串口"""
        print("\n=== 可用串口列表 ===")
        ports = list_available_ports()
        
        if not ports:
            print("未找到可用串口设备")
            return None
        
        for i, port in enumerate(ports):
            print(f"{i+1}. {port['port']} - {port['desc']} [{port['hwid']}]")
        
        return ports
    
    def select_port(self):
        """选择串口"""
        ports = self.list_ports()
        
        if not ports:
            return None
        
        while True:
            try:
                choice = input("\n请选择串口 (输入序号或直接输入串口名，q退出): ")
                
                if choice.lower() == 'q':
                    return None
                
                # 尝试作为索引解析
                try:
                    idx = int(choice) - 1
                    if 0 <= idx < len(ports):
                        return ports[idx]['port']
                    else:
                        print("无效的选择，请重试")
                except ValueError:
                    # 如果不是数字，视为直接输入的串口名
                    if choice in [p['port'] for p in ports]:
                        return choice
                    else:
                        print("无效的串口名，请重试")
                
            except KeyboardInterrupt:
                return None
    
    def init_serial(self, port=None, config_file=None):
        """
        初始化串口管理器
        
        Args:
            port (str): 串口设备，None则交互选择
            config_file (str): 配置文件路径
        
        Returns:
            bool: 是否成功初始化
        """
        try:
            # 如果未指定端口，交互选择
            if port is None:
                port = self.select_port()
                if port is None:
                    return False
            
            # 创建串口管理器
            self.serial_manager = SerialManager(config_file=config_file, callback=self.data_callback)
            
            # 注册其他事件回调
            self.serial_manager.on('connect', self.connect_callback)
            self.serial_manager.on('disconnect', self.disconnect_callback)
            self.serial_manager.on('error', self.error_callback)
            
            # 连接到串口
            self.serial_manager.connect(port)
            
            return True
            
        except ConfigError as e:
            print(f"配置错误: {e}")
            return False
            
        except ConnectionError as e:
            print(f"连接错误: {e}")
            return False
            
        except Exception as e:
            print(f"初始化串口时出错: {e}")
            return False
    
    def send_command(self, command=None):
        """
        发送命令
        
        Args:
            command (str): 要发送的命令，None则交互输入
            
        Returns:
            bool: 是否成功发送
        """
        if not self.serial_manager or not self.serial_manager.is_connected():
            print("未连接到串口，无法发送命令")
            return False
            
        try:
            # 如果未指定命令，交互输入
            if command is None:
                command = input("请输入要发送的命令 (q退出): ")
                
                if command.lower() == 'q':
                    return False
            
            # 发送命令
            print(f"发送: {command}")
            self.serial_manager.write_line(command)
            return True
            
        except Exception as e:
            print(f"发送命令时出错: {e}")
            return False
    
    def run_interactive(self):
        """运行交互式演示"""
        print("=== 串口通信包演示程序 ===")
        print("此演示展示串口通信包的基本功能和用法")
        
        # 初始化串口
        if not self.init_serial():
            print("初始化串口失败，退出演示")
            return
        
        print("\n=== 命令模式 ===")
        print("输入命令并按回车发送，输入q退出")
        
        # 命令循环
        while not self.stop_event.is_set():
            try:
                if not self.send_command():
                    break
                
                # 短暂等待接收响应
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                break
        
        # 断开连接
        if self.serial_manager:
            self.serial_manager.disconnect()
    
    def run_test(self, port=None, config_file=None, commands=None):
        """
        运行自动测试
        
        Args:
            port (str): 串口设备
            config_file (str): 配置文件路径
            commands (list): 要发送的命令列表
        """
        print("=== 串口通信包自动测试 ===")
        
        # 使用默认测试命令
        if commands is None:
            commands = [
                "AT",
                "AT+VERSION",
                "AT+HELP"
            ]
        
        # 初始化串口
        if not self.init_serial(port, config_file):
            print("初始化串口失败，退出测试")
            return
        
        # 发送命令
        print("\n=== 发送测试命令 ===")
        for cmd in commands:
            print(f"\n> 测试命令: {cmd}")
            self.send_command(cmd)
            time.sleep(1)  # 等待响应
        
        print("\n=== 测试完成 ===")
        
        # 断开连接
        if self.serial_manager:
            self.serial_manager.disconnect()

if __name__ == "__main__":
    demo = SerialDemo()
    
    # 解析命令行参数
    if len(sys.argv) > 1:
        if sys.argv[1] == "test":
            port = sys.argv[2] if len(sys.argv) > 2 else None
            config_file = sys.argv[3] if len(sys.argv) > 3 else None
            demo.run_test(port, config_file)
        else:
            demo.run_interactive()
    else:
        demo.run_interactive() 