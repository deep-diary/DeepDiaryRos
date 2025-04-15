#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import yaml
import os
from threading import Lock
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension

class SerialNode:
    """纯串口通信节点"""
    
    def __init__(self):
        rospy.init_node('serial_node')
        
        # 加载配置
        try:
            config_file = rospy.get_param("~config_file")
        except KeyError:
            default_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                      "config", "serial_config.yaml")
            rospy.logwarn("No config_file parameter found, using default: %s", default_path)
            config_file = default_path
            
        self.load_config(config_file)
        
        # 串口锁
        self.serial_lock = Lock()
        
        # 初始化发布者和订阅者
        self.serial_pub = rospy.Publisher('~data_received', UInt8MultiArray, queue_size=10)
        self.serial_sub = rospy.Subscriber('~data_to_send', UInt8MultiArray, self.send_data_callback)
        
        # 初始化串口
        try:
            self.serial_port = serial.Serial(
                port=self.config['port'],
                baudrate=self.config['baudrate'],
                timeout=self.config['timeout']
            )
            rospy.loginfo("Serial port %s opened successfully", self.config['port'])
        except serial.SerialException as e:
            rospy.logerr("Failed to open serial port: %s", e)
            return

        # 启动串口读取线程
        self.timer = rospy.Timer(rospy.Duration(1.0/self.config['update_rate']), self.read_serial)
        
        rospy.loginfo("Serial node initialized successfully")
        
    def load_config(self, config_file):
        """加载配置文件"""
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)['serial']
            rospy.loginfo("Loaded configuration from %s", config_file)
        except Exception as e:
            rospy.logerr("Failed to load config file: %s", e)
            # 使用默认配置
            self.config = {
                'port': '/dev/ttyUSB1', 
                'baudrate': 115200, 
                'timeout': 0.1,
                'update_rate': 50
            }
            rospy.logwarn("Using default configuration")
    
    def send_data_callback(self, msg):
        """接收要发送的数据并写入串口"""
        try:
            with self.serial_lock:
                data = bytearray(msg.data)
                self.serial_port.write(data)
                rospy.logdebug("Sent %d bytes to serial port", len(data))
        except Exception as e:
            rospy.logerr("Failed to send data to serial port: %s", e)
    
    def read_serial(self, event):
        """读取串口数据并发布"""
        if not hasattr(self, 'serial_port') or not self.serial_port.is_open:
            return
            
        with self.serial_lock:
            if self.serial_port.in_waiting:
                try:
                    # 读取一行数据
                    data = self.serial_port.readline()
                    if data:
                        # 创建ROS消息并发布
                        msg = UInt8MultiArray()
                        msg.layout.dim.append(MultiArrayDimension(
                            label="length",
                            size=len(data),
                            stride=1
                        ))
                        msg.data = [ord(b) if isinstance(b, str) else b for b in data]
                        self.serial_pub.publish(msg)
                        rospy.logdebug("Published %d bytes from serial", len(data))
                except Exception as e:
                    rospy.logerr("Failed to read from serial port: %s", e)
    
    def shutdown(self):
        """关闭串口"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            rospy.loginfo("Serial port closed")

if __name__ == '__main__':
    node = SerialNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin() 