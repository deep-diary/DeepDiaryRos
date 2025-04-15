#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import struct
from std_msgs.msg import UInt8MultiArray

class SerialMonitor:
    """监控并可视化串口通信数据"""
    
    def __init__(self):
        rospy.init_node('serial_monitor')
        
        # 订阅串口数据话题
        self.send_sub = rospy.Subscriber('/serial_node/data_to_send', UInt8MultiArray, 
                                        self.send_callback)
        self.recv_sub = rospy.Subscriber('/serial_node/data_received', UInt8MultiArray, 
                                        self.recv_callback)
        
        rospy.loginfo("串口监视器已启动")
        
    def send_callback(self, msg):
        """处理发送的数据"""
        self.process_data(msg.data, "发送")
        
    def recv_callback(self, msg):
        """处理接收的数据"""
        self.process_data(msg.data, "接收")
    
    def process_data(self, data, direction):
        """解析并显示数据包"""
        try:
            data_bytes = bytearray(data)
            
            if len(data_bytes) < 3:
                rospy.logwarn("%s的数据包过短: %s", direction, self.format_bytes(data_bytes))
                return
                
            if data_bytes[0] != 0xFF or data_bytes[-1] != 0xFE:
                rospy.logwarn("%s的数据包标记无效: %s", direction, self.format_bytes(data_bytes))
                return
                
            # 获取命令类型和关节数量
            cmd_type = data_bytes[1]
            joint_count = data_bytes[2]
            
            # 显示头部信息
            rospy.loginfo("--- %s数据包 ---", direction)
            rospy.loginfo("命令类型: 0x%02X", cmd_type)
            rospy.loginfo("关节数量: %d", joint_count)
            
            # 根据命令类型处理数据
            if cmd_type == 0x01:  # 位置命令
                self.parse_position_command(data_bytes, joint_count)
            elif cmd_type == 0x02:  # 状态反馈
                self.parse_status_feedback(data_bytes, joint_count)
            else:
                rospy.loginfo("未知命令类型, 原始数据: %s", self.format_bytes(data_bytes))
        
        except Exception as e:
            rospy.logwarn("解析%s数据出错: %s", direction, e)
    
    def parse_position_command(self, data, joint_count):
        """解析位置命令"""
        try:
            for i in range(joint_count):
                base_idx = 3 + i * 8  # 跳过头部，每个关节8字节
                
                if base_idx + 8 > len(data) - 2:
                    rospy.logwarn("数据长度不足以解析关节 %d", i)
                    break
                    
                pos = struct.unpack('<f', data[base_idx:base_idx+4])[0]
                vel = struct.unpack('<f', data[base_idx+4:base_idx+8])[0]
                
                rospy.loginfo("关节 %d: 位置=%.4f, 速度=%.4f", i, pos, vel)
                
            checksum = data[-2]
            calc_checksum = sum(data[:-2]) & 0xFF
            
            if checksum == calc_checksum:
                rospy.loginfo("校验和: 0x%02X (有效)", checksum)
            else:
                rospy.logwarn("校验和: 0x%02X (应为 0x%02X)", checksum, calc_checksum)
                
        except Exception as e:
            rospy.logwarn("解析位置命令出错: %s", e)
    
    def parse_status_feedback(self, data, joint_count):
        """解析状态反馈"""
        try:
            for i in range(joint_count):
                base_idx = 3 + i * 12  # 跳过头部，每个关节12字节
                
                if base_idx + 12 > len(data) - 2:
                    rospy.logwarn("数据长度不足以解析关节 %d", i)
                    break
                    
                pos = struct.unpack('<f', data[base_idx:base_idx+4])[0]
                vel = struct.unpack('<f', data[base_idx+4:base_idx+8])[0]
                acc = struct.unpack('<f', data[base_idx+8:base_idx+12])[0]
                
                rospy.loginfo("关节 %d: 位置=%.4f, 速度=%.4f, 加速度=%.4f", i, pos, vel, acc)
                
            checksum = data[-2]
            calc_checksum = sum(data[:-2]) & 0xFF
            
            if checksum == calc_checksum:
                rospy.loginfo("校验和: 0x%02X (有效)", checksum)
            else:
                rospy.logwarn("校验和: 0x%02X (应为 0x%02X)", checksum, calc_checksum)
                
        except Exception as e:
            rospy.logwarn("解析状态反馈出错: %s", e)
    
    def format_bytes(self, data):
        """格式化字节数组"""
        return ' '.join(['0x%02X' % b for b in data])

if __name__ == '__main__':
    try:
        monitor = SerialMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 