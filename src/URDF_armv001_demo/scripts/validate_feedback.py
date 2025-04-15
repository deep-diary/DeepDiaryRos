#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import struct
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState

class FeedbackValidator:
    """
    验证串口命令与关节反馈之间的一致性
    """
    def __init__(self):
        rospy.init_node('feedback_validator')
        
        # 存储最新数据
        self.latest_command = None
        self.latest_feedback = None
        self.latest_joint_state = None
        
        # 创建订阅器
        self.cmd_sub = rospy.Subscriber('/serial_node/data_to_send', 
                                      UInt8MultiArray, self.command_callback)
        self.fb_sub = rospy.Subscriber('/serial_node/data_received', 
                                     UInt8MultiArray, self.feedback_callback)
        self.joint_sub = rospy.Subscriber('/joint_states', 
                                        JointState, self.joint_state_callback)
        
        # 定时器 - 每秒验证一次
        self.timer = rospy.Timer(rospy.Duration(1.0), self.validate_data)
        
        rospy.loginfo("反馈验证器已初始化")
    
    def command_callback(self, msg):
        """处理发送的命令"""
        self.latest_command = msg
    
    def feedback_callback(self, msg):
        """处理接收的反馈"""
        self.latest_feedback = msg
    
    def joint_state_callback(self, msg):
        """处理关节状态"""
        self.latest_joint_state = msg
    
    def parse_command(self, data):
        """解析命令数据"""
        result = {"type": "unknown", "positions": []}
        
        try:
            if len(data) >= 4 and data[0] == 0xFF and data[-1] == 0xFE:
                result["type"] = "command"
                cmd_type = data[1]
                joint_count = data[2]
                
                if cmd_type == 0x01:  # 位置命令
                    positions = []
                    for i in range(joint_count):
                        base_idx = 3 + i * 8
                        if base_idx + 4 <= len(data) - 2:
                            pos = struct.unpack('<f', bytes(data[base_idx:base_idx+4]))[0]
                            positions.append(pos)
                    
                    result["positions"] = positions
        except Exception as e:
            rospy.logwarn("解析命令出错: %s", e)
            
        return result
    
    def parse_feedback(self, data):
        """解析反馈数据"""
        result = {"type": "unknown", "positions": []}
        
        try:
            if len(data) >= 4 and data[0] == 0xFF and data[-1] == 0xFE:
                result["type"] = "feedback"
                cmd_type = data[1]
                joint_count = data[2]
                
                if cmd_type == 0x02:  # 状态反馈
                    positions = []
                    for i in range(joint_count):
                        base_idx = 3 + i * 12
                        if base_idx + 4 <= len(data) - 2:
                            pos = struct.unpack('<f', bytes(data[base_idx:base_idx+4]))[0]
                            positions.append(pos)
                    
                    result["positions"] = positions
        except Exception as e:
            rospy.logwarn("解析反馈出错: %s", e)
            
        return result
    
    def validate_data(self, event):
        """验证命令和反馈的一致性"""
        if not all([self.latest_command, self.latest_joint_state]):
            return
            
        cmd_data = self.latest_command.data
        joint_pos = self.latest_joint_state.position
        
        # 解析命令
        cmd_info = self.parse_command(cmd_data)
        
        # 如果有反馈数据，也解析它
        fb_info = {"positions": []}
        if self.latest_feedback:
            fb_info = self.parse_feedback(self.latest_feedback.data)
        
        # 输出比较信息
        rospy.loginfo("\n--- 数据一致性验证 ---")
        
        if cmd_info["positions"]:
            rospy.loginfo("命令位置: %s", cmd_info["positions"])
            
        if fb_info["positions"]:
            rospy.loginfo("反馈位置: %s", fb_info["positions"])
            
        if joint_pos:
            rospy.loginfo("关节状态: %s", list(joint_pos))
        
        # 比较命令和关节状态
        if cmd_info["positions"] and len(cmd_info["positions"]) == len(joint_pos):
            match = True
            for i, (cmd, js) in enumerate(zip(cmd_info["positions"], joint_pos)):
                if abs(cmd - js) > 0.01:
                    match = False
                    rospy.logwarn("关节 %d 不匹配: 命令=%.4f, 关节状态=%.4f", 
                                 i, cmd, js)
            
            if match:
                rospy.loginfo("命令与关节状态匹配!")
            else:
                rospy.logwarn("命令与关节状态不匹配!")
                
        # 比较反馈和关节状态
        if fb_info["positions"] and len(fb_info["positions"]) == len(joint_pos):
            match = True
            for i, (fb, js) in enumerate(zip(fb_info["positions"], joint_pos)):
                if abs(fb - js) > 0.01:
                    match = False
                    rospy.logwarn("关节 %d 不匹配: 反馈=%.4f, 关节状态=%.4f", 
                                 i, fb, js)
            
            if match:
                rospy.loginfo("反馈与关节状态匹配!")
            else:
                rospy.logwarn("反馈与关节状态不匹配!")

if __name__ == '__main__':
    try:
        validator = FeedbackValidator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 