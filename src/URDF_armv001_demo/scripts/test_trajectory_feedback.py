#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import struct
import time
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState

class TrajectoryTester:
    """
    发送测试轨迹命令并监控反馈
    """
    def __init__(self):
        rospy.init_node('trajectory_tester')
        
        # 关节名称
        self.joint_names = ['first_joint', 'second_joint', 'third_joint', 
                           'fourth_joint', 'fifth_joint', 'sixth_joint']
        
        # 创建发布器和订阅器
        self.cmd_pub = rospy.Publisher('/serial_node/data_to_send', UInt8MultiArray, queue_size=10)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # 存储最新的关节状态
        self.latest_joint_state = None
        
        # 等待订阅器连接
        rospy.sleep(1.0)
        
        rospy.loginfo("轨迹测试器已初始化")
    
    def joint_state_callback(self, msg):
        """处理关节状态反馈"""
        # 存储最新的关节状态
        self.latest_joint_state = msg
    
    def create_test_command(self, positions):
        """
        创建测试命令数据包
        @param positions: 关节位置数组
        """
        # 构建数据包
        data = bytearray()
        data.append(0xFF)  # 起始标记
        data.append(0x01)  # 命令类型 - 位置命令
        data.append(len(positions))  # 关节数量
        
        # 添加每个关节的位置和速度数据
        for pos in positions:
            # 位置
            pos_bytes = struct.pack('<f', pos)
            data.extend(pos_bytes)
            # 速度 (固定为0.5)
            vel_bytes = struct.pack('<f', 0.5)
            data.extend(vel_bytes)
        
        # 计算校验和
        checksum = sum(data) & 0xFF
        data.append(checksum)
        
        # 添加结束标记
        data.append(0xFE)
        
        return data
    
    def send_command(self, positions):
        """
        发送命令并等待反馈
        @param positions: 关节位置数组
        """
        # 创建命令数据包
        data = self.create_test_command(positions)
        
        # 创建消息
        msg = UInt8MultiArray()
        msg.layout.dim.append(MultiArrayDimension(
            label="length",
            size=len(data),
            stride=1
        ))
        msg.data = list(data)
        
        # 记录发送前的时间
        before_send = rospy.Time.now()
        
        # 发送命令
        rospy.loginfo("发送命令: 位置=%s", positions)
        self.cmd_pub.publish(msg)
        
        # 等待反馈 (最多2秒)
        timeout = rospy.Duration(2.0)
        rate = rospy.Rate(10)  # 10Hz
        
        initial_state = self.latest_joint_state
        
        while rospy.Time.now() - before_send < timeout:
            # 检查是否收到新的关节状态
            if (self.latest_joint_state is not None and 
                (initial_state is None or 
                 self.latest_joint_state.header.stamp > initial_state.header.stamp)):
                
                # 打印反馈
                rospy.loginfo("收到反馈: 位置=%s", self.latest_joint_state.position)
                
                # 检查反馈是否与命令匹配
                match = True
                for i, (cmd, fb) in enumerate(zip(positions, self.latest_joint_state.position)):
                    if abs(cmd - fb) > 0.01:
                        match = False
                        rospy.logwarn("关节 %d 不匹配: 命令=%.4f, 反馈=%.4f", 
                                     i, cmd, fb)
                
                if match:
                    rospy.loginfo("反馈与命令匹配!")
                else:
                    rospy.logwarn("反馈与命令不匹配!")
                
                return
            
            rate.sleep()
        
        rospy.logwarn("等待反馈超时!")
    
    def run_test_sequence(self):
        """
        运行测试序列
        """
        # 确保已连接
        if self.cmd_pub.get_num_connections() == 0:
            rospy.logwarn("未检测到订阅者，等待连接...")
            rospy.sleep(2.0)
        
        # 测试序列 - 一系列不同的关节位置配置
        test_positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 原位
            [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第一个关节转动
            [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],  # 第二个关节转动
            [0.0, 0.0, 0.5, 0.0, 0.0, 0.0],  # 第三个关节转动
            [0.0, 0.0, 0.0, 0.5, 0.0, 0.0],  # 第四个关节转动
            [0.0, 0.0, 0.0, 0.0, 0.5, 0.0],  # 第五个关节转动
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.5],  # 第六个关节转动
            [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],  # 所有关节转动
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 回到原位
        ]
        
        # 执行测试序列
        for i, positions in enumerate(test_positions):
            rospy.loginfo("\n--- 测试 %d/%d ---", i+1, len(test_positions))
            self.send_command(positions)
            rospy.sleep(2.0)  # 等待2秒进行下一个测试
        
        rospy.loginfo("测试序列完成!")

if __name__ == '__main__':
    try:
        tester = TrajectoryTester()
        tester.run_test_sequence()
    except rospy.ROSInterruptException:
        pass 