#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import struct
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
from control_msgs.msg import FollowJointTrajectoryActionGoal
import yaml
import os

class FakeFeedbackPublisher:
    """
    发布模拟的关节状态数据，用于测试轨迹发送
    """
    def __init__(self):
        rospy.init_node('fake_feedback_publisher')
        
        # 加载配置
        self.load_config()
        
        # 标准关节顺序(1-6)
        self.standard_joint_names = ['first_joint', 'second_joint', 'third_joint', 
                                   'fourth_joint', 'fifth_joint', 'sixth_joint']
        
        # 实际关节顺序（将从轨迹消息中获取并更新）
        self.trajectory_joint_names = None
        
        # 使用标准顺序初始化
        self.joint_names = self.standard_joint_names[:]
        self.current_positions = [0.0] * len(self.joint_names)
        
        # 关节状态发布器
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # 模拟串口数据发布器
        self.serial_pub = rospy.Publisher('/serial_node/data_received', UInt8MultiArray, queue_size=10)
        
        # 订阅发送到串口的命令，用于模拟反馈
        self.serial_sub = rospy.Subscriber('/serial_node/data_to_send', UInt8MultiArray, self.command_callback)
        
        # 订阅轨迹目标，获取关节顺序
        self.trajectory_sub = rospy.Subscriber('/URDF_armv001/arm_joint_controller/follow_joint_trajectory/goal', 
                                             FollowJointTrajectoryActionGoal,
                                             self.trajectory_callback)
        
        # 立即发布初始关节状态
        self.publish_joint_states(None)
        
        # 定时器 - 增加频率至25Hz
        self.timer = rospy.Timer(rospy.Duration(0.04), self.publish_joint_states)
        
        rospy.loginfo("Fake feedback publisher initialized with standard joints: %s", self.joint_names)
        rospy.loginfo("Publishing initial state: %s", self.current_positions)
    
    def load_config(self):
        """加载配置文件"""
        try:
            # 尝试获取配置文件路径
            config_file = rospy.get_param("~config_file", None)
            
            if config_file is None:
                # 如果未指定，使用默认路径
                config_file = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                         "config", "arm_controller.yaml")
            
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
            rospy.loginfo("已加载配置文件: %s", config_file)
            
            # 获取关节ID映射
            if 'arm' in self.config and 'joint_names' in self.config['arm'] and 'joint_ids' in self.config['arm']:
                self.joint_names_config = self.config['arm']['joint_names']
                self.joint_ids_config = self.config['arm']['joint_ids']
                self.joint_id_to_name = dict(zip(self.joint_ids_config, self.joint_names_config))
                rospy.loginfo("已加载关节ID映射: %s", self.joint_id_to_name)
            else:
                rospy.logwarn("配置中未找到关节ID映射，将使用默认值")
                self.joint_id_to_name = {1: 'first_joint', 2: 'second_joint', 3: 'third_joint',
                                       4: 'fourth_joint', 5: 'fifth_joint', 6: 'sixth_joint'}
                
        except Exception as e:
            rospy.logerr("加载配置文件失败: %s", e)
            # 使用默认映射
            self.joint_id_to_name = {1: 'first_joint', 2: 'second_joint', 3: 'third_joint',
                                   4: 'fourth_joint', 5: 'fifth_joint', 6: 'sixth_joint'}
            self.config = {
                'protocol': {'start_marker': 0xFF, 'end_marker': 0xFE, 'cmd_type': 0x01}
            }
    
    def trajectory_callback(self, msg):
        """处理轨迹目标消息，获取关节顺序"""
        try:
            # 更新轨迹关节顺序
            if msg.goal.trajectory.joint_names:
                self.trajectory_joint_names = msg.goal.trajectory.joint_names
                rospy.loginfo("更新轨迹关节顺序: %s", self.trajectory_joint_names)
                
            # 如果有轨迹点，处理第一个点
            if len(msg.goal.trajectory.points) > 0:
                traj_positions = msg.goal.trajectory.points[0].positions
                
                # 将轨迹位置映射到标准顺序
                standard_positions = self.map_trajectory_to_standard(
                    self.trajectory_joint_names, list(traj_positions))
                
                rospy.loginfo("收到轨迹目标位置: %s", traj_positions)
                rospy.loginfo("映射到标准顺序: %s", standard_positions)
                
                # 更新位置
                self.current_positions = standard_positions
                
                # 发布更新
                self.publish_joint_states(None)
        except Exception as e:
            rospy.logwarn("处理轨迹目标时出错: %s", e)
    
    def map_trajectory_to_standard(self, traj_joint_names, traj_positions):
        """将轨迹中的关节位置映射到标准顺序"""
        if not traj_joint_names or len(traj_joint_names) != len(traj_positions):
            rospy.logwarn("无法映射关节位置: 名称或位置不匹配")
            return traj_positions
            
        # 创建与标准顺序对应的位置数组
        standard_positions = [0.0] * len(self.standard_joint_names)
        
        # 映射每个位置
        for i, joint_name in enumerate(traj_joint_names):
            if joint_name in self.standard_joint_names:
                standard_idx = self.standard_joint_names.index(joint_name)
                standard_positions[standard_idx] = traj_positions[i]
            else:
                rospy.logwarn("未知关节名: %s", joint_name)
                
        return standard_positions
    
    def map_joint_id_to_standard(self, joint_id, position):
        """将关节ID和位置映射到标准顺序"""
        # 从ID查找关节名称
        joint_name = self.joint_id_to_name.get(joint_id)
        if joint_name and joint_name in self.standard_joint_names:
            standard_idx = self.standard_joint_names.index(joint_name)
            return standard_idx, position
        else:
            rospy.logwarn("未知关节ID: %d", joint_id)
            return None, position
        
    def command_callback(self, msg):
        """
        解析发送给机械臂的命令，并使用这些值更新当前位置
        """
        try:
            data = bytearray(msg.data)
            
            # 简化调试输出 - Python 2.7中bytearray元素已经是整数
            debug_str = ' '.join(['%02X' % b for b in data])
            rospy.loginfo("收到命令数据: [%s]", debug_str)
            
            # 检查数据格式是否合法
            if len(data) < 4 or data[0] != 0xFF or data[-1] != 0xFE:
                rospy.logwarn("数据格式不合法: [%s]", debug_str)
                return
                
            # 获取关节数量和命令类型
            cmd_type = data[1]
            joint_count = data[2]
            
            rospy.loginfo("命令类型: 0x%02X, 关节数量: %d", cmd_type, joint_count)
            
            # 新格式：每个关节数据为(ID + 位置 + 速度)
            # 解析关节位置和ID
            parsed_joints = []  # 存储(joint_id, position)元组
            
            for i in range(joint_count):
                # 每个关节的数据现在是9字节: ID(1) + 位置(4) + 速度(4)
                base_idx = 3 + i * 9
                
                # 确保索引有效
                if base_idx + 8 <= len(data) - 2:  # 减去校验和和结束标记的2字节
                    try:
                        # 获取关节ID
                        joint_id = data[base_idx]
                        
                        # 获取位置数据
                        pos_bytes_str = ''.join([chr(b) for b in data[base_idx+1:base_idx+5]])
                        pos = struct.unpack('<f', pos_bytes_str)[0]
                        
                        parsed_joints.append((joint_id, pos))
                        rospy.loginfo("关节ID: %d, 位置: %.4f", joint_id, pos)
                    except Exception as e:
                        rospy.logwarn("解析关节数据时出错: %s", e)
            
            # 根据关节ID更新位置
            if parsed_joints:
                # 创建新的位置数组
                updated_positions = self.current_positions[:]
                
                # 更新每个关节的位置
                for joint_id, position in parsed_joints:
                    standard_idx, mapped_position = self.map_joint_id_to_standard(joint_id, position)
                    if standard_idx is not None:
                        updated_positions[standard_idx] = mapped_position
                
                rospy.loginfo("更新后的位置: %s", updated_positions)
                
                # 更新当前位置
                self.current_positions = updated_positions
                
                # 立即发布更新的关节状态
                self.publish_joint_states(None)
                
                # 发布模拟的串口反馈数据
                self.publish_serial_feedback()
            else:
                rospy.logwarn("未能解析任何关节数据")
                
        except Exception as e:
            rospy.logwarn("解析命令数据出错: %s", e)
            import traceback
            rospy.logwarn(traceback.format_exc())
    
    def publish_joint_states(self, event):
        """
        定时发布关节状态
        """
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_state_pub.publish(msg)
    
    def publish_serial_feedback(self):
        """
        发布模拟的串口反馈数据 - 使用相同的协议格式
        """
        try:
            # 获取协议参数
            START_MARKER = self.config.get('protocol', {}).get('start_marker', 0xFF)
            CMD_TYPE_FEEDBACK = 0x02  # 反馈命令类型
            END_MARKER = self.config.get('protocol', {}).get('end_marker', 0xFE)
            
            # 构建反馈数据包
            data = bytearray()
            data.append(START_MARKER)  # 起始标记
            data.append(CMD_TYPE_FEEDBACK)  # 命令类型 - 状态反馈
            data.append(len(self.joint_names))  # 关节数量
            
            # 添加每个关节的ID、位置、速度和加速度
            for i, joint_name in enumerate(self.standard_joint_names):
                # 根据关节名称查找ID
                joint_id = None
                for id_val, name in self.joint_id_to_name.items():
                    if name == joint_name:
                        joint_id = id_val
                        break
                
                if joint_id is None:
                    # 如果找不到ID映射，使用索引+1作为默认ID
                    joint_id = i + 1
                
                # 添加关节ID
                data.append(joint_id)
                
                # 添加位置
                pos_bytes = struct.pack('<f', self.current_positions[i])
                pos_bytes_array = [ord(b) for b in pos_bytes]
                data.extend(pos_bytes_array)
                
                # 添加速度 (0.0)
                vel_bytes = struct.pack('<f', 0.0)
                vel_bytes_array = [ord(b) for b in vel_bytes]
                data.extend(vel_bytes_array)
            
            # 计算校验和 (所有前面字节的和的低8位)
            checksum = sum(data) & 0xFF
            data.append(checksum)
            
            # 添加结束标记
            data.append(END_MARKER)
            
            # 创建并发布消息
            msg = UInt8MultiArray()
            msg.layout.dim.append(MultiArrayDimension(
                label="length",
                size=len(data),
                stride=1
            ))
            msg.data = list(data)
            self.serial_pub.publish(msg)
            
            # 调试输出反馈数据 - 简化格式化
            debug_str = ' '.join(['%02X' % b for b in data])
            rospy.loginfo("发布反馈数据: [%s]", debug_str)
            
        except Exception as e:
            rospy.logwarn("创建反馈数据出错: %s", e)
            import traceback
            rospy.logwarn(traceback.format_exc())

if __name__ == '__main__':
    try:
        publisher = FakeFeedbackPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 