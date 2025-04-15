#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
import os
import importlib
import sys
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryFeedback

class FeedbackProcessor:
    """处理串口反馈并发布关节状态的通用接口"""
    
    def __init__(self):
        rospy.init_node('feedback_processor')
        
        # 获取当前包名
        self.package_name = self._get_package_name()
        rospy.loginfo("Running in package: {}".format(self.package_name))
        
        # 加载配置
        try:
            config_file = rospy.get_param("~config_file")
        except KeyError:
            default_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                      "config", "arm_controller.yaml")
            rospy.logwarn("No config_file parameter found, using default: %s", default_path)
            config_file = default_path
            
        self.load_config(config_file)
        
        # 加载对应的协议处理器
        self.feedback_handler = self.load_feedback_handler()
        
        # 创建ROS接口
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.feedback_pub = rospy.Publisher(
            "/{}/arm_joint_controller/state".format(self.config['arm']['name']),
            FollowJointTrajectoryFeedback, 
            queue_size=10
        )
        
        self.serial_sub = rospy.Subscriber('/serial_node/data_received', UInt8MultiArray, 
                                         self.serial_data_callback)
        
        rospy.loginfo("Feedback processor node initialized")

        # 立即发布初始关节状态
        self.publish_joint_states(None)
        
        # 定时器 - 增加频率至25Hz
        self.timer = rospy.Timer(rospy.Duration(0.04), self.publish_joint_states_and_controller_feedback)
    
    def _get_package_name(self):
        """获取当前ROS包名"""
        try:
            # 首先尝试从ROS参数服务器获取包名
            if rospy.has_param('/feedback_processor/package_name'):
                return rospy.get_param('/feedback_processor/package_name')
            
            # 使用Python模块路径推导包名
            # 获取脚本所在目录的父目录名称
            script_dir = os.path.dirname(os.path.abspath(__file__))
            package_dir = os.path.dirname(script_dir)
            return os.path.basename(package_dir)
        except Exception as e:
            rospy.logwarn("Unable to determine package name automatically: {}".format(e))
            # 如果出错，返回默认包名
            return "URDF_armv001_demo"
        
    def load_config(self, config_file):
        """加载配置文件"""
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
            rospy.loginfo("Loaded configuration from %s", config_file)
        except Exception as e:
            rospy.logerr("Failed to load config file: %s", e)
            # 使用默认配置
            self.config = {
                'arm': {'name': 'URDF_armv001', 'joint_names': ['first_joint', 'second_joint', 'third_joint', 'fourth_joint', 'fifth_joint', 'sixth_joint']},
                'protocol': {'start_marker': 0xFF, 'end_marker': 0xFE}
            }
            rospy.logwarn("Using default configuration")
    
    def load_feedback_handler(self):
        """根据机械臂名称加载对应的反馈协议处理器"""
        arm_name = self.config['arm']['name']
        try:
            # 将scripts目录添加到搜索路径中
            scripts_dir = os.path.dirname(os.path.abspath(__file__))
            if scripts_dir not in sys.path:
                sys.path.insert(0, scripts_dir)
                
            # 优先尝试直接导入
            try:
                feedback_module = importlib.import_module("feedback_handlers")
            except ImportError:
                # 如果直接导入失败，尝试使用完整包路径
                handlers_module_name = "{}.scripts.feedback_handlers".format(self.package_name)
                feedback_module = importlib.import_module(handlers_module_name)
                
            # 获取反馈处理器工厂函数
            get_handler = getattr(feedback_module, 'get_feedback_handler')
            
            # 根据机械臂名称获取对应的反馈处理器
            handler = get_handler(arm_name, self.config)
            rospy.loginfo("Loaded feedback handler for '{}'".format(arm_name))
            return handler
        except Exception as e:
            rospy.logerr("Failed to load feedback handler for '{}': {}".format(arm_name, e))
            rospy.logerr("Using built-in fallback handler")
            
            # 动态导入DefaultFeedbackHandler
            try:
                # 首先尝试直接从本地导入
                feedback_handlers = importlib.import_module("feedback_handlers")
                return feedback_handlers.DefaultFeedbackHandler(self.config)
            except Exception as e2:
                rospy.logerr("Failed to load even the default handler: {}".format(e2))
                # 最后尝试手动实现一个基本处理器
                return self._create_fallback_handler()
    
    def _create_fallback_handler(self):
        """创建一个基本的应急处理器"""
        import struct
        
        class EmergencyFeedbackHandler:
            def __init__(self, config):
                self.config = config
                
            def parse_feedback_data(self, data):
                """基本的反馈数据解析方法"""
                rospy.logwarn("Using emergency fallback feedback handler!")
                
                # 提取协议常量
                START_MARKER = self.config.get('protocol', {}).get('start_marker', 0xFF)
                END_MARKER = self.config.get('protocol', {}).get('end_marker', 0xFE)
                
                # 检查数据长度
                if len(data) < 4:  # 至少需要起始标记、命令类型、关节数和结束标记
                    return None
                    
                # 检查起始和结束标记
                if data[0] != START_MARKER or data[-1] != END_MARKER:
                    rospy.logwarn("Invalid packet markers")
                    return None
                    
                # 获取关节数量
                joint_count = data[2]
                
                # 尝试用一个简单的格式解析数据
                try:
                    positions = []
                    velocities = []
                    
                    for i in range(joint_count):
                        # 假设每个关节数据是9字节: ID(1) + 位置(4) + 速度(4)
                        base_idx = 3 + i * 9
                        
                        # 跳过ID，直接读取位置和速度
                        pos_bytes = ''.join([chr(b) for b in data[base_idx+1:base_idx+5]])
                        vel_bytes = ''.join([chr(b) for b in data[base_idx+5:base_idx+9]])
                        
                        pos = struct.unpack('<f', pos_bytes)[0]
                        vel = struct.unpack('<f', vel_bytes)[0]
                        
                        positions.append(pos)
                        velocities.append(vel)
                    
                    return {
                        'positions': positions,
                        'velocities': velocities,
                        'accelerations': [0.0] * joint_count,
                        'effort': [0.0] * joint_count
                    }
                except Exception as e:
                    rospy.logerr("Emergency parsing failed: {}".format(e))
                    return None
                
        return EmergencyFeedbackHandler(self.config)
    
    def serial_data_callback(self, msg):
        """处理从串口接收的数据"""
        try:
            # 将字节数组转换为字节
            data = bytearray(msg.data)
            
            # 使用协议处理器解析数据
            joint_states = self.feedback_handler.parse_feedback_data(data)
            
            if joint_states:
                # 发布关节状态
                self.publish_joint_state(joint_states)
                
                # 发布控制器反馈
                self.publish_controller_feedback(joint_states)
                
        except Exception as e:
            rospy.logerr("Error processing serial data: %s", e)
    
    def publish_joint_state(self, joint_states):
        """发布关节状态消息"""
        try:
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.config['arm']['joint_names']
            msg.position = joint_states['positions']
            msg.velocity = joint_states['velocities']
            msg.effort = joint_states['effort']
            
            self.joint_state_pub.publish(msg)
            rospy.logdebug("Published joint states")
        except Exception as e:
            rospy.logerr("Failed to publish joint states: %s", e)
    
    def publish_controller_feedback(self, joint_states):
        """发布控制器反馈消息"""
        try:
            msg = FollowJointTrajectoryFeedback()
            msg.header.stamp = rospy.Time.now()
            msg.joint_names = self.config['arm']['joint_names']
            
            # 设置当前关节状态
            msg.actual.positions = joint_states['positions']
            msg.actual.velocities = joint_states['velocities']
            msg.actual.accelerations = joint_states['accelerations']
            
            # 假设期望状态等于当前状态
            msg.desired = msg.actual
            
            # 计算误差
            msg.error.positions = [0.0] * len(joint_states['positions'])
            msg.error.velocities = [0.0] * len(joint_states['velocities'])
            msg.error.accelerations = [0.0] * len(joint_states['accelerations'])
            
            self.feedback_pub.publish(msg)
            rospy.logdebug("Published controller feedback")
        except Exception as e:
            rospy.logerr("Failed to publish controller feedback: %s", e)

    def publish_joint_states_and_controller_feedback(self, event):
        """发布关节状态和控制器反馈消息"""
        joint_states = self.feedback_handler.arm.get_status(motor_ids=[5,1,4,2,6,3]) 
        joint_states['accelerations'] = [0.0] * len(joint_states['positions'])
        self.publish_joint_state(joint_states)
        self.publish_controller_feedback(joint_states)

if __name__ == '__main__':
    processor = FeedbackProcessor()
    rospy.spin() 