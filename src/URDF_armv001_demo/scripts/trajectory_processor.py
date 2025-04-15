#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import yaml
import os
import importlib
import sys
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult
)
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension

class TrajectoryProcessor:
    """处理轨迹并转换为串口命令的通用接口"""
    
    def __init__(self):
        rospy.init_node('trajectory_processor')
        
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
        self.protocol_handler = self.load_protocol_handler()
        
        # 创建ROS接口
        self.cmd_pub = rospy.Publisher('/serial_node/data_to_send', UInt8MultiArray, queue_size=10)
        
        # 创建Action服务器
        self.action_server = actionlib.SimpleActionServer(
            "/{}/arm_joint_controller/follow_joint_trajectory".format(self.config['arm']['name']),
            FollowJointTrajectoryAction,
            execute_cb=self.execute_trajectory_cb,
            auto_start=False
        )
        
        self.action_server.start()
        rospy.loginfo("Trajectory processor node initialized")
        
        # 添加这段来调试话题匹配问题
        self.debug_action_topics()
        
    def _get_package_name(self):
        """获取当前ROS包名"""
        try:
            # 首先尝试从ROS参数服务器获取包名
            if rospy.has_param('/trajectory_processor/package_name'):
                return rospy.get_param('/trajectory_processor/package_name')
            
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
                'protocol': {'start_marker': 0xFF, 'end_marker': 0xFE, 'cmd_type': 0x01}
            }
            rospy.logwarn("Using default configuration")
    
    def load_protocol_handler(self):
        """根据机械臂名称加载对应的协议处理器"""
        arm_name = self.config['arm']['name']
        try:
            # 使用当前包名动态导入协议处理器模块
            handlers_module_name = "{}.scripts.protocol_handlers".format(self.package_name)
            
            # 将scripts目录添加到搜索路径中
            scripts_dir = os.path.dirname(os.path.abspath(__file__))
            if scripts_dir not in sys.path:
                sys.path.insert(0, scripts_dir)
                
            # 优先尝试直接导入
            try:
                protocol_module = importlib.import_module("protocol_handlers")
            except ImportError:
                # 如果直接导入失败，尝试使用完整包路径
                protocol_module = importlib.import_module(handlers_module_name)
                
            # 获取协议处理器工厂函数
            get_handler = getattr(protocol_module, 'get_protocol_handler')
            
            # 根据机械臂名称获取对应的协议处理器
            handler = get_handler(arm_name, self.config)
            rospy.loginfo("Loaded protocol handler for '{}'".format(arm_name))
            return handler
        except Exception as e:
            rospy.logerr("Failed to load protocol handler for '{}': {}".format(arm_name, e))
            rospy.logerr("Using built-in fallback handler")
            
            # 动态导入DefaultProtocolHandler
            try:
                # 首先尝试直接从本地导入
                protocol_handlers = importlib.import_module("protocol_handlers")
                return protocol_handlers.DefaultProtocolHandler(self.config)
            except Exception as e2:
                rospy.logerr("Failed to load even the default handler: {}".format(e2))
                # 最后尝试手动实现一个基本处理器
                return self._create_fallback_handler()
    
    def _create_fallback_handler(self):
        """创建一个基本的应急处理器"""
        import struct
        
        class EmergencyFallbackHandler:
            def __init__(self, config):
                self.config = config
                
            def format_trajectory_point(self, point):
                """基本的轨迹点格式化方法"""
                rospy.logwarn("Using emergency fallback protocol handler!")
                # 基本协议参数
                START_MARKER = 0xFF
                CMD_TYPE = 0x01
                END_MARKER = 0xFE
                
                positions = point['positions']
                velocities = point['velocities']
                joint_count = len(positions)
                
                data = bytearray()
                data.append(START_MARKER)
                data.append(CMD_TYPE)
                data.append(joint_count)
                
                for i in range(joint_count):
                    data.extend(struct.pack('<f', positions[i]))
                    data.extend(struct.pack('<f', velocities[i]))
                
                checksum = sum(data) & 0xFF
                data.append(checksum)
                data.append(END_MARKER)
                
                return data
                
        return EmergencyFallbackHandler(self.config)

    def execute_trajectory_cb(self, goal):
        """Action回调，处理轨迹目标"""
        rospy.loginfo("Received trajectory with %d points", len(goal.trajectory.points))
        
        # 检查是否需要中止
        if self.action_server.is_preempt_requested():
            self.action_server.set_preempted()
            return
        
        feedback = FollowJointTrajectoryFeedback()
        result = FollowJointTrajectoryResult()
        
        # 获取轨迹中的关节名称
        joint_names = goal.trajectory.joint_names
        
        # 处理轨迹中的每个点
        start_time = rospy.Time.now()
        for i, point in enumerate(goal.trajectory.points):
            # 等待直到达到轨迹点的时间
            target_time = start_time + point.time_from_start
            while rospy.Time.now() < target_time and not rospy.is_shutdown():
                rospy.sleep(0.001)
                
                # 检查是否需要中止
                if self.action_server.is_preempt_requested():
                    self.action_server.set_preempted()
                    return
            
            # 构造轨迹点数据
            point_data = {
                'joint_names': joint_names,  # 添加关节名称
                'time_from_start': point.time_from_start,
                'positions': point.positions,
                'velocities': point.velocities if point.velocities else [0.0] * len(point.positions)
            }
            
            # 使用协议处理器格式化数据
            serial_data = self.protocol_handler.format_trajectory_point(point_data)
            self.send_serial_data(serial_data)
            
            # 更新反馈
            progress = 100.0 * (i + 1) / len(goal.trajectory.points)
            rospy.loginfo("Sent point %d/%d (%.1f%%) at time: %.2fs", 
                         i+1, len(goal.trajectory.points), progress, point.time_from_start.to_sec())
        
        # 轨迹执行完成
        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.action_server.set_succeeded(result)
        rospy.loginfo("Trajectory execution completed")
    
    def send_serial_data(self, data):
        """发送数据到串口节点"""
        try:
            msg = UInt8MultiArray()
            msg.layout.dim.append(MultiArrayDimension(
                label="length",
                size=len(data),
                stride=1
            ))
            msg.data = list(data)
            self.cmd_pub.publish(msg)
        except Exception as e:
            rospy.logerr("Failed to send data to serial node: %s", e)

    def debug_action_topics(self):
        """监控与Action相关的话题"""
        import subprocess
        try:
            output = subprocess.check_output(["rostopic", "list"]).decode('utf-8')
            action_topics = [t for t in output.split('\n') if 'follow_joint_trajectory' in t]
            rospy.loginfo("找到的Action话题: %s", action_topics)
        except Exception as e:
            rospy.logerr("调试话题时出错: %s", e)

if __name__ == '__main__':
    processor = TrajectoryProcessor()
    rospy.spin() 