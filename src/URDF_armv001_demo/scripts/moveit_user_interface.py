#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import yaml
import os
import threading
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from moveit_msgs.msg import CollisionObject
from visualization_msgs.msg import Marker

# 导入自定义模块
from moveit_controller import MoveItController
from trajectory_filters import PositionFilter, VelocityFilter

class MoveItUserInterface:
    """
    MoveIt用户接口节点
    提供统一的接口来控制机械臂，支持多种控制方法
    """
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('moveit_user_interface')  # 不使用anonymous=True，因为需要使用ROS_NAMESPACE
        
        # 获取机械臂名称
        self.arm_name = rospy.get_param('~arm_name', 'DeepArm')
        self.group_name = rospy.get_param('~group_name', 'DeepArm')
        
        # 加载配置
        self.load_config()
        
        # 创建控制器实例
        self.controller = MoveItController(self.group_name)
        
        # 创建轨迹滤波器（用于目标跟踪）
        self.position_filter = PositionFilter(
            window_size=5, 
            position_threshold=0.005
        )
        self.velocity_filter = VelocityFilter(
            smoothing_factor=0.7,
            max_velocity=0.1
        )
        
        # 跟踪状态变量
        self.is_tracking = False
        self.tracking_thread = None
        self.stop_tracking = False
        
        # 最后一次执行移动的时间
        self.last_move_time = rospy.Time.now()
        self.move_interval = rospy.Duration(0.2)  # 执行移动的最小时间间隔
        
        # 创建订阅者
        self.create_subscribers()
        
        # 打印状态信息
        rospy.loginfo("MoveIt用户接口已初始化。机械臂：%s，规划组：%s", 
                     self.arm_name, self.group_name)
        rospy.loginfo("可用预设位置: %s", self.controller.get_named_targets())
        
    def load_config(self):
        """加载节点配置"""
        try:
            config_file = rospy.get_param("~config_file", None)
            if config_file is None:
                # 使用默认配置路径
                script_dir = os.path.dirname(os.path.abspath(__file__))
                package_dir = os.path.dirname(script_dir)
                config_file = os.path.join(package_dir, "config", "user_interface.yaml")
                
            if os.path.exists(config_file):
                with open(config_file, 'r') as f:
                    self.config = yaml.safe_load(f)
                rospy.loginfo("已加载配置文件: %s", config_file)
            else:
                self.config = {}
                rospy.logwarn("配置文件不存在: %s, 使用默认配置", config_file)
                
        except Exception as e:
            self.config = {}
            rospy.logerr("加载配置失败: %s", e)
    
    def create_subscribers(self):
        """创建所有订阅者"""
        # 预设位置命令订阅者
        rospy.Subscriber("~named_target", String, self.named_target_callback)
        
        # 关节位置命令订阅者
        rospy.Subscriber("~joint_target", Float64MultiArray, self.joint_target_callback)
        
        # 末端位置命令订阅者
        rospy.Subscriber("~pose_target", PoseStamped, self.pose_target_callback)
        
        # 目标跟踪订阅者
        rospy.Subscriber("~tracking_target", PoseStamped, self.tracking_target_callback)
        
        # 跟踪控制命令
        rospy.Subscriber("~tracking_control", String, self.tracking_control_callback)
        
        # 障碍物订阅者
        rospy.Subscriber("~collision_object", CollisionObject, self.collision_object_callback)
        
        # 末端路径点列表
        rospy.Subscriber("~waypoints", PoseStamped, self.waypoint_callback, queue_size=10)
        
        rospy.loginfo("已创建所有订阅者")
        
    def named_target_callback(self, msg):
        """处理预设位置命令"""
        target_name = msg.data
        rospy.loginfo("执行预设位置: %s", target_name)
        
        try:
            result = self.controller.goto_named_target(target_name)
            if result:
                rospy.loginfo("成功移动到预设位置: %s", target_name)
            else:
                rospy.logwarn("移动到预设位置失败: %s", target_name)
        except Exception as e:
            rospy.logerr("执行预设位置命令时出错: %s", e)
    
    def joint_target_callback(self, msg):
        """处理关节位置命令"""
        joint_positions = msg.data
        rospy.loginfo("执行关节位置命令: %s", joint_positions)
        
        try:
            result = self.controller.goto_joint_target(joint_positions)
            if result:
                rospy.loginfo("成功移动到关节位置")
            else:
                rospy.logwarn("移动到关节位置失败")
        except Exception as e:
            rospy.logerr("执行关节位置命令时出错: %s", e)
    
    def pose_target_callback(self, msg):
        """处理末端位置命令"""
        # 如果正在跟踪目标，忽略单次位置命令
        if self.is_tracking:
            rospy.loginfo("正在跟踪目标，忽略单次位置命令")
            return
        
        pose = msg.pose
        frame_id = msg.header.frame_id or 'base_link'
        
        rospy.loginfo("执行末端位置命令: 位置[%.3f, %.3f, %.3f], 朝向[%.3f, %.3f, %.3f, %.3f], 参考坐标系: %s", 
                     pose.position.x, pose.position.y, pose.position.z,
                     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                     frame_id)
        
        try:
            result = self.controller.goto_pose_target(pose, frame_id)
            if result:
                rospy.loginfo("成功移动到指定位置")
            else:
                rospy.logwarn("移动到指定位置失败")
        except Exception as e:
            rospy.logerr("执行末端位置命令时出错: %s", e)
    
    def tracking_target_callback(self, msg):
        """处理目标跟踪位置"""
        if not self.is_tracking:
            return
            
        # 获取当前时间
        now = rospy.Time.now()
        
        # 应用位置过滤器
        filtered_pose = self.position_filter.update(msg.pose)
        
        # 只有在间隔足够长，且位置变化超过阈值时才执行移动
        if now - self.last_move_time > self.move_interval:
            self.last_move_time = now
            
            # 使用速度滤波器生成平滑的运动
            smoothed_pose = self.velocity_filter.update(filtered_pose)
            
            frame_id = msg.header.frame_id or 'base_link'
            
            try:
                # 使用笛卡尔路径规划进行跟踪（更为平滑）
                result = self.controller.plan_cartesian_path([smoothed_pose], frame_id)
                if result:
                    self.controller.execute_plan(result)
                    rospy.logdebug("跟踪目标更新: 位置[%.3f, %.3f, %.3f]", 
                                 smoothed_pose.position.x, 
                                 smoothed_pose.position.y, 
                                 smoothed_pose.position.z)
            except Exception as e:
                rospy.logwarn("目标跟踪执行出错: %s", e)
    
    def tracking_control_callback(self, msg):
        """控制目标跟踪状态"""
        command = msg.data.lower()
        
        if command == "start" and not self.is_tracking:
            self.start_tracking()
        elif command == "stop" and self.is_tracking:
            self.stop_tracking_process()
    
    def start_tracking(self):
        """启动目标跟踪过程"""
        self.is_tracking = True
        self.stop_tracking = False
        self.position_filter.reset()
        self.velocity_filter.reset()
        
        # 创建跟踪线程
        self.tracking_thread = threading.Thread(target=self.tracking_process)
        self.tracking_thread.daemon = True
        self.tracking_thread.start()
        
        rospy.loginfo("开始目标跟踪")
    
    def stop_tracking_process(self):
        """停止目标跟踪过程"""
        self.stop_tracking = True
        self.is_tracking = False
        
        if self.tracking_thread:
            self.tracking_thread.join(timeout=1.0)
            self.tracking_thread = None
            
        rospy.loginfo("停止目标跟踪")
    
    def tracking_process(self):
        """目标跟踪主过程"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not self.stop_tracking and not rospy.is_shutdown():
            # 主要工作在回调中完成
            rate.sleep()
    
    def collision_object_callback(self, msg):
        """处理碰撞对象"""
        operation = msg.operation
        
        if operation == CollisionObject.ADD:
            rospy.loginfo("添加碰撞对象: %s", msg.id)
            self.controller.add_collision_object(msg)
        elif operation == CollisionObject.REMOVE:
            rospy.loginfo("移除碰撞对象: %s", msg.id)
            self.controller.remove_collision_object(msg.id)
        elif operation == CollisionObject.MOVE:
            rospy.loginfo("移动碰撞对象: %s", msg.id)
            self.controller.update_collision_object(msg)
    
    def waypoint_callback(self, msg):
        """处理末端路径点"""
        # 添加到路径点缓存
        self.waypoints_buffer.append(msg.pose)
        
        # 当积累了足够多的路径点或时间间隔足够长，执行路径规划
        if len(self.waypoints_buffer) >= 5:
            rospy.loginfo("规划通过 %d 个路径点的轨迹", len(self.waypoints_buffer))
            
            try:
                frame_id = msg.header.frame_id or 'base_link'
                result = self.controller.plan_cartesian_path(self.waypoints_buffer, frame_id)
                
                if result:
                    rospy.loginfo("成功规划路径，覆盖率: %.2f%%", result[1] * 100)
                    self.controller.execute_plan(result[0])
                else:
                    rospy.logwarn("路径点规划失败")
            except Exception as e:
                rospy.logerr("执行路径点规划时出错: %s", e)
            
            # 清空缓存
            self.waypoints_buffer = []

if __name__ == '__main__':
    try:
        interface = MoveItUserInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 