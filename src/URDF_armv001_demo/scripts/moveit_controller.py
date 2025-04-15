#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, DisplayTrajectory
from shape_msgs.msg import SolidPrimitive
from copy import deepcopy

class MoveItController:
    """
    MoveIt控制器
    封装MoveIt功能，提供更简单的接口
    """
    
    def __init__(self, group_name):
        """
        初始化MoveIt控制器
        
        Args:
            group_name: 规划组名称
        """
        # 初始化moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 获取机器人对象
        self.robot = moveit_commander.RobotCommander()
        
        # 获取场景对象
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # 创建move_group对象
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        
        # 获取规划参考坐标系
        self.reference_frame = self.move_group.get_planning_frame()
        
        # 获取末端执行器链接
        self.end_effector_link = self.move_group.get_end_effector_link()
        
        # 设置位置和姿态的允许误差
        self.move_group.set_goal_position_tolerance(0.001)
        self.move_group.set_goal_orientation_tolerance(0.01)
        
        # 允许重新规划
        self.move_group.allow_replanning(True)
        
        # 设置规划时间
        self.move_group.set_planning_time(5.0)
        
        # 设置最大速度和加速度缩放因子
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        
        # 碰撞对象字典
        self.collision_objects = {}
        
        # 创建轨迹显示发布者
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=20
        )
        
        rospy.loginfo("MoveIt控制器已初始化: 规划组=%s, 末端执行器=%s, 参考坐标系=%s", 
                     group_name, self.end_effector_link, self.reference_frame)
    
    def get_named_targets(self):
        """获取所有已知的命名目标位置"""
        return self.move_group.get_named_targets()
    
    def get_current_joint_values(self):
        """获取当前关节位置"""
        return self.move_group.get_current_joint_values()
    
    def get_current_pose(self):
        """获取当前末端位姿"""
        return self.move_group.get_current_pose().pose
    
    def goto_named_target(self, target_name):
        """
        移动到命名的目标位置
        
        Args:
            target_name: 目标位置名称，如'home'
            
        Returns:
            bool: 是否成功
        """
        if target_name not in self.get_named_targets():
            rospy.logwarn("未知的目标位置名称: %s", target_name)
            return False
        
        self.move_group.set_named_target(target_name)
        return self.move_group.go(wait=True)
    
    def goto_joint_target(self, joint_positions):
        """
        移动到指定的关节位置
        
        Args:
            joint_positions: 关节位置列表
            
        Returns:
            bool: 是否成功
        """
        # 检查关节位置长度是否匹配
        current_joints = self.get_current_joint_values()
        if len(joint_positions) != len(current_joints):
            rospy.logwarn("关节位置长度不匹配: 提供=%d, 需要=%d", 
                         len(joint_positions), len(current_joints))
            return False
        
        self.move_group.set_joint_value_target(joint_positions)
        return self.move_group.go(wait=True)
    
    def goto_pose_target(self, pose, frame_id=None):
        """
        移动末端执行器到指定位姿
        
        Args:
            pose: 目标位姿
            frame_id: 参考坐标系ID
            
        Returns:
            bool: 是否成功
        """
        if frame_id and frame_id != self.reference_frame:
            # 如果需要，可以在这里添加坐标系转换
            self.move_group.set_pose_reference_frame(frame_id)
        
        self.move_group.set_pose_target(pose)
        success = self.move_group.go(wait=True)
        
        # 恢复默认参考坐标系
        if frame_id and frame_id != self.reference_frame:
            self.move_group.set_pose_reference_frame(self.reference_frame)
            
        return success
    
    def plan_cartesian_path(self, waypoints, frame_id=None, eef_step=0.01, jump_threshold=0.0):
        """
        规划笛卡尔路径
        
        Args:
            waypoints: 路径点列表
            frame_id: 参考坐标系ID
            eef_step: 末端执行器最大步长
            jump_threshold: 跳跃阈值
            
        Returns:
            tuple: (轨迹, 成功率)
        """
        if frame_id and frame_id != self.reference_frame:
            self.move_group.set_pose_reference_frame(frame_id)
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                waypoints,   # 路径点
                                eef_step,    # 步长
                                jump_threshold,  # 跳跃阈值
                                avoid_collisions=True)
        
        # 恢复默认参考坐标系
        if frame_id and frame_id != self.reference_frame:
            self.move_group.set_pose_reference_frame(self.reference_frame)
            
        return (plan, fraction)
    
    def execute_plan(self, plan):
        """
        执行已规划的轨迹
        
        Args:
            plan: 由plan_cartesian_path返回的轨迹
            
        Returns:
            bool: 是否成功
        """
        return self.move_group.execute(plan, wait=True)
    
    def add_collision_object(self, collision_object):
        """
        添加碰撞对象到规划场景
        
        Args:
            collision_object: 碰撞对象消息
        """
        self.collision_objects[collision_object.id] = collision_object
        self.scene.add_object(collision_object)
    
    def remove_collision_object(self, object_id):
        """
        从规划场景中移除碰撞对象
        
        Args:
            object_id: 碰撞对象ID
        """
        if object_id in self.collision_objects:
            del self.collision_objects[object_id]
        self.scene.remove_world_object(object_id)
    
    def update_collision_object(self, collision_object):
        """
        更新碰撞对象位置
        
        Args:
            collision_object: 碰撞对象消息
        """
        self.remove_collision_object(collision_object.id)
        self.add_collision_object(collision_object)
    
    def attach_object(self, object_id, link_name=None):
        """
        将对象附着到机器人链接
        
        Args:
            object_id: 碰撞对象ID
            link_name: 要附着到的链接名称，默认使用末端执行器
        """
        if link_name is None:
            link_name = self.end_effector_link
            
        self.scene.attach_object(object_id, link_name)
    
    def detach_object(self, object_id):
        """
        分离附着的对象
        
        Args:
            object_id: 碰撞对象ID
        """
        self.scene.remove_attached_object(self.end_effector_link, name=object_id)
    
    def display_trajectory(self, plan):
        """
        在RViz中显示轨迹
        
        Args:
            plan: 轨迹计划
        """
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory) 