#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from collections import deque
from copy import deepcopy

class PositionFilter:
    """
    位置滤波器
    基于移动窗口平均，减少目标位置抖动
    """
    
    def __init__(self, window_size=5, position_threshold=0.005):
        """
        初始化位置滤波器
        
        Args:
            window_size: 滑动窗口大小
            position_threshold: 位置变化阈值，小于此值的变化将被忽略
        """
        self.window_size = window_size
        self.position_threshold = position_threshold
        self.position_buffer = deque(maxlen=window_size)
        self.last_filtered_position = None
    
    def reset(self):
        """重置滤波器状态"""
        self.position_buffer.clear()
        self.last_filtered_position = None
    
    def update(self, pose):
        """
        更新并滤波新的位置
        
        Args:
            pose: 新位置
            
        Returns:
            Pose: 滤波后的位置
        """
        # 深拷贝输入位姿，防止修改原始对象
        filtered_pose = deepcopy(pose)
        
        # 将新位置添加到缓冲区
        self.position_buffer.append((
            pose.position.x,
            pose.position.y,
            pose.position.z
        ))
        
        # 如果缓冲区未满，直接返回当前位置
        if len(self.position_buffer) < 2:
            self.last_filtered_position = (
                pose.position.x,
                pose.position.y,
                pose.position.z
            )
            return filtered_pose
        
        # 计算平均位置
        positions = np.array(self.position_buffer)
        avg_position = np.mean(positions, axis=0)
        
        # 如果有上一次的滤波位置，检查变化是否超过阈值
        if self.last_filtered_position is not None:
            last_pos = np.array(self.last_filtered_position)
            diff = np.linalg.norm(avg_position - last_pos)
            
            # 如果变化小于阈值，保持上一次的位置
            if diff < self.position_threshold:
                filtered_pose.position.x = self.last_filtered_position[0]
                filtered_pose.position.y = self.last_filtered_position[1]
                filtered_pose.position.z = self.last_filtered_position[2]
                return filtered_pose
        
        # 更新滤波后的位置
        filtered_pose.position.x = avg_position[0]
        filtered_pose.position.y = avg_position[1]
        filtered_pose.position.z = avg_position[2]
        
        # 保存最新的滤波位置
        self.last_filtered_position = (
            filtered_pose.position.x,
            filtered_pose.position.y,
            filtered_pose.position.z
        )
        
        return filtered_pose

class VelocityFilter:
    """
    速度滤波器
    基于低通滤波，平滑轨迹运动
    """
    
    def __init__(self, smoothing_factor=0.5, max_velocity=0.1):
        """
        初始化速度滤波器
        
        Args:
            smoothing_factor: 平滑因子 (0-1)，越大平滑效果越强
            max_velocity: 最大允许速度，单位米/秒
        """
        self.smoothing_factor = smoothing_factor
        self.max_velocity = max_velocity
        self.last_position = None
        self.last_velocity = np.zeros(3)
        self.last_time = None
    
    def reset(self):
        """重置滤波器状态"""
        self.last_position = None
        self.last_velocity = np.zeros(3)
        self.last_time = None
    
    def update(self, pose):
        """
        更新并滤波新的位置
        
        Args:
            pose: 新位置
            
        Returns:
            Pose: 按照速度限制平滑后的位置
        """
        # 深拷贝输入位姿，防止修改原始对象
        smoothed_pose = deepcopy(pose)
        
        # 获取当前位置
        current_position = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])
        
        # 获取当前时间
        current_time = rospy.Time.now()
        
        # 如果是第一次调用，直接返回当前位置
        if self.last_position is None:
            self.last_position = current_position
            self.last_time = current_time
            return smoothed_pose
        
        # 计算时间间隔（秒）
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:
            dt = 0.01  # 防止除零错误
        
        # 计算当前速度
        current_velocity = (current_position - self.last_position) / dt
        
        # 应用低通滤波计算平滑速度
        filtered_velocity = self.smoothing_factor * self.last_velocity + \
                           (1 - self.smoothing_factor) * current_velocity
        
        # 限制最大速度
        velocity_magnitude = np.linalg.norm(filtered_velocity)
        if velocity_magnitude > self.max_velocity:
            filtered_velocity = filtered_velocity * (self.max_velocity / velocity_magnitude)
        
        # 根据平滑速度更新位置
        smoothed_position = self.last_position + filtered_velocity * dt
        
        # 更新滤波后的位置
        smoothed_pose.position.x = smoothed_position[0]
        smoothed_pose.position.y = smoothed_position[1]
        smoothed_pose.position.z = smoothed_position[2]
        
        # 保存状态供下次使用
        self.last_position = smoothed_position
        self.last_velocity = filtered_velocity
        self.last_time = current_time
        
        return smoothed_pose 