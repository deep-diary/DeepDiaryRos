#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
import time
import numpy as np
from tf.transformations import quaternion_from_euler

class PoseCommander:
    """发送末端位姿命令的演示 - 使用正向运动学计算可达点"""
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('pose_commander')
        
        # 创建发布器
        self.pose_pub = rospy.Publisher('/moveit_user_interface/pose_target', 
                                       PoseStamped, queue_size=10)
        
        # 初始化MoveIt接口
        self.moveit_init()
        
        # 等待发布器连接
        rospy.loginfo("等待连接到用户接口...")
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while self.pose_pub.get_num_connections() == 0:
            if rospy.Time.now() > timeout:
                rospy.logwarn("超时等待连接，继续执行...")
                break
            rospy.sleep(0.1)
                
        rospy.loginfo("已连接到用户接口或超时继续")
    
    def moveit_init(self):
        """初始化MoveIt并计算可达位置"""
        moveit_commander.roscpp_initialize(sys.argv)
        self.group_name = "DeepArm"
        self.arm = moveit_commander.MoveGroupCommander(self.group_name)
        
        # 获取参考坐标系
        self.reference_frame = self.arm.get_planning_frame()
        rospy.loginfo("规划参考坐标系: %s", self.reference_frame)
        
        # 获取末端执行器链接
        self.end_effector_link = self.arm.get_end_effector_link()
        rospy.loginfo("末端执行器链接: %s", self.end_effector_link)
        
        # 计算已知可达点
        self.generate_reachable_poses()
    
    def get_joint_limits(self):
        """获取关节限制信息 - 使用兼容方法"""
        limits = {}
        try:
            # 尝试使用get_variable_bounds方法
            for joint_name in self.arm.get_active_joints():
                try:
                    bounds = self.arm.get_variable_bounds(joint_name)
                    if bounds:
                        limits[joint_name] = bounds
                except Exception:
                    pass
            
            if not limits:
                # 如果上面的方法失败，尝试读取默认值
                rospy.logwarn("无法获取关节限制，使用默认值")
                for joint_name in self.arm.get_active_joints():
                    limits[joint_name] = [[-3.14, 3.14]]  # 默认限制为±π
        except Exception as e:
            rospy.logwarn("获取关节限制失败: %s", e)
            
        rospy.loginfo("关节限制: %s", limits)
        return limits
    
    def generate_reachable_poses(self):
        """生成已知可达的位姿"""
        self.reachable_poses = []
        
        # 从预定义位置获取可达点
        for target in self.arm.get_named_targets():
            try:
                self.arm.set_named_target(target)
                self.arm.go(wait=True)
                
                # 获取当前姿态
                current_pose = self.arm.get_current_pose().pose
                self.reachable_poses.append({
                    'name': target,
                    'position': [current_pose.position.x, 
                                current_pose.position.y, 
                                current_pose.position.z],
                    'orientation': [current_pose.orientation.x,
                                   current_pose.orientation.y,
                                   current_pose.orientation.z,
                                   current_pose.orientation.w]
                })
                rospy.loginfo("添加可达位姿 '%s': 位置 [%.3f, %.3f, %.3f]", 
                             target, 
                             current_pose.position.x,
                             current_pose.position.y,
                             current_pose.position.z)
            except Exception as e:
                rospy.logwarn("获取位姿 '%s' 失败: %s", target, e)
        
        # 如果没有找到预定义位置，添加一些基本可达点
        if not self.reachable_poses:
            # 获取当前位置作为参考点
            try:
                current_pose = self.arm.get_current_pose().pose
                home_pos = [current_pose.position.x, 
                           current_pose.position.y, 
                           current_pose.position.z]
                home_ori = [current_pose.orientation.x,
                           current_pose.orientation.y,
                           current_pose.orientation.z,
                           current_pose.orientation.w]
                
                # 添加基于当前位置的小偏移
                self.reachable_poses.append({
                    'name': 'current',
                    'position': home_pos,
                    'orientation': home_ori
                })
                
                # 向前移动5cm
                self.reachable_poses.append({
                    'name': 'forward',
                    'position': [home_pos[0] + 0.05, home_pos[1], home_pos[2]],
                    'orientation': home_ori
                })
                
                # 向上5cm
                self.reachable_poses.append({
                    'name': 'up',
                    'position': [home_pos[0], home_pos[1], home_pos[2] + 0.05],
                    'orientation': home_ori
                })
                
                # 向右5cm
                self.reachable_poses.append({
                    'name': 'right',
                    'position': [home_pos[0], home_pos[1] + 0.05, home_pos[2]],
                    'orientation': home_ori
                })
                
            except Exception as e:
                rospy.logwarn("获取当前位置失败，使用默认位置: %s", e)
                
                # 添加一些基于经验的安全位置
                vertical_orientation = quaternion_from_euler(0, 0, 0)
                
                self.reachable_poses = [
                    {
                        'name': 'default',
                        'position': [0.25, 0.0, 0.25],
                        'orientation': list(vertical_orientation)
                    },
                    {
                        'name': 'front',
                        'position': [0.30, 0.0, 0.25],
                        'orientation': list(vertical_orientation)
                    },
                    {
                        'name': 'side',
                        'position': [0.25, 0.1, 0.25],
                        'orientation': list(vertical_orientation)
                    },
                    {
                        'name': 'low',
                        'position': [0.25, 0.0, 0.20],
                        'orientation': list(vertical_orientation)
                    }
                ]
    
    def send_pose(self, position, orientation):
        """发送末端位姿命令"""
        rospy.loginfo("发送末端位姿: 位置 %s, 姿态 %s", position, orientation)
        
        # 创建消息
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.reference_frame
        
        # 设置位置
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        
        # 设置姿态
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3]
        
        # 发布消息
        self.pose_pub.publish(msg)
    
    def run_demo(self):
        """运行演示 - 使用已知可达点"""
        try:
            # 等待1秒确保节点已就绪
            rospy.sleep(1)
            
            if not self.reachable_poses:
                rospy.logerr("没有可达的位姿")
                return False
            
            # 发送每个可达位姿
            for pose in self.reachable_poses:
                rospy.loginfo("移动到位姿: %s", pose['name'])
                self.send_pose(pose['position'], pose['orientation'])
                rospy.sleep(5)  # 给机械臂更多时间到达位置
            
            # 最后回到第一个位姿（如果可用）
            if self.reachable_poses:
                first_pose = self.reachable_poses[0]
                rospy.loginfo("返回到初始位姿: %s", first_pose['name'])
                self.send_pose(first_pose['position'], first_pose['orientation'])
                rospy.sleep(5)
                
            rospy.loginfo("演示完成")
            return True
            
        except KeyboardInterrupt:
            rospy.loginfo("用户中断演示")
            return False
            
        except Exception as e:
            rospy.logerr("演示出错: %s", e)
            return False
        
        finally:
            # 清理MoveIt资源
            moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        commander = PoseCommander()
        commander.run_demo()
    except rospy.ROSInterruptException:
        pass 