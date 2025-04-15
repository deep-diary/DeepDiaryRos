#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import thread
import copy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

class AttachedObjectDemo:
    """附着物体演示类，展示如何在RVIZ场景中添加物体并进行抓取演示"""
    
    def __init__(self):
        # 初始化moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('attached_object_demo', anonymous=True)
        
        # 初始化场景对象
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)  # 等待场景接口初始化
        
        # 初始化机械臂规划组
        self.arm = moveit_commander.MoveGroupCommander("DeepArm")
        
        # 获取末端执行器链接名称
        self.end_effector_link = self.arm.get_end_effector_link()
        rospy.loginfo("末端执行器: %s", self.end_effector_link)
        
        # 设置参考系
        self.reference_frame = "base_link"
        self.arm.set_pose_reference_frame(self.reference_frame)
        
        # 允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置位置和姿态误差允许范围
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)
        
        # 设置最大速度和加速度缩放因子
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        
    def run_demo(self):
        """运行演示"""
        try:
            # 清除场景中的物体
            self.scene.remove_world_object()
            
            # 控制机械臂先回到初始化位置
            rospy.loginfo("移动到home位置")
            self.arm.set_named_target('home')
            self.arm.go()
            rospy.sleep(1)
            
            # 添加桌子作为障碍物
            self.add_table()
            
            # 添加物体
            self.add_box()
            rospy.sleep(1)
            
            # 尝试抓取物体
            self.pick_object()
            rospy.sleep(1)
            
            # 移动物体
            self.place_object()
            rospy.sleep(1)
            
            # 返回初始位置
            rospy.loginfo("返回home位置")
            self.arm.set_named_target('home')
            self.arm.go()
            rospy.sleep(1)
            
            # 清除场景
            self.scene.remove_world_object()
            
            return True
            
        except Exception as e:
            rospy.logerr("演示执行出错: %s", e)
            return False
    
    def add_table(self):
        """添加桌子到场景"""
        rospy.loginfo("添加桌子到场景")
        
        # 创建桌子
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.reference_frame
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -0.1
        table_pose.pose.orientation.w = 1.0
        
        # 添加桌子到场景
        self.scene.add_box("table", table_pose, size=(1.0, 1.0, 0.05))
    
    def add_box(self):
        """添加盒子到场景"""
        rospy.loginfo("添加盒子到场景")
        
        # 创建盒子
        box_pose = PoseStamped()
        box_pose.header.frame_id = self.reference_frame
        box_pose.pose.position.x = 0.25
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.05
        box_pose.pose.orientation.w = 1.0
        
        # 添加盒子到场景
        self.scene.add_box("box", box_pose, size=(0.05, 0.05, 0.05))
    
    def pick_object(self):
        """抓取物体"""
        rospy.loginfo("设置抓取物体的位置")
        
        # 设置目标位置
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.pose.position.x = 0.25
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.10  # 稍微高于盒子
        target_pose.pose.orientation.w = 1.0
        
        # 规划并执行
        self.arm.set_pose_target(target_pose)
        self.arm.go()
        rospy.sleep(1)
        
        # 向下靠近物体
        target_pose.pose.position.z = 0.075  # 降低高度
        self.arm.set_pose_target(target_pose)
        self.arm.go()
        rospy.sleep(1)
        
        # 附着物体
        rospy.loginfo("附着物体到末端执行器")
        self.scene.attach_box(self.end_effector_link, "box")
        rospy.sleep(1)
        
        # 抬起物体
        target_pose.pose.position.z = 0.2
        self.arm.set_pose_target(target_pose)
        self.arm.go()
        rospy.sleep(1)
    
    def place_object(self):
        """放置物体"""
        rospy.loginfo("移动到放置位置")
        
        # 设置放置位置
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.pose.position.x = 0.1
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.2
        target_pose.pose.orientation.w = 1.0
        
        # 移动到放置位置上方
        self.arm.set_pose_target(target_pose)
        self.arm.go()
        rospy.sleep(1)
        
        # 降低高度
        target_pose.pose.position.z = 0.075
        self.arm.set_pose_target(target_pose)
        self.arm.go()
        rospy.sleep(1)
        
        # 分离物体
        rospy.loginfo("分离物体")
        self.scene.remove_attached_object(self.end_effector_link, "box")
        
        # 重新添加物体到场景中新位置
        box_pose = PoseStamped()
        box_pose.header.frame_id = self.reference_frame
        box_pose.pose.position.x = 0.1
        box_pose.pose.position.y = 0.2
        box_pose.pose.position.z = 0.05
        box_pose.pose.orientation.w = 1.0
        
        self.scene.add_box("box", box_pose, size=(0.05, 0.05, 0.05))
        
        # 抬起机械臂
        target_pose.pose.position.z = 0.2
        self.arm.set_pose_target(target_pose)
        self.arm.go()
        rospy.sleep(1)
    
    def shutdown(self):
        """关闭节点"""
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("演示已完成")

if __name__ == "__main__":
    try:
        demo = AttachedObjectDemo()
        success = demo.run_demo()
        demo.shutdown()
        
        if success:
            rospy.loginfo("附着物体演示完成")
        else:
            rospy.logerr("附着物体演示失败")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("用户中断演示") 