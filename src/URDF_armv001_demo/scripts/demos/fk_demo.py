#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose
import numpy as np

class FkDemo:
    """正向运动学演示"""
    
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('DeepArm')
        
        # 设置机械臂运动的允许误差值
        self.arm.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取当前的状态
        rospy.loginfo("当前关节角度: %s", self.arm.get_current_joint_values())
        curr_pose = self.arm.get_current_pose().pose
        rospy.loginfo("当前末端位置: 位置(%.3f, %.3f, %.3f) 姿态(%.3f, %.3f, %.3f, %.3f)", 
                   curr_pose.position.x, curr_pose.position.y, curr_pose.position.z,
                   curr_pose.orientation.x, curr_pose.orientation.y,
                   curr_pose.orientation.z, curr_pose.orientation.w)
        
    def run_demo(self):
        """运行演示"""
        # 控制机械臂先回到初始化位置
        rospy.loginfo("移动到home位置")
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
         
        try:
            # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
            joint_positions = [-0.65, -1.5, 1.4, 0.27, -0.79, 0.45]
            rospy.loginfo("设置关节目标: %s", joint_positions)
            self.arm.set_joint_value_target(joint_positions)
                     
            # 控制机械臂完成运动
            rospy.loginfo("执行关节运动...")
            self.arm.go()
            rospy.sleep(1)
            
            # 再设置另一个目标
            joint_positions = [0.5, -0.8, 0.6, -0.5, 0.3, 0.2]
            rospy.loginfo("设置关节目标: %s", joint_positions)
            self.arm.set_joint_value_target(joint_positions)
            
            # 控制机械臂完成运动
            rospy.loginfo("执行关节运动...")
            self.arm.go()
            rospy.sleep(1)
    
            # 控制机械臂回到初始化位置
            rospy.loginfo("返回home位置")
            self.arm.set_named_target('home')
            self.arm.go()
            rospy.sleep(1)
            
            return True
            
        except Exception as e:
            rospy.logerr("运动规划或执行出错: %s", e)
            return False
            
    def shutdown(self):
        """关闭节点"""
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("FK演示已完成")

if __name__ == "__main__":
    try:
        demo = FkDemo()
        success = demo.run_demo()
        demo.shutdown()
        
        if success:
            rospy.loginfo("正向运动学演示完成")
        else:
            rospy.logerr("正向运动学演示失败")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("用户中断演示")
        pass 