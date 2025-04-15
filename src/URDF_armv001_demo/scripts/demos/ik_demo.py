#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose

class IkDemo:
    """逆向运动学演示"""
    
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('ik_demo', anonymous=True)
                
        # 初始化需要使用move group控制的机械臂
        self.arm = moveit_commander.MoveGroupCommander('DeepArm')
                
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
        rospy.loginfo("末端执行器: %s", self.end_effector_link)
                        
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)
        
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.3)
        self.arm.set_max_velocity_scaling_factor(0.3)
        
        # 获取当前位姿
        current_pose = self.arm.get_current_pose(self.end_effector_link).pose
        rospy.loginfo("当前末端位置: 位置(%.3f, %.3f, %.3f) 姿态(%.3f, %.3f, %.3f, %.3f)", 
                   current_pose.position.x, current_pose.position.y, current_pose.position.z,
                   current_pose.orientation.x, current_pose.orientation.y,
                   current_pose.orientation.z, current_pose.orientation.w)
        
    def run_demo(self):
        """运行演示"""
        # 控制机械臂先回到初始化位置
        rospy.loginfo("移动到home位置")
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
               
        try:
            # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
            # 姿态使用四元数描述，基于base_link坐标系
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.reference_frame
            target_pose.header.stamp = rospy.Time.now()     
            target_pose.pose.position.x = 0.2593
            target_pose.pose.position.y = 0.0636
            target_pose.pose.position.z = 0.1787
            target_pose.pose.orientation.x = 0.70692
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = 0.0
            target_pose.pose.orientation.w = 0.70729
            
            # 设置目标位姿
            rospy.loginfo("设置目标位姿: 位置(%.3f, %.3f, %.3f) 姿态(%.3f, %.3f, %.3f, %.3f)", 
                       target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
                       target_pose.pose.orientation.x, target_pose.pose.orientation.y,
                       target_pose.pose.orientation.z, target_pose.pose.orientation.w)
            
            # 设置机器臂当前的状态作为运动初始状态
            self.arm.set_start_state_to_current_state()
            
            # 设置机械臂终端运动的目标位姿
            self.arm.set_pose_target(target_pose, self.end_effector_link)
            
            # 规划运动路径
            rospy.loginfo("正在规划路径...")
            traj = self.arm.plan()
            
            # 按照规划的运动路径控制机械臂运动
            rospy.loginfo("执行规划的路径...")
            self.arm.execute(traj)
            rospy.sleep(1)
    
            # 设置第二个目标位姿
            target_pose.pose.position.x = 0.35
            target_pose.pose.position.y = -0.15
            target_pose.pose.position.z = 0.25
            target_pose.pose.orientation.x = 0.0
            target_pose.pose.orientation.y = 0.707
            target_pose.pose.orientation.z = 0.0
            target_pose.pose.orientation.w = 0.707
            
            rospy.loginfo("设置第二个目标位姿: 位置(%.3f, %.3f, %.3f) 姿态(%.3f, %.3f, %.3f, %.3f)", 
                       target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
                       target_pose.pose.orientation.x, target_pose.pose.orientation.y,
                       target_pose.pose.orientation.z, target_pose.pose.orientation.w)
                       
            # 设置目标位姿
            self.arm.set_pose_target(target_pose, self.end_effector_link)
            
            # 规划新路径
            rospy.loginfo("正在规划路径...")
            traj = self.arm.plan()
            
            # 执行新路径
            rospy.loginfo("执行规划的路径...")
            self.arm.execute(traj)
            rospy.sleep(1)
    
            # 控制机械臂回到初始化位置
            rospy.loginfo("返回home位置")
            self.arm.set_named_target('home')
            self.arm.go()
            
            return True
            
        except Exception as e:
            rospy.logerr("运动规划或执行出错: %s", e)
            return False
            
    def shutdown(self):
        """关闭节点"""
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("IK演示已完成")

if __name__ == "__main__":
    try:
        demo = IkDemo()
        success = demo.run_demo()
        demo.shutdown()
        
        if success:
            rospy.loginfo("逆向运动学演示完成")
        else:
            rospy.logerr("逆向运动学演示失败")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("用户中断演示") 