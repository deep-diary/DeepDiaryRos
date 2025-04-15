#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class CartesianDemo:
    """笛卡尔路径演示"""
    
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('cartesian_demo', anonymous=True)

        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = MoveGroupCommander('DeepArm')
        
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
        
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.3)
        self.arm.set_max_velocity_scaling_factor(0.3)
        
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
        rospy.loginfo("末端执行器: %s", self.end_effector_link)
        
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
            # 获取当前位置作为路径规划的起始点
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose
            
            # 初始化路点列表
            waypoints = []
            
            # 将初始位姿加入路点列表
            waypoints.append(start_pose)
            
            # 创建路点（相对于起始点）
            waypoint1 = deepcopy(start_pose)
            waypoint1.position.z -= 0.08  # 下移8cm
            waypoints.append(deepcopy(waypoint1))
            
            waypoint2 = deepcopy(waypoint1)
            waypoint2.position.x += 0.12  # 前移12cm
            waypoints.append(deepcopy(waypoint2))
            
            waypoint3 = deepcopy(waypoint2)
            waypoint3.position.y += 0.12  # 右移12cm
            waypoints.append(deepcopy(waypoint3))
            
            waypoint4 = deepcopy(waypoint3)
            waypoint4.position.y -= 0.24  # 左移24cm
            waypoints.append(deepcopy(waypoint4))
            
            waypoint5 = deepcopy(waypoint4)
            waypoint5.position.x -= 0.12  # 后移12cm
            waypoints.append(deepcopy(waypoint5))
            
            waypoint6 = deepcopy(waypoint5)
            waypoint6.position.z += 0.08  # 上移8cm
            waypoints.append(deepcopy(waypoint6))
            
            # 输出路点信息
            for i, point in enumerate(waypoints):
                rospy.loginfo("路点 %d: 位置(%.3f, %.3f, %.3f)", 
                           i, point.position.x, point.position.y, point.position.z)
                
            # 设置最大尝试规划次数
            maxtries = 100
            attempts = 0
            
            # 设置机器臂当前的状态作为运动初始状态
            self.arm.set_start_state_to_current_state()
         
            # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
            rospy.loginfo("尝试规划笛卡尔路径...")
            fraction = 0.0
            
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,      # waypoint poses，路点列表
                                        0.01,           # eef_step，终端步进值
                                        0.0,            # jump_threshold，跳跃阈值
                                        True)           # avoid_collisions，避障规划
                
                # 尝试次数累加
                attempts += 1
                
                # 打印运动规划进程
                if attempts % 10 == 0:
                    rospy.loginfo("运行 %d 次规划后, 路径覆盖率为: %.2f%%", attempts, fraction * 100.0)
                         
            # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
            if fraction == 1.0:
                rospy.loginfo("规划成功! 路径覆盖率: 100%")
                rospy.loginfo("开始执行路径...")
                self.arm.execute(plan)
                rospy.sleep(1)
                rospy.loginfo("路径执行完成")
                result = True
            else:
                rospy.logwarn("路径规划失败: 在尝试 %d 次后，覆盖率为 %.2f%%", attempts, fraction * 100.0)
                result = False
    
            # 控制机械臂回到初始化位置
            rospy.loginfo("返回home位置")
            self.arm.set_named_target('home')
            self.arm.go()
            
            return result
            
        except Exception as e:
            rospy.logerr("运动规划或执行出错: %s", e)
            return False
            
    def shutdown(self):
        """关闭节点"""
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("Cartesian演示已完成")

if __name__ == "__main__":
    try:
        demo = CartesianDemo()
        success = demo.run_demo()
        demo.shutdown()
        
        if success:
            rospy.loginfo("笛卡尔路径演示完成")
        else:
            rospy.logerr("笛卡尔路径演示失败")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("用户中断演示") 