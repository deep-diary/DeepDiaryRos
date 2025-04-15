#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander

class PresentationDemo:
    """演示机械臂演示路径，循环执行预设位置序列"""
    
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('presentation_demo', anonymous=True)
                
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('DeepArm')
                
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.002)
        self.arm.set_goal_orientation_tolerance(0.02)
       
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)
    
    def run_demo(self):
        """运行演示"""
        try:
            # 首先回到初始位置
            self.execute_trajectory('home')
            
            # 执行三轮演示
            loop_count = 0
            while loop_count < 3:
                loop_count += 1
                rospy.loginfo("--------------------------------Loop %d--------------------------------", loop_count)
                
                rospy.loginfo("Loop %d: Moving source up", loop_count)
                self.execute_trajectory('source_up')
                
                rospy.loginfo("Loop %d: Moving source pick", loop_count)
                self.execute_trajectory('source_pick')
                
                rospy.loginfo("Loop %d: Moving target up", loop_count)
                self.execute_trajectory('target_up')
                
                rospy.loginfo("Loop %d: Moving target pick", loop_count)
                self.execute_trajectory('target_pick')
    
            # 控制机械臂回到初始化位置
            self.execute_trajectory('home')
            
            return True
            
        except Exception as e:
            rospy.logerr("演示执行出错: %s", e)
            return False

    def execute_trajectory(self, target_name):
        """规划并执行轨迹，打印相关信息"""
        rospy.loginfo("规划到 %s 位置的轨迹...", target_name)
        
        # 设置目标位置
        self.arm.set_named_target(target_name)
        
        # 进行运动规划
        plan = self.arm.plan()
        
        # 获取规划结果
        if len(plan.joint_trajectory.points) > 0:
            # 获取轨迹的总时间
            trajectory_time = plan.joint_trajectory.points[-1].time_from_start.to_sec()
            num_points = len(plan.joint_trajectory.points)
            rospy.loginfo("轨迹规划成功!")
            rospy.loginfo("轨迹中的点数: %d", num_points)
            rospy.loginfo("估计执行时间: %.2f 秒", trajectory_time)
            
            # 执行轨迹
            rospy.loginfo("执行轨迹...")
            self.arm.execute(plan)
            
            # 等待执行完成
            rospy.sleep(trajectory_time + 0.5)  # 额外添加0.5秒以确保执行完成
            self.arm.stop()
            rospy.loginfo("轨迹执行完成!")
            return True
        else:
            rospy.logerr("轨迹规划失败!")
            return False
    
    def shutdown(self):
        """关闭节点"""
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("演示已完成")

if __name__ == "__main__":
    try:
        demo = PresentationDemo()
        success = demo.run_demo()
        demo.shutdown()
        
        if success:
            rospy.loginfo("演示路径执行完成")
        else:
            rospy.logerr("演示路径执行失败")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("用户中断演示") 