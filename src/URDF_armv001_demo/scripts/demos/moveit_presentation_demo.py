#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose


class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_presentation_demo_deeparm')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('DeepArm')
                
        # 获取终端link的名称
        end_effector_link = self.arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.002)
        self.arm.set_goal_orientation_tolerance(0.02)
       
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)

        self.execute_trajectory('home')
        
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

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def execute_trajectory(self, target_name):
        """规划并执行轨迹，打印相关信息"""
        rospy.loginfo("Planning trajectory to %s position...", target_name)
        
        # 设置目标位置
        self.arm.set_named_target(target_name)
        
        # 进行运动规划
        plan = self.arm.plan()
        
        # 获取规划结果
        if len(plan.joint_trajectory.points) > 0:
            # 获取轨迹的总时间
            trajectory_time = plan.joint_trajectory.points[-1].time_from_start.to_sec()
            num_points = len(plan.joint_trajectory.points)
            rospy.loginfo("Trajectory planning succeeded!")
            rospy.loginfo("Number of points in trajectory: %d", num_points)
            rospy.loginfo("Estimated execution time: %.2f seconds", trajectory_time)
            
            # 执行轨迹
            rospy.loginfo("Executing trajectory...")
            self.arm.execute(plan)
            
            # 等待执行完成
            rospy.sleep(trajectory_time + 0.5)  # 额外添加0.5秒以确保执行完成
            self.arm.stop()
            rospy.loginfo("Trajectory execution completed!")
        else:
            rospy.logerr("Trajectory planning failed!")
            return False
        
        return True

if __name__ == "__main__":
    MoveItIkDemo()

    
    
