import rospy
from abc import ABC, abstractmethod
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionFeedback
from trajectory_msgs.msg import JointTrajectoryPoint

class RobotBase(ABC):
    """机器人基础抽象类"""
    
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.goal_sub = rospy.Subscriber(
            f"/{robot_name}/arm_joint_controller/follow_joint_trajectory/goal",
            FollowJointTrajectoryActionGoal,
            self.goal_callback
        )
        self.feedback_pub = rospy.Publisher(
            f"/{robot_name}/arm_joint_controller/follow_joint_trajectory/feedback",
            FollowJointTrajectoryActionFeedback,
            queue_size=10
        )

    @abstractmethod
    def write_goal(self, cmd_list):
        """发送轨迹点到机器人
        Args:
            cmd_list: 包含时间戳、位置、速度的轨迹点列表
        """
        pass

    @abstractmethod
    def read_feedback(self, data):
        """处理机器人反馈数据
        Args:
            data: 串口接收到的数据
        """
        pass

    def goal_callback(self, msg):
        """处理轨迹目标回调"""
        rospy.loginfo(f"Received trajectory for {self.robot_name}")
        
        # 提取轨迹点列表
        trajectory_points = []
        for point in msg.goal.trajectory.points:
            trajectory_point = {
                'time_from_start': point.time_from_start,
                'positions': point.positions,
                'velocities': point.velocities
            }
            trajectory_points.append(trajectory_point)
        
        # 按时间戳排序
        trajectory_points.sort(key=lambda x: x['time_from_start'].to_sec())
        
        # 发送轨迹点
        self.write_goal(trajectory_points)

    def publish_feedback(self, joint_states):
        """发布关节状态反馈
        Args:
            joint_states: 包含位置、速度、加速度的关节状态字典
        """
        feedback_msg = FollowJointTrajectoryActionFeedback()
        feedback_msg.header.stamp = rospy.Time.now()
        
        # 设置关节状态
        point = JointTrajectoryPoint()
        point.positions = joint_states.get('positions', [])
        point.velocities = joint_states.get('velocities', [])
        point.accelerations = joint_states.get('accelerations', [])
        point.effort = joint_states.get('effort', [])
        
        feedback_msg.feedback.desired = point
        feedback_msg.feedback.actual = point
        feedback_msg.feedback.error = JointTrajectoryPoint()
        
        self.feedback_pub.publish(feedback_msg) 
