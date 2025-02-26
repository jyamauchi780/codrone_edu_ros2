# Copyright (c) 2025 Junya Yamauchi
# This software is released under the MIT License, see LICENSE.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# from std_msgs.msg import Empty, UInt8, Bool
from geometry_msgs.msg import Twist, PoseStamped
from codrone_msgs.msg import CoDroneStatus
import numpy as np
import math
from tf_transformations import euler_from_quaternion

class VelocityObserver(Node):
    def __init__(self) -> None:
        super().__init__('velocity_observer')

        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.SYSTEM_DEFAULT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        qos_profile_mocap = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.SYSTEM_DEFAULT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self.declare_parameter('robot_dev', 'CoDrone1')  # デフォルト値
        self.robot_dev = self.get_parameter('robot_dev').value

        self._timer_period = 0.01
        self.pos = np.zeros(3)
        self.theta = 0.0
        self.pose = np.zeros(4)
        self.obs_state = np.zeros(4)
        self.est_vel = Twist()
        self.status = CoDroneStatus()
        self.cmd_vel = Twist()
        self.obs_eigenvalue = np.array([-4, -4, -7, -12])
        self.timeconstant = np.array([0.5, 0.5, 0.2, 0.1])
        self.A = np.array(self.obs_eigenvalue)
        B = []
        for i in range(4):
            b = np.array([-self.obs_eigenvalue[i]*(self.obs_eigenvalue[i] + 1/self.timeconstant[i]), 1/self.timeconstant[i]])
            B.append(b)
        self.B = np.array(B)
        self.C = np.ones(4)
        D = []
        for i in range(4):
            d = np.array([-(self.obs_eigenvalue[i] + 1/self.timeconstant[i]), 0])
            D.append(d)
        self.D = np.array(D)    

        # subscriber
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.callback_cmd_vel, qos_profile)
        self.sub_status = self.create_subscription(CoDroneStatus, 'codrone_status', self.callback_status, qos_profile)
        self.sub_pose = self.create_subscription(PoseStamped, '/' + self.robot_dev.lower() + '/pose', self.callback_pose, qos_profile_mocap)

        # publisher
        self.pub_est_vel = self.create_publisher(Twist, 'est_vel', qos_profile)

        # Create a timer to publish control commands
        self.timer = self.create_timer(self._timer_period, self.callback_timer) 

    def callback_cmd_vel(self, msg):
        self.cmd_vel = msg

    def callback_status(self, msg):
        self.status = msg

    def callback_pose(self, msg):
        self.pose = msg
        self.pos = np.array([self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z])
        self.theta = self.get_euler_from_quaternion(self.pose.pose.orientation)[2]
        self.pose = np.append(self.pos, self.theta)

    def callback_timer(self):
        if self.status.flight:
            obs_state_next = np.zeros(4)
            est_vel = np.zeros(4)
            current_time = self.get_seconds()
            dt = current_time - self.previous_time
            input = [self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.linear.z, self.cmd_vel.angular.z]
            R = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
            pos = self.pos
            pos[:2] = R.T @ self.pos[:2]
            pose = np.append(pos, self.theta)
            for i in range(4):
                obs_state_next[i] = self.obs_state[i] + (self.A[i]*self.obs_state[i] + self.B[i] @ np.array([pose[i], input[i]]))*dt
                est_vel[i] = self.C[i]*self.obs_state[i] + self.D[i] @ np.array([pose[i], input[i]])
            # R = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
            # est_vel[:2] = R.T @ est_vel[:2]
            self.est_vel.linear.x = est_vel[0]
            self.est_vel.linear.y = est_vel[1]
            self.est_vel.linear.z = est_vel[2]
            self.est_vel.angular.z = est_vel[3]
            self.pub_est_vel.publish(self.est_vel)
            
            self.obs_state = obs_state_next
            self.previous_time = current_time
        else:
            R = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
            pos = self.pos
            pos[:2] = R.T @ self.pos[:2]
            pose = np.append(pos, self.theta)
            self.obs_state = (self.obs_eigenvalue + 1/self.timeconstant)*pose  # set the initial state of the observer
            self.previous_time = self.get_seconds()

    def get_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1_000_000_000

    def get_euler_from_quaternion(self, quaternion):
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        return euler_from_quaternion(quaternion_list)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityObserver()
    rclpy.spin(node)
    node.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()