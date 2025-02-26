# Copyright (c) 2025 Junya Yamauchi
# This software is released under the MIT License, see LICENSE.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# from std_msgs.msg import Empty, UInt8, Bool
from geometry_msgs.msg import Twist, PoseStamped
from codrone_msgs.msg import CoDroneStatus
import numpy as np
from collections import namedtuple

class VelocityController(Node):
    def __init__(self) -> None:
        super().__init__('velocity_controller')

        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.SYSTEM_DEFAULT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self._timer_period = 0.01
        self.fb_state = np.zeros((4,2))
        self.ff_state = np.zeros(4)
        self.filter_state = np.zeros(4)
        self.status = CoDroneStatus()
        self.cmd_vel = Twist()
        self.est_vel = Twist()
        self.command_vel = Twist()

        K = namedtuple('K', ['A', 'B', 'C', 'D'])
        # feedback controller
        Kfb_x = K(A=np.array([[-19, 0], [1, 0]]), B=np.array([8, 0]), C=np.array([1.375, 4.4]),     D=0)
        Kfb_y = K(A=np.array([[-19, 0], [1, 0]]), B=np.array([8, 0]), C=np.array([1.375, 4.4]),     D=0)
        Kfb_z = K(A=np.array([[-26, 0], [1, 0]]), B=np.array([8, 0]), C=np.array([1.25, 6.125]),    D=0)
        # Kfb_t = K(A=np.array([[-24, 0], [1, 0]]), B=np.array([8, 0]), C= np.array([1, 9]),       D=0)
        Kfb_t = K(A=np.array([[-51, 0], [1, 0]]), B=np.array([16, 0]), C= np.array([1.375, 12.38]), D=0)
        self.Kfb = [Kfb_x, Kfb_y, Kfb_z, Kfb_t]

        # feedforward controller
        Kff_x = K(A=-3.33, B=2, C=-1.11, D=1.66)
        Kff_y = K(A=-3.33, B=2, C=-1.11, D=1.66)
        Kff_z = K(A=-5.88, B=1, C=-1.04, D=1.17)
        Kff_t = K(A=0, B=0, C=0, D=0)
        # Kff_t = K(A=0, B=0, C=0, D=0)
        self.Kff = [Kff_x, Kff_y, Kff_z, Kff_t]

        # feedforward filter
        F_x = K(A=-3.33, B=2, C=1.667, D=0)
        F_y = K(A=-3.33, B=2, C=1.667, D=0)
        F_z = K(A=-5.88, B=2, C=2.941, D=0)
        F_t = K(A=0, B=0, C=0, D=1)
        self.F = [F_x, F_y, F_z, F_t]

        # subscriber
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.callback_cmd_vel, qos_profile)
        self.sub_est_vel = self.create_subscription(Twist, 'est_vel', self.callback_est_vel, qos_profile)
        self.sub_status = self.create_subscription(CoDroneStatus, 'codrone_status', self.callback_status, qos_profile)

        # publisher
        self.pub_command_vel = self.create_publisher(Twist, 'command_vel', qos_profile)

        # Create a timer to publish control commands
        self.timer = self.create_timer(self._timer_period, self.callback_timer) 

    def callback_cmd_vel(self, msg):
        self.cmd_vel = msg
        # self.get_logger().info(f'cmd_vel: {self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.linear.z, self.cmd_vel.angular.z}')

    def callback_est_vel(self, msg):
        self.est_vel = msg

    def callback_status(self, msg):
        self.status = msg

    def callback_timer(self):
        if not self.status.autonomous_flight_mode:
            self.fb_state = np.zeros((4,2))
            self.ff_state = np.zeros(4)
            self.filter_state = np.zeros(4)

        if self.status.flight:
            fb_state_next = np.zeros((4,2))
            ff_state_next = np.zeros(4)
            filter_state_next = np.zeros(4)
            ufb = np.zeros(4)
            uff = np.zeros(4)
            ref_filtered = np.zeros(4)
            current_time = self.get_seconds()
            dt = current_time - self.previous_time
            cmd_vel = np.array([self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.linear.z, self.cmd_vel.angular.z])
            # cmd_vel = np.zeros(4)
            est_vel = np.array([self.est_vel.linear.x, self.est_vel.linear.y, self.est_vel.linear.z, self.est_vel.angular.z])
            for i in range(4):
                # feedforward filter
                filter_state_next[i] = self.filter_state[i] + (self.F[i].A*self.filter_state[i] + self.F[i].B*cmd_vel[i])*dt
                ref_filtered[i] = self.F[i].C*self.filter_state[i] + self.F[i].D*cmd_vel[i]
                # feedback controller
                error = ref_filtered - est_vel
                fb_state_next[i] = self.fb_state[i] + (self.Kfb[i].A @ self.fb_state[i] + self.Kfb[i].B*error[i])*dt
                ufb[i] = self.Kfb[i].C @ self.fb_state[i] + self.Kfb[i].D*error[i]
                # feedforward controller
                ff_state_next[i] = self.ff_state[i] + (self.Kff[i].A*self.ff_state[i] + self.Kff[i].B*cmd_vel[i])*dt
                uff[i] = self.Kff[i].C*self.ff_state[i] + self.Kff[i].D*cmd_vel[i]

            self.command_vel.linear.x = ufb[0] + uff[0]
            self.command_vel.linear.y = ufb[1] + uff[1]
            self.command_vel.linear.z = ufb[2] + uff[2]
            self.command_vel.angular.z = ufb[3] + uff[3]
            self.pub_command_vel.publish(self.command_vel)
            
            self.fb_state = fb_state_next
            self.ff_state = ff_state_next
            self.filter_state = filter_state_next
            self.previous_time = current_time
        else:
            self.previous_time = self.get_seconds()

    def get_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1_000_000_000


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    node.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()