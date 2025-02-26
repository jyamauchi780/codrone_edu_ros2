# Copyright (c) 2025 Junya Yamauchi
# This software is released under the MIT License, see LICENSE.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# from std_msgs.msg import Empty, UInt8, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from codrone_msgs.msg import CoDroneStatus


class GenerateCmdVelFromCodroneJoy(Node):
    def __init__(self) -> None:
        super().__init__('generate_cmd_vel_from_codrone_joy')

        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.SYSTEM_DEFAULT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        # joy setting
        joy_parameters = [
            ('axis_linear_x', 0),
            ('axis_linear_y', 1),
            ('axis_linear_z', 2),
            ('axis_angular_z', 3),
            ('button_l1', 0),
            ('button_r1', 1),
            ('button_l2', 2),
            ('button_r2', 3),
            ('button_left', 4),
            ('button_down', 5),
            ('button_right', 6),
            ('button_up', 7),
            ('button_h', 8),
            ('button_s', 9),
            ('button_p', 10),
        ]
        self.declare_parameters('', joy_parameters)

        self._AXIS_LINEAR_X = self.get_parameter('axis_linear_x').value
        self._AXIS_LINEAR_Y = self.get_parameter('axis_linear_y').value
        self._AXIS_LINEAR_Z = self.get_parameter('axis_linear_z').value
        self._AXIS_ANGULAR_Z = self.get_parameter('axis_angular_z').value

        self._BUTTON_L1 = self.get_parameter('button_l1').value
        self._BUTTON_R1 = self.get_parameter('button_r1').value
        self._BUTTON_L2 = self.get_parameter('button_l2').value
        self._BUTTON_R2 = self.get_parameter('button_r2').value
        self._BUTTON_LEFT = self.get_parameter('button_left').value
        self._BUTTON_DOWN = self.get_parameter('button_down').value
        self._BUTTON_RIGHT = self.get_parameter('button_right').value
        self._BUTTON_UP = self.get_parameter('button_up').value
        self._BUTTON_H = self.get_parameter('button_h').value
        self._BUTTON_S = self.get_parameter('button_s').value
        self._BUTTON_P = self.get_parameter('button_p').value

        self.joy = Joy()
        self.status = CoDroneStatus()
        self.cmd_vel = Twist()
        self.pow_to_vel = 130

        # subscriber
        self.sub_codrone_status = self.create_subscription(CoDroneStatus, 'codrone_status', self.callback_codrone_status, qos_profile)
        self.sub_codrone_joy = self.create_subscription(Joy, 'codrone_joy', self.callback_codrone_joy, qos_profile)

        # publisher
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', qos_profile)

    def callback_codrone_status(self, msg):
        self.status = msg

    def callback_codrone_joy(self, msg):
        self.joy = msg
        self.cmd_vel.linear.x = self.joy.axes[self._AXIS_LINEAR_Y]/self.pow_to_vel
        self.cmd_vel.linear.y = self.joy.axes[self._AXIS_LINEAR_X]/self.pow_to_vel
        self.cmd_vel.linear.z = self.joy.axes[self._AXIS_LINEAR_Z]/self.pow_to_vel

        if self.status.flight==True:
            self.pub_cmd_vel.publish(self.cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = GenerateCmdVelFromCodroneJoy()
    rclpy.spin(node)
    node.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()