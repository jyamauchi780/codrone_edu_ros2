# Copyright (c) 2025 Junya Yamauchi
# This software is released under the MIT License, see LICENSE.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from codrone_edu.drone import *

# from std_msgs.msg import Empty, UInt8, Bool
from geometry_msgs.msg import Twist
from codrone_msgs.msg import CoDroneStatus, CoDroneSensor, CoDroneOdom
from sensor_msgs.msg import Joy
import numpy as np


class CodroneNode(Node, Drone):
    def __init__(self) -> None:
        Node.__init__(self, 'codrone_driver')
        Drone.__init__(self)

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

        self.declare_parameter('robot_dev', 'CoDrone1')  # デフォルト値
        self.robot_dev = self.get_parameter('robot_dev').value
        portname = '/dev/' + self.robot_dev

        self._POWER_LINEAR = 0.2
        self._POWER_ANGULAR = 0.2
        self._POWER_THROTTLE = 0.2
        self._timer_period = 0.02

        self._start_time = self.get_seconds()
        self.previous_time = self._start_time

        self.codrone_joy = Joy()
        self.ident_vel = Twist()
        self.codrone_joy.axes =  [0.]*4
        self.codrone_joy.buttons = [0]*11
        self.status = CoDroneStatus()
        self.status.flight = False
        self.status.autonomous_flight_mode = True
        self.sensor = CoDroneSensor()
        self.odom = CoDroneOdom()
        self.previous_vel = [0., 0., 0.]

        # subscriber
        self.sub_ident_vel = self.create_subscription(Twist, 'ident_vel', self.callback_ident_vel, qos_profile)

        # publisher
        self.pub_codrone_joy = self.create_publisher(Joy, 'codrone_joy', qos_profile)
        self.pub_codrone_status = self.create_publisher(CoDroneStatus, 'codrone_status', qos_profile)
        self.pub_codrone_sensor = self.create_publisher(CoDroneSensor, 'codrone_sensor', qos_profile)
        self.pub_codrone_odom = self.create_publisher(CoDroneOdom, 'codrone_odom', qos_profile)

        # Create a timer to publish control commands
        self.timer = self.create_timer(self._timer_period, self.callback_timer) 

        self.pair(portname=portname) # paring codrone 
        self.drone_LED_off()

    def callback_ident_vel(self, msg):
        self.ident_vel = msg

    def callback_timer(self):
        elapsed_time = self.get_seconds() - self._start_time 
        self.node_period = elapsed_time - self.previous_time
        # self.get_logger().info(f"elapsed time: '{elapsed_time}'")
        # self.get_logger().info(f"dt: '{self.node_period}'")
        self.previous_time = elapsed_time

        self.publish_codrone_joy()
        self.publish_codrone_status()
        # self.publish_codrone_sensor()
        # self.publish_codrone_odom()

        if self.codrone_joy.buttons[self._BUTTON_UP] and self.status.flight==False:
            self.get_logger().info('Take off!!!')
            self.takeoff()
            time.sleep(1)

        if self.codrone_joy.buttons[self._BUTTON_DOWN] and self.status.flight==True:
            self.get_logger().info('Landing!!!')
            self.land()
            time.sleep(1)

        if self.codrone_joy.buttons[self._BUTTON_H]:
            self.get_logger().info('Emergency stop!!!')
            self.emergency_stop()
            time.sleep(1)
        
        if self.status.flight:
            if self.status.autonomous_flight_mode:
                self.set_roll(int(-self.ident_vel.linear.y))
                self.set_pitch(int(self.ident_vel.linear.x))
                self.set_yaw(int(self.ident_vel.angular.z))
                self.set_throttle(int(self.ident_vel.linear.z))
                self.move(self._timer_period)
            else:
                linear_x = self._POWER_LINEAR*self.codrone_joy.axes[self._AXIS_LINEAR_X]
                linear_y = self._POWER_LINEAR*self.codrone_joy.axes[self._AXIS_LINEAR_Y]
                linear_z = self._POWER_THROTTLE*self.codrone_joy.axes[self._AXIS_LINEAR_Z]
                angular_z = -self._POWER_ANGULAR*self.codrone_joy.axes[self._AXIS_ANGULAR_Z]
                self.get_logger().info('%d, %d, %d, %d' % (linear_y, linear_x, angular_z, linear_z))
                self.set_roll(linear_y)
                self.set_pitch(linear_x)
                self.set_yaw(angular_z)
                self.set_throttle(linear_z)
                self.move(self._timer_period)

    def publish_codrone_joy(self):
        self.codrone_joy.axes[self._AXIS_LINEAR_X] = float(self.get_left_joystick_y())
        self.codrone_joy.axes[self._AXIS_LINEAR_Y] = float(self.get_left_joystick_x())
        self.codrone_joy.axes[self._AXIS_LINEAR_Z] = float(self.get_right_joystick_y())
        self.codrone_joy.axes[self._AXIS_ANGULAR_Z] = float(self.get_right_joystick_x())
        self.codrone_joy.buttons[self._BUTTON_L1] = self.l1_pressed()
        self.codrone_joy.buttons[self._BUTTON_R1] = self.r1_pressed()
        self.codrone_joy.buttons[self._BUTTON_L2] = self.l2_pressed()
        self.codrone_joy.buttons[self._BUTTON_R2] = self.r2_pressed()
        self.codrone_joy.buttons[self._BUTTON_LEFT] = self.left_arrow_pressed()
        self.codrone_joy.buttons[self._BUTTON_DOWN] = self.down_arrow_pressed()
        self.codrone_joy.buttons[self._BUTTON_RIGHT] = self.right_arrow_pressed()
        self.codrone_joy.buttons[self._BUTTON_UP] = self.up_arrow_pressed()
        self.codrone_joy.buttons[self._BUTTON_H] = self.h_pressed()
        self.codrone_joy.buttons[self._BUTTON_S] = self.s_pressed()
        self.codrone_joy.buttons[self._BUTTON_P] = self.p_pressed()

        self.pub_codrone_joy.publish(self.codrone_joy)

    def publish_codrone_status(self):
        self.status.frequency = 1/self.node_period

        flight_status = self.get_state_data()
        if flight_status[2] == ModeFlight.Flight:
            self.status.flight = True
        else:
            self.status.flight = False
        self.status.battery = self.get_battery()

        if flight_status[7] == ModeMovement.Ready:
            self.status.movement = "ready"
        elif flight_status[7] == ModeMovement.Hovering:
            self.status.movement = "hovering"
        else:
            self.status.movement = "moving"

        self.pub_codrone_status.publish(self.status)

    def publish_codrone_sensor(self):
        self.sensor.range.front = self.get_front_range(unit='m')
        self.sensor.range.bottom = self.get_bottom_range(unit='m')
        self.sensor.angular.x = float(self.get_angular_speed_x())*np.pi/180
        self.sensor.angular.y = float(self.get_angular_speed_y())*np.pi/180
        self.sensor.angular.z = float(self.get_angular_speed_z())*np.pi/180
        self.sensor.accel.x = float(self.get_accel_x())/10
        self.sensor.accel.y = float(self.get_accel_y())/10
        self.sensor.accel.z = float(self.get_accel_z())/10 - 9.8

        self.pub_codrone_sensor.publish(self.sensor)

    def publish_codrone_odom(self):
        self.odom.position.x = float(self.get_pos_x(unit="m"))
        self.odom.position.y = float(self.get_pos_y(unit="m"))
        self.odom.position.z = float(self.get_pos_z(unit="m"))
        self.odom.orientation.z = float(self.get_angle_z())*np.pi/180
       
        if self.status.movement == "hovering" or self.status.movement == "moving":
            self.calculate_velocity()
        else: 
            self.odom.linear.x = 0.0
            self.odom.linear.y = 0.0
            self.odom.linear.z = 0.0

        self.pub_codrone_odom.publish(self.odom)

    def calculate_velocity(self):
        if self.status.movement == "hovering":
            self.previous_vel = [0., 0., 0.]
        self.odom.linear.x = self.previous_vel[0] + self.node_period * self.sensor.accel.x
        self.odom.linear.y = self.previous_vel[1] + self.node_period * self.sensor.accel.y
        self.odom.linear.z = self.previous_vel[2] + self.node_period * self.sensor.accel.z

        self.previous_vel = [self.odom.linear.x, self.odom.linear.y, self.odom.linear.z]

    def get_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1_000_000_000


def main(args=None):
    rclpy.init(args=args)
    node = CodroneNode()
    rclpy.spin(node)
    node.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()