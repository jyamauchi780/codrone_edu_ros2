# Copyright (c) 2025 Junya Yamauchi
# This software is released under the MIT License, see LICENSE.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# from std_msgs.msg import Empty, UInt8, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from codrone_msgs.msg import CoDroneOdom, CoDroneStatus
import numpy as np
import os
import csv
from datetime import datetime


class IdentInputGenerator(Node):
    def __init__(self) -> None:
        super().__init__('ident_input_generator')

        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.SYSTEM_DEFAULT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

           # joy setting
        joy_parameters = [
            ('button_h', 8),
        ]
        self.declare_parameters('', joy_parameters)
        self._BUTTON_H = self.get_parameter('button_h').value

        self._ident_direction = 'x'  # direction a drone moves in the world frame
        self._input_power = [20, 30] # larger order
        self._each_exp_num = 2
        direction_goal_pos = [-1.0, 1.0] # the 1st element must be smaller than the 2nd element
        if self._ident_direction == 'x':
            self._goal_pos = np.array([[direction_goal_pos[0], 0.0, 0.9, 0.0], [direction_goal_pos[1], 0.0, 0.9, 0.0]])
        elif self._ident_direction == 'y':
            self._goal_pos = np.array([[0.0, direction_goal_pos[0], 0.9, 0.0], [0.0, direction_goal_pos[1], 0.9, 0.0]])
        elif self._ident_direction == 'z':
            self._goal_pos = np.array([[0.0, 0.0, direction_goal_pos[0], 0.0], [0.0, 0.0, direction_goal_pos[1], 0.0]])
        elif self._ident_direction == 'theta':
            self._goal_pos = np.array([[0.0, 0.0, 0.9, direction_goal_pos[0]], [0.0, 0.0, 0.9, direction_goal_pos[1]]])
        else:
            self.get_logger().error(f"Invalid setting: '{self._ident_direction}'.")
            rclpy.shutdown()

        self._exp_num = len(self._input_power)
        multipliers = [1 if i % 2 == 0 else -1 for i in range(self._each_exp_num )]
        self._ident_input = [x * y for x in self._input_power for y in multipliers]
        self._fb_gain = 0.5
        self._initialize_dwell_time_condition = 2.0 

        self.ident_started = False
        self.ident_finished = False
        self.ident_vel = Twist()
        self.codrone_joy = Joy()
        self.codrone_odom = CoDroneOdom()
        self.codrone_status = CoDroneStatus()

        # subscriber
        self.sub_codrone_joy = self.create_subscription(Joy, 'codrone_joy', self.callback_codrone_joy, qos_profile)
        self.sub_codrone_odom = self.create_subscription(CoDroneOdom, 'codrone_odom', self.callback_codrone_odom, qos_profile)
        self.sub_codrone_status = self.create_subscription(CoDroneStatus, 'codrone_status', self.callback_codrone_status, qos_profile)

        # publisher
        self.pub_ident_vel = self.create_publisher(Twist, 'ident_vel', qos_profile)

    def create_output_directory(self):
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.output_directory = 'src/codrone_ros2_driver/data/' + self._ident_direction + '_' + timestamp
        # self.output_directory = os.path.join(folder_name, timestamp)
        os.makedirs(self.output_directory, exist_ok=True)

    def save_data_to_csv(self, file_name, save_data):
        csv_file_path = os.path.join(self.output_directory, file_name)
        with open(csv_file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'theta']) 
            writer.writerows(save_data) 

    def save_input_to_csv(self, file_name, save_data):
        csv_file_path = os.path.join(self.output_directory, file_name)
        with open(csv_file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(save_data) 
            # writer.writerows(save_data) 

    def callback_codrone_joy(self, msg):
        self.codrone_joy = msg

        if self.codrone_joy.buttons[self._BUTTON_H]:
            self.ident_started = True

    def callback_codrone_odom(self, msg):
        self.codrone_odom = msg

    def callback_codrone_status(self, msg):
        self.codrone_status = msg

    def generate_ident_input(self):
        for i in range(self._exp_num):
            for j in range(self._each_exp_num):
                start_initialize_time = self.get_seconds()
                previous_time = start_initialize_time
                initialize_dwell_time = 0.0
                while initialize_dwell_time < self._initialize_dwell_time_condition:
                    rclpy.spin_once(self)
                    self.get_logger().info(f"Initializing: '{self.codrone_odom.position.z}'")  
                    current_time = self.get_seconds()
                    node_period = current_time - previous_time
                    self.move_to_initial_position(j) 
                    state = np.array([self.codrone_odom.position.x, self.codrone_odom.position.y, self.codrone_odom.position.z, self.codrone_odom.orientation.z])
                    initialize_error = np.linalg.norm(state-self._goal_pos[j])
                    if initialize_error <= 0.5:
                        initialize_dwell_time += initialize_dwell_time + node_period
                    else:
                        initialize_dwell_time = 0.0
                    previous_time = current_time

                ident_start_time = self.get_seconds()
                # previous_time = start_ident_time
                # ident_elapsed_time = 0.0
                # ident_complete = False
                save_data = []
                while not self.is_one_ident_complete(j):
                    rclpy.spin_once(self)
                    self.get_logger().info(f"Moving to the goal pos: '{self.codrone_odom.position.z}'")  
                    current_time = self.get_seconds()
                    ident_elapsed_time = current_time - ident_start_time
                    self.move_to_goal_position(i,j)

                    data = [ident_elapsed_time, self.codrone_odom.position.x, self.codrone_odom.position.y, self.codrone_odom.position.z, self.codrone_odom.orientation.z]
                    save_data.append(data)
                file_name = str(abs(self._ident_input[2*i+j])) + '_' + str(j+1) + '_' + '.csv'
                self.save_data_to_csv(file_name, save_data)

    def move_to_initial_position(self, j):
        self.ident_vel.linear.x = self._fb_gain*(self._goal_pos[j,0] - self.codrone_odom.position.x)
        self.ident_vel.linear.y = self._fb_gain*(self._goal_pos[j,1] - self.codrone_odom.position.y)
        self.ident_vel.linear.z = self._fb_gain*(self._goal_pos[j,2] - self.codrone_odom.position.z)
        self.ident_vel.angular.z = self._fb_gain*(self._goal_pos[j,3] - self.codrone_odom.orientation.z)

        self.pub_ident_vel.publish(self.ident_vel)

    def move_to_goal_position(self, i, j):
        if self._ident_direction == 'x':
            self.ident_vel.linear.x = float(self._ident_input[i*2+j])
            self.ident_vel.linear.y = self._fb_gain*(self._goal_pos[j,1] - self.codrone_odom.position.y)
            self.ident_vel.linear.z = self._fb_gain*(self._goal_pos[j,2] - self.codrone_odom.position.z)
            self.ident_vel.angular.z = self._fb_gain*(self._goal_pos[j,3] - self.codrone_odom.orientation.z)
        elif self._ident_direction == 'y':
            self.ident_vel.linear.x = self._fb_gain*(self._goal_pos[j,0] - self.codrone_odom.position.x)
            self.ident_vel.linear.y = float(self._ident_input[i*2+j])
            self.ident_vel.linear.z = self._fb_gain*(self._goal_pos[j,2] - self.codrone_odom.position.z)
            self.ident_vel.angular.z = self._fb_gain*(self._goal_pos[j,3] - self.codrone_odom.orientation.z)
        elif self._ident_direction == 'z':
            self.ident_vel.linear.x = self._fb_gain*(self._goal_pos[j,0] - self.codrone_odom.position.x)
            self.ident_vel.linear.y = self._fb_gain*(self._goal_pos[j,1] - self.codrone_odom.position.y)
            self.ident_vel.linear.z = float(self._ident_input[i*2+j])
            self.ident_vel.angular.z = self._fb_gain*(self._goal_pos[j,3] - self.codrone_odom.orientation.z)
        elif self._ident_direction == 'theta':
            self.ident_vel.linear.x = self._fb_gain*(self._goal_pos[j,0] - self.codrone_odom.position.x)
            self.ident_vel.linear.y = self._fb_gain*(self._goal_pos[j,1] - self.codrone_odom.position.y)
            self.ident_vel.linear.z = self._fb_gain*(self._goal_pos[j,2] - self.codrone_odom.position.z)
            self.ident_vel.angular.z = float(self._ident_input[i*2+j])

        self.pub_ident_vel.publish(self.ident_vel)

    def is_one_ident_complete(self, j):
        if self._ident_direction == 'x':
            ident_error = np.sign(self._goal_pos[j-1,0] - self._goal_pos[j,0])*(self.codrone_odom.position.x - self._goal_pos[j-1,0])
        elif self._ident_direction == 'y':
            ident_error = np.sign(self._goal_pos[j-1,1] - self._goal_pos[j,1])*(self.codrone_odom.position.y - self._goal_pos[j-1,1])
        elif self._ident_direction == 'z':
            ident_error = np.sign(self._goal_pos[j-1,2] - self._goal_pos[j,2])*(self.codrone_odom.position.z - self._goal_pos[j-1,2])
        elif self._ident_direction == 'theta':
            ident_error = np.sign(self._goal_pos[j-1,3] - self._goal_pos[j,3])*(self.codrone_odom.orientation.z - self._goal_pos[j-1,3])
       
        if ident_error >= 0:
            ident_complete = True
        else:
            ident_complete = False
        return ident_complete

    def get_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1_000_000_000

    def loop(self):
        while not self.ident_finished:
            rclpy.spin_once(self)  
            if self.ident_started and self.codrone_status.flight:
                self.create_output_directory()
                self.get_logger().info('Experiment Started!!!')
                self.generate_ident_input()
                self.ident_finished = True
                self.get_logger().info('Experiment Finished!!!')
                self.ident_vel.linear.x = 0.0
                self.ident_vel.linear.y = 0.0
                self.ident_vel.linear.z = 0.0
                self.ident_vel.angular.z = 0.0
                self.pub_ident_vel.publish(self.ident_vel)
                self.save_input_to_csv('ident_input.csv', self._ident_input)
            else:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = IdentInputGenerator()
    try:
        node.loop()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()