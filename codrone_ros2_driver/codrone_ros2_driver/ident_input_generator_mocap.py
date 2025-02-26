# Copyright (c) 2025 Junya Yamauchi
# This software is released under the MIT License, see LICENSE.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# from std_msgs.msg import Empty, UInt8, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Joy
from codrone_msgs.msg import CoDroneStatus
import numpy as np
import os, csv, math
from datetime import datetime
from tf_transformations import euler_from_quaternion


class IdentInputGenerator(Node):
    def __init__(self) -> None:
        super().__init__('ident_input_generator')

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

           # joy setting
        joy_parameters = [
            ('button_s', 9),
        ]
        self.declare_parameters('', joy_parameters)
        self._BUTTON_S = self.get_parameter('button_s').value

        self.declare_parameter('robot_dev', 'CoDrone1')  # デフォルト値
        self.robot_dev = self.get_parameter('robot_dev').value

        self._ident_direction = 'x'  # direction a drone moves in the world frame
        self._input_power = [66, 78] # larger order
        self._each_exp_num = 5
        _default_x = 0.0
        _default_y = 1.0
        _default_z = 0.6
        _default_theta = 0.0
        _direction_goal_pos = [-0.8, 0.8] # the 1st element must be smaller than the 2nd element
        # _direction_goal_pos = [-np.pi/2, np.pi/2] # the 1st element must be smaller than the 2nd element
        if self._ident_direction == 'x':
            self._goal_pos = np.array([[_direction_goal_pos[0], _default_y, _default_z, _default_theta], [_direction_goal_pos[1], _default_y, _default_z, _default_theta]])
        elif self._ident_direction == 'y':
            self._goal_pos = np.array([[_default_x, _direction_goal_pos[0], _default_z, _default_theta], [_default_x, _direction_goal_pos[1], _default_z, _default_theta]])
        elif self._ident_direction == 'z':
            self._goal_pos = np.array([[_default_x, _default_y, _direction_goal_pos[0], _default_theta], [_default_x, _default_y, _direction_goal_pos[1], _default_theta]])
        elif self._ident_direction == 'theta':
            self._goal_pos = np.array([[_default_x, _default_y, _default_z, _direction_goal_pos[0]], [_default_x, _default_y, _default_z, _direction_goal_pos[1]]])
        else:
            self.get_logger().error(f"Invalid setting: '{self._ident_direction}'.")
            rclpy.shutdown()

        self._input_num = len(self._input_power)
        self._Kp = np.array([60.0, 60.0, 100.0, 60.0])
        self._Ki = np.array([30.0, 30.0, 50.0, 30.0])
        self._initialize_dwell_time_condition = 5.0 

        self.ident_started = False
        self.ident_finished = False
        self.ident_vel = Twist()
        self.codrone_joy = Joy()
        self.codrone_status = CoDroneStatus()
        self.codrone_pose = PoseStamped()
        self.pos = [0., 0., 0.]
        self.theta = 0.0
        self.which_exp = 0
        self.initialize_error_threshold = 0.27

        # subscriber
        self.sub_codrone_joy = self.create_subscription(Joy, 'codrone_joy', self.callback_codrone_joy, qos_profile)
        self.sub_codrone_status = self.create_subscription(CoDroneStatus, 'codrone_status', self.callback_codrone_status, qos_profile)
        self.sub_codrone_pose = self.create_subscription(PoseStamped, '/' + self.robot_dev.lower() + '/pose', self.callback_codrone_pose, qos_profile_mocap)

        # publisher
        self.pub_ident_vel = self.create_publisher(Twist, 'ident_vel', qos_profile)

    def create_output_directory(self):
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.output_directory = '/home/cheetar/codrone_ws/src/codrone_ros2_driver/data/' + self._ident_direction + '_' + timestamp
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

        if self.codrone_joy.buttons[self._BUTTON_S]:
            self.ident_started = True

    def callback_codrone_status(self, msg):
        self.codrone_status = msg

    def callback_codrone_pose(self, msg):
        self.codrone_pose = msg
        self.pos = [self.codrone_pose.pose.position.x, self.codrone_pose.pose.position.y, self.codrone_pose.pose.position.z]
        self.theta = self.get_euler_from_quaternion(self.codrone_pose.pose.orientation)[2]

    def generate_ident_input(self):
        for i in range(self._input_num):
            for j in range(self._each_exp_num):
                k = self.which_exp
                self.get_logger().info(f"k: '{k}'")  
                start_initialize_time = self.get_seconds()
                previous_time = start_initialize_time
                initialize_dwell_time = 0.0
                self.error_integral = [0.]*4
                while initialize_dwell_time < self._initialize_dwell_time_condition:
                    rclpy.spin_once(self)
                    self.get_logger().info(f"Initializing: '{self._input_power[i],  j+1}'")  
                    current_time = self.get_seconds()
                    node_period = current_time - previous_time
                    self.move_to_initial_position(k, node_period) 
                    # initialize_error = np.linalg.norm(state - self._goal_pos[k])
                    initialize_error = np.linalg.norm(self.pos - self._goal_pos[k, 0:3]) + (self.theta - self._goal_pos[k, 3])**2
                    # initialize_error = 0.0*np.linalg.norm(self.pos - self._goal_pos[k, 0:3]) + 5*(self.theta - self._goal_pos[k, 3])**2
                    if initialize_error <= self.initialize_error_threshold:
                        initialize_dwell_time += node_period
                    else:
                        initialize_dwell_time = 0.0
                    previous_time = current_time

                ident_start_time = self.get_seconds()
                # previous_time = ident_start_time
                # ident_elapsed_time = 0.0
                # ident_complete = False
                save_data = []
                while not self.is_one_ident_complete(k):
                    rclpy.spin_once(self)
                    self.get_logger().info(f"Moving to the goal pos: '{self._input_power[i],  j+1}'")  
                    current_time = self.get_seconds()
                    ident_elapsed_time = current_time - ident_start_time
                    self.move_to_goal_position(i,k)

                    data = [ident_elapsed_time] + self.pos + [self.theta]
                    save_data.append(data)
                file_name = str(abs(self._input_power[i])) + '_' + str(j+1) + '.csv'
                self.save_data_to_csv(file_name, save_data)
                
                if self.which_exp == 0:
                    self.which_exp = 1
                elif self.which_exp == 1:
                    self.which_exp = 0

    def move_to_initial_position(self, k, node_period):
        error = self._goal_pos[k] - np.append(self.pos, self.theta)
        self.error_integral += error*node_period
        self.ident_vel.linear.x = self._Kp[0]*error[0] + self._Ki[0]*self.error_integral[0]
        self.ident_vel.linear.y = self._Kp[1]*error[1] + self._Ki[1]*self.error_integral[1]
        self.ident_vel.linear.z = self._Kp[2]*error[2] + self._Ki[2]*self.error_integral[2]
        self.ident_vel.angular.z = self._Kp[3]*error[3] + self._Ki[3]*self.error_integral[3]

        self.pub_ident_vel.publish(self.ident_vel)

    def move_to_goal_position(self, i, j):
        if self._ident_direction == 'x':
            self.ident_vel.linear.x = float((-2*self.which_exp+1)*self._input_power[i])
            self.ident_vel.linear.y = self._Kp[1]*(self._goal_pos[j,1] - self.pos[1])
            self.ident_vel.linear.z = self._Kp[2]*(self._goal_pos[j,2] - self.pos[2])
            self.ident_vel.angular.z = self._Kp[3]*(self._goal_pos[j,3] - self.theta)
        elif self._ident_direction == 'y':
            self.ident_vel.linear.x = self._Kp[0]*(self._goal_pos[j,0] - self.pos[0])
            self.ident_vel.linear.y = float((-2*self.which_exp+1)*self._input_power[i])
            self.ident_vel.linear.z = self._Kp[2]*(self._goal_pos[j,2] - self.pos[2])
            self.ident_vel.angular.z = self._Kp[3]*(self._goal_pos[j,3] - self.theta)
        elif self._ident_direction == 'z':
            self.ident_vel.linear.x = self._Kp[0]*(self._goal_pos[j,0] - self.pos[0])
            self.ident_vel.linear.y = self._Kp[1]*(self._goal_pos[j,1] - self.pos[1])
            self.ident_vel.linear.z = float((-2*self.which_exp+1)*self._input_power[i])
            self.ident_vel.angular.z = self._Kp[3]*(self._goal_pos[j,3] - self.theta)
        elif self._ident_direction == 'theta':
            # self.ident_vel.linear.x = self._Kp[0]*(self._goal_pos[j,0] - self.pos[0])
            # self.ident_vel.linear.y = self._Kp[1]*(self._goal_pos[j,1] - self.pos[1])
            # self.ident_vel.linear.z = self._Kp[2]*(self._goal_pos[j,2] - self.pos[2])
            self.ident_vel.angular.z = float((-2*self.which_exp+1)*self._input_power[i])

        self.pub_ident_vel.publish(self.ident_vel)

    def is_one_ident_complete(self, j):
        if self._ident_direction == 'x':
            ident_error = np.sign(self._goal_pos[j-1,0] - self._goal_pos[j,0])*(self.pos[0] - self._goal_pos[j-1,0])
        elif self._ident_direction == 'y':
            ident_error = np.sign(self._goal_pos[j-1,1] - self._goal_pos[j,1])*(self.pos[1] - self._goal_pos[j-1,1])
        elif self._ident_direction == 'z':
            ident_error = np.sign(self._goal_pos[j-1,2] - self._goal_pos[j,2])*(self.pos[2] - self._goal_pos[j-1,2])
        elif self._ident_direction == 'theta':
            ident_error = np.sign(self._goal_pos[j-1,3] - self._goal_pos[j,3])*(self.theta - self._goal_pos[j-1,3])
       
        if ident_error >= 0:
            ident_complete = True
        else:
            ident_complete = False
        return ident_complete

    def get_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1_000_000_000
    
    def get_euler_from_quaternion(self, quaternion):
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        return euler_from_quaternion(quaternion_list)

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
                self.save_input_to_csv('ident_input.csv', self._input_power)
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