#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import math
import time

from phidget_stepper_controllers_msgs.action import SpeedController
from phidget_stepper_controllers.speed_controller_server import SpeedControllerServer

from geometry_msgs.msg import Twist, Pose

from navigation_utils.utils import *

class CmdVelSubscriber(Node):

    def __init__(self):
        super().__init__('simple_cmdvel_sub')

        assert self.declare_parameter('left_wheel_stepper', 0)
        assert self.declare_parameter('right_wheel_stepper', 1)
        assert self.declare_parameter('entraxe', 18.44)
        assert self.declare_parameter('wheel_radius', 4.06)
        assert self.declare_parameter('step_count', 200)
        assert self.declare_parameter('encoder_tics_count', 1200)
        assert self.declare_parameter('left_wheel_encoder', 2)
        assert self.declare_parameter('right_wheel_encoder', 3)
        assert self.declare_parameter('navigation_hub', 723793)
        assert self.declare_parameter('odom_period', 0.5)
        assert self.declare_parameter('stop_topic', "stop")
        assert self.declare_parameter('speed_factor',"speed_factor")
        assert self.declare_parameter('tics_per_step', 32)

        # ===== get the param and compute all the values needed

        hub_serial = self.get_parameter('navigation_hub').get_parameter_value().integer_value

        # for steppers
        stop_topic = self.get_parameter('stop_topic').get_parameter_value().string_value
        speed_factor = self.get_parameter('speed_factor').get_parameter_value().string_value
        tics_per_step = self.get_parameter('tics_per_step').get_parameter_value().integer_value
        step_count = self.get_parameter('step_count').get_parameter_value().double_value # steps per rotation

        # geometry constraints
        self.entraxe = self.get_parameter('entraxe').get_parameter_value().double_value / 100  # meters
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value / 100  # meters
        self.step_per_meter = 1 / (2 * math.pi * self.wheel_radius) * step_count

        # for encoders
        self.odom_activate = self.get_parameter('encoder_tics_count').get_parameter_value().integer_value\
                        or self.get_parameter('left_wheel_encoder').get_parameter_value().integer_value\
                        or self.get_parameter('right_wheel_encoder').get_parameter_value().integer_value

        if self.odom_activate:
            assert self.get_parameter('encoder_tics_count').get_parameter_value().integer_value
            assert self.get_parameter('left_wheel_encoder').get_parameter_value().integer_value
            assert self.get_parameter('right_wheel_encoder').get_parameter_value().integer_value

        self.encoder_tics_count = self.get_parameter('encoder_tics_count').get_parameter_value().integer_value
        odom_period = self.get_parameter('odom_period').get_parameter_value().double_value

        print("[cmd_vel] All params init")

        # ==== init steppers and associated action server
        time.sleep(0.5)

        self.left_stepper_obj = SpeedControllerServer(self, hub_serial,
                                                 self.get_parameter('left_wheel_stepper').get_parameter_value().integer_value,
                                                 "left_wheel",
                                                 1 / tics_per_step,
                                                 stop_topic,
                                                 speed_factor)
        time.sleep(0.5)
        
        self.left_stepper_as = ActionClient(self, SpeedController, 'left_wheel')

        time.sleep(0.5)

        self.right_stepper_obj = SpeedControllerServer(self, hub_serial,
                                                 self.get_parameter('right_wheel_stepper').get_parameter_value().integer_value,
                                                 "right_wheel",
                                                 1 / tics_per_step,
                                                 stop_topic,
                                                 speed_factor)
        time.sleep(0.5)
        self.right_stepper_as = ActionClient(self, SpeedController, 'right_wheel')

        time.sleep(0.5)

        print("[cmd_vel] All steppers started")

        if self.odom_activate:
            # init the odom publisher
            self.odom_pub = self.create_publisher(Odometry, "/odom", 1)

            # init encoders
            self.left_encoder = init_encoder(hub_serial,
                                             self.get_parameter('left_wheel_encoder').get_value().integer_value,
                                             int(odom_period*1000/5))
            
            time.sleep(0.5)

            self.right_encoder = init_encoder(hub_serial,
                                              self.get_parameter('right_wheel_encoder').get_value().integer_value,
                                              int(odom_period*1000/5))

            # the initial position and speed
            self.wheels_pos = (-self.left_encoder.getPosition(), self.right_encoder.getPosition())
            self.timer = self.create_timer(odom_period, self.timer_callback)

            # the position (0, 0)
            self.pos = Pose()

            self.get_logger().info('[cmd_vel] All encoders started')

        self.last_cmd_vel = None
        self.cmd_vel_canceled = False
        self._cmd_vel = self.create_subscription(Twist, "cmd_vel", self.cmd_callback, 1)

        self.get_logger().info('[cmd_vel] Ready')


def cmd_callback(self, msg):
        self.last_cmd_vel = msg  # update the last cmd_vel received

        if not self.cmd_vel_canceled:  # if the action server weren't canceled
            self.cmd_vel_canceled = True  # cancel the stepper action

            # wait for server to cancel, and cmd_vel
            if self.right_stepper_as.get_state() == SpeedControllerServer.GoalStatus.STATUS_EXECUTING:
                self.right_stepper_as.cancel_goal()

            if self.left_stepper_as.get_state() == SpeedControllerServer.GoalStatus.STATUS_EXECUTING:
                self.left_stepper_as.cancel_goal()

            right_goal = SpeedController()
            right_goal.velocity_limit = (self.last_cmd_vel.linear.x + self.last_cmd_vel.angular.z
                                         * self.get_parameter('entraxe').get_value()/2) * self.get_parameter('step_per_meter').get_value()
            left_goal = SpeedController()
            left_goal.velocity_limit = -(self.last_cmd_vel.linear.x - self.last_cmd_vel.angular.z
                                         * self.get_parameter('entraxe').get_value()/2) * self.get_parameter('step_per_meter').get_value()

            self.right_stepper_as.send_goal(right_goal)
            self.left_stepper_as.send_goal(left_goal)
            self.cmd_vel_canceled = False

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library

    # Create an instance of your custom node
    cmd_vel_subscriber = CmdVelSubscriber()

    need = {linear: {x: 2.0, y: 3.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}

    # ==== TEST 1
    future = cmd_vel_subscriber.cmd_callback(need)
    rclpy.spin_until_future_complete(cmd_vel_subscriber, future)

    # Spin the node so its callbacks can be called
    rclpy.spin(cmd_vel_subscriber)

    # Shutdown the ROS 2 Python client library
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()