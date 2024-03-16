#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus

from irobot_create_msgs.action import Dock, Undock, RotateAngle, DriveDistance


class robotHandler(Node):

    def __init__(self, namespace):
        super().__init__(str(namespace)+'_handler')

        #Action Clients
        self._rotate_angle_action_client = ActionClient(self, RotateAngle, str(namespace)+'/rotate_angle')
        self._drive_distance_action_client = ActionClient(self, DriveDistance, str(namespace)+'/drive_distance')

        self.sim_counter = 0

        self.get_logger().info(f'Action clients setup', once=True)


    def send_rotate_angle_goal(self, angle):

        self.get_logger().info('Waiting for action server...')
        self._rotate_angle_action_client.wait_for_server()

        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = 0.5

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._rotate_angle_action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_drive_distance_goal(self, distance):

        self.get_logger().info('Waiting for action server...')
        self._rotate_angle_action_client.wait_for_server()

        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        goal_msg.max_translation_speed = 0.5

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._drive_distance_action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed')

        self.sim_counter += 1

        sim_loop(self)

def sim_loop(robot_handler):

    if (robot_handler.sim_counter % 2) == 0:
        robot_handler.send_rotate_angle_goal(1.57)
    else:
        robot_handler.send_drive_distance_goal(3.0)

def main(args=None):
    rclpy.init(args=args)

    robot1_handler = robotHandler('robot1')
    robot2_handler = robotHandler('robot2')

    sim_loop(robot1_handler)
    sim_loop(robot2_handler)

    rclpy.spin(robot2_handler)


if __name__ == '__main__':
    main()