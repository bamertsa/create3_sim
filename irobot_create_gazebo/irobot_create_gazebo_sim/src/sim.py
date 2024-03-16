#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time


from action_msgs.msg import GoalStatus

from irobot_create_msgs.action import Dock, Undock, RotateAngle, DriveDistance, NavigateToPosition


class robotHandler(Node):

    def __init__(self, namespace):
        super().__init__(str(namespace)+'_handler')

        #Action Clients
        self._undock_action_client = ActionClient(self, Undock, str(namespace)+'/undock')
        self._rotate_angle_action_client = ActionClient(self, RotateAngle, str(namespace)+'/rotate_angle')
        self._drive_distance_action_client = ActionClient(self, DriveDistance, str(namespace)+'/drive_distance')
        self._navigate_to_position_action_client = ActionClient(self, NavigateToPosition, str(namespace)+'/navigate_to_position')

        self.sim_counter = 0

        timer_period = 60  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'Action clients setup', once=True)

    def timer_callback(self):
        self.sim_counter = -1
        self.sim_loop()

    def sim_loop(self):
        if self.sim_counter == 0:
            self.send_undock_goal()
        elif (self.sim_counter % 2) == 1:
            self.send_rotate_angle_goal(1.57)
        elif self.sim_counter == -1:
            self.get_logger().info('Simulation Finished!')
            rclpy.shutdown()
        else:
            self.send_drive_distance_goal(2.0)

    def send_undock_goal(self):

        self.get_logger().info('Waiting for action server...')
        self._undock_action_client.wait_for_server()

        goal_msg = RotateAngle.Goal()

        self.get_logger().info('Sending undock request...')

        self._send_goal_future = self._rotate_angle_action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_rotate_angle_goal(self, angle):

        self.get_logger().info('Waiting for action server...')
        self._rotate_angle_action_client.wait_for_server()

        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = 1.0

        self.get_logger().info('Sending roatate angle request...')

        self._send_goal_future = self._rotate_angle_action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_drive_distance_goal(self, distance):

        self.get_logger().info('Waiting for action server...')
        self._drive_distance_action_client.wait_for_server()

        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        goal_msg.max_translation_speed = 0.5

        self.get_logger().info('Sending drive distance request...')

        self._send_goal_future = self._drive_distance_action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_naviagte_to_position_goal(self, distance):

        self.get_logger().info('Waiting for action server...')
        self._navigate_to_position_action_client.wait_for_server()

        goal_msg = NavigateToPosition.Goal()
        goal_msg.achieve_goal_heading = true
        goal_msg.goal_pose.pose.position.x = self.x_start
        goal_msg.goal_pose.pose.position.y = 0.0
        goal_msg.goal_pose.pose.position.z = 0.0
        goal_msg.goal_pose.pose.orientation.x = 0.0
        goal_msg.goal_pose.pose.orientation.y = 0.0
        goal_msg.goal_pose.pose.orientation.z = 0.0
        goal_msg.goal_pose.pose.orientation.w = 1.0

        self.get_logger().info('Sending navigate to position request...')

        self._send_goal_future = self._navigate_to_position_action_client.send_goal_async(goal_msg)

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

        self.sim_loop()


def main(args=None):
    rclpy.init(args=args)

    robot2_handler = robotHandler('robot2')
    robot2_handler.sim_loop()

    robot1_handler = robotHandler('robot1')
    robot1_handler.sim_loop()

    rclpy.spin(robot2_handler)
    rclpy.spin(robot1_handler)
    


if __name__ == '__main__':
    main()