#!/usr/bin/env python3
"""Cafe delivery mission for serv_robot (NavigateToPose action client)."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# yaw set to 0.0 everywhere; loose yaw tolerance means heading isn't forced
A_COUNTER = ("A_counter", 1.75, -11.3, 0.0)
B_TABLE8  = ("B_table8",  1.75, -9.5,  0.0)
C_TABLE2  = ("C_table2",  1.75, -2.5,  0.0)
SERVE_WAIT = 20.0

MISSION = [
    (B_TABLE8, SERVE_WAIT),
    (C_TABLE2, SERVE_WAIT),
    (B_TABLE8, SERVE_WAIT),
    (A_COUNTER, 0.0),
]

def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

class DeliveryRobot(Node):
    def __init__(self):
        super().__init__('cafe_delivery')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def make_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp.sec = 0
        p.header.stamp.nanosec = 0
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        goal.pose = p
        return goal

    def go_to_once(self, waypoint):
        label, x, y, yaw = waypoint
        goal = self.make_goal(x, y, yaw)
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if not gh.accepted:
            self.get_logger().error(f'    Goal to {label} REJECTED')
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().status == 4

    def go_to(self, waypoint, retries=1):
        label = waypoint[0]
        for attempt in range(retries + 1):
            self.get_logger().info(f'--> Navigating to {label} (attempt {attempt+1})')
            if self.go_to_once(waypoint):
                self.get_logger().info(f'    Reached {label}')
                return True
            self.get_logger().warn(f'    Attempt {attempt+1} to {label} failed')
            time.sleep(2.0)  # let costmaps settle before retry
        return False

    def run_mission(self):
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 action server not available.')
            return
        self.get_logger().info('Nav2 ready. Starting delivery mission.')
        for waypoint, wait_s in MISSION:
            ok = self.go_to(waypoint, retries=1)
            if ok and wait_s > 0:
                self.get_logger().info(f'    Serving... waiting {wait_s:.0f} s')
                time.sleep(wait_s)
            time.sleep(1.0)  # settle between legs
        self.get_logger().info('Delivery mission complete.')

def main():
    rclpy.init()
    node = DeliveryRobot()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
