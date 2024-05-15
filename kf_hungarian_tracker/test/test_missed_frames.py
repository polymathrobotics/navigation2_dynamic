import pytest
import launch
import launch_testing
import launch_testing.actions
import launch_testing.util
import uuid
import math
from unique_identifier_msgs.msg import UUID
from kf_hungarian_tracker.obstacle_class import ObstacleClass
from launch.actions import IncludeLaunchDescription
from geometry_msgs.msg import Point, Vector3
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import unittest

import rclpy
import time

from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
from nav_msgs.msg import Odometry


@pytest.mark.launch_test
def generate_test_description():
    device_under_test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("kf_hungarian_tracker"), "launch/gazebo.launch.py"]
            )
        )
    )
    context = {"device_under_test": device_under_test}
    return (
        launch.LaunchDescription(
            [device_under_test, launch_testing.actions.ReadyToTest()]
        ),
        context,
    )


class TestProcessOutput(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setupObstacle(self, x, y, z, vx, vy, vz, uuid):
        obstacle = Obstacle()
        obstacle.uuid = uuid
        obstacle.position.x = x
        obstacle.position.y = y
        obstacle.position.z = z

        obstacle.velocity.x = vx
        obstacle.velocity.y = vy
        obstacle.velocity.z = vz

        obstacle.size.x = 0.0
        obstacle.size.y = 0.0
        obstacle.size.z = 0.0

        obstacle_class = ObstacleClass(
            obstacle,
            True,
            [1e-6, 1e-6, 1e-6],
            [1.0, 1.0, 1.0, 10.0, 10.0, 10.0],
            [1e-6, 1e-6, 1e-6],
        )

        return obstacle_class

    def setUp(self):
        # Create a ROS node for tests
        self.received_messages = []
        self.node = rclpy.create_node("listener_node")
        self.publisher_ = self.node.create_publisher(ObstacleArray, "/detection", 10)
        self.odom_publisher_ = self.node.create_publisher(Odometry, "/odom", 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.node.create_timer(self.timer_period, self.timer_callback)

        uuid_msg_a = UUID()
        uuid_msg_a.uuid = list(uuid.uuid4().bytes)

        uuid_msg_b = UUID()
        uuid_msg_b.uuid = list(uuid.uuid4().bytes)

        self.obstacle_list = []
        self.obstacle_list.append(
            self.setupObstacle(10.0, 20.0, 0.0, 1.5, 0.0, 0.0, uuid_msg_a)
        )
        self.obstacle_list.append(
            self.setupObstacle(10.0, -20.0, 0.0, 1.0, 1.0, 0.0, uuid_msg_b)
        )

        self.velocity_tracking_threshold = 1e-3
        self.position_tracking_threshold = 1e-3
        self.number_of_obstacles = len(self.obstacle_list)
        self.missed_counter = 0.0
        self.ep_x = [0] * self.number_of_obstacles
        self.ep_y = [0] * self.number_of_obstacles
        self.ep_z = [0] * self.number_of_obstacles
        self.ev_x = [0] * self.number_of_obstacles
        self.ev_y = [0] * self.number_of_obstacles
        self.ev_z = [0] * self.number_of_obstacles

        self.received_obstacle_array = ObstacleArray()
        self.tracking_sent = False
        self.time0 = time.time()

    def tearDown(self):
        self.node.destroy_node()

    def predict(self, obstacle, dt):
        obstacle.predict(dt)
        obstacle.msg.position.x = float(obstacle.kalman.statePre[0][0])
        obstacle.msg.position.y = float(obstacle.kalman.statePre[1][0])
        obstacle.msg.position.z = float(obstacle.kalman.statePre[2][0])
        obstacle.msg.velocity.x = float(obstacle.kalman.statePre[3][0])
        obstacle.msg.velocity.y = float(obstacle.kalman.statePre[4][0])
        obstacle.msg.velocity.z = float(obstacle.kalman.statePre[5][0])

    def timer_callback(self):

        obstacle_array = ObstacleArray()
        obstacle_array.header.stamp = self.node.get_clock().now().to_msg()
        obstacle_array.header.frame_id = "front_radar_link"

        dt = time.time() - self.time0
        self.time0 = self.time0 + dt

        for i in range(0, len(self.obstacle_list)):
            self.predict(self.obstacle_list[i], dt)
            if i == 0:
                obstacle_array.obstacles.append(self.obstacle_list[i].msg)
            else:
                if self.missed_counter < 3:
                    obstacle_array.obstacles.append(self.obstacle_list[i].msg)
                    self.missed_counter = self.missed_counter + 1
                else:
                    # do not append the obstacle, reset the counter
                    self.missed_counter = 0

        self.publisher_.publish(obstacle_array)

    def listener_callback(self, msg):
        self.tracking_set = True
        self.received_obstacle_array = msg
        self.received_messages.append(msg)
        for i in range(0, len(self.received_obstacle_array.obstacles)):

            self.ep_x[i] = math.fabs(
                self.received_obstacle_array.obstacles[i].position.x
                - self.obstacle_list[i].msg.position.x
            )
            self.ep_y[i] = math.fabs(
                self.received_obstacle_array.obstacles[i].position.y
                - self.obstacle_list[i].msg.position.y
            )
            self.ep_z[i] = math.fabs(
                self.received_obstacle_array.obstacles[i].position.z
                - self.obstacle_list[i].msg.position.z
            )
            self.ev_x[i] = math.fabs(
                self.received_obstacle_array.obstacles[i].velocity.x
                - self.obstacle_list[i].msg.velocity.x
            )
            self.ev_y[i] = math.fabs(
                self.received_obstacle_array.obstacles[i].velocity.y
                - self.obstacle_list[i].msg.velocity.y
            )
            self.ev_z[i] = math.fabs(
                self.received_obstacle_array.obstacles[i].velocity.z
                - self.obstacle_list[i].msg.velocity.z
            )

    def test_multi_object_tracking(self, device_under_test, proc_output):
        sub = self.node.create_subscription(
            ObstacleArray, "/tracking", self.listener_callback, 10
        )
        try:
            transient_time = time.time() + 10
            end_time = time.time() + 40
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

                # Wait for convergence
                if time.time() > transient_time:
                    self.assertEqual(
                        self.number_of_obstacles,
                        len(self.received_obstacle_array.obstacles),
                        "obstacles not equal to detections",
                    )
                    for i in range(0, len(self.received_obstacle_array.obstacles)):
                        # check if received obstacle and output tracking has deviated at all
                        ep_x = math.fabs(
                            self.received_obstacle_array.obstacles[i].position.x
                            - self.obstacle_list[i].msg.position.x
                        )
                        self.assertLess(
                            self.ep_x[i],
                            self.position_tracking_threshold,
                            "x position error larger than threshold for obstacle "
                            + str(i),
                        )
                        ep_y = math.fabs(
                            self.received_obstacle_array.obstacles[i].position.y
                            - self.obstacle_list[i].msg.position.y
                        )
                        self.assertLess(
                            self.ep_y[i],
                            self.position_tracking_threshold,
                            "y position error larger than threshold for obstacle "
                            + str(i),
                        )
                        ep_z = math.fabs(
                            self.received_obstacle_array.obstacles[i].position.z
                            - self.obstacle_list[i].msg.position.z
                        )
                        self.assertLess(
                            self.ep_z[i],
                            self.position_tracking_threshold,
                            "z position error larger than threshold for obstacle "
                            + str(i),
                        )

                        ev_x = math.fabs(
                            self.received_obstacle_array.obstacles[i].velocity.x
                            - self.obstacle_list[i].msg.velocity.x
                        )
                        self.assertLess(
                            self.ev_x[i],
                            self.velocity_tracking_threshold,
                            "x velocity error larger than threshold for obstacle "
                            + str(i),
                        )
                        ev_y = math.fabs(
                            self.received_obstacle_array.obstacles[i].velocity.y
                            - self.obstacle_list[i].msg.velocity.y
                        )
                        self.assertLess(
                            self.ev_y[i],
                            self.velocity_tracking_threshold,
                            "y velocity error larger than threshold for obstacle "
                            + str(i),
                        )
                        ev_z = math.fabs(
                            self.received_obstacle_array.obstacles[i].velocity.z
                            - self.obstacle_list[i].msg.velocity.z
                        )
                        self.assertLess(
                            self.ev_z[i],
                            self.velocity_tracking_threshold,
                            "z velocity error larger than threshold for obstacle "
                            + str(i),
                        )

        finally:
            self.node.destroy_subscription(sub)
