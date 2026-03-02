import math
from enum import Enum, auto
from typing import Any, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from typing_extensions import Self


class ShowPhase(Enum):
    UNDOCK = auto()
    PATROL = auto()
    DOCK = auto()
    FINISHED = auto()
    FAILED = auto()


class Tb4Patrol(Node):
    """Simple TurtleBot4 Pro show cycle: Undock -> Patrol -> Dock."""

    def __init__(self) -> None:
        super().__init__("tb4_patrol")

        self.work_callback_group = ReentrantCallbackGroup()

        self.declare_parameters(
            namespace="",
            parameters=[
                ("patrol_duration_sec", 45.0),
                ("linear_speed", 0.16),
                ("turn_speed", 0.8),
                ("obstacle_distance_m", 0.55),
                ("front_sector_deg", 40.0),
                ("side_sector_deg", 40.0),
                ("max_action_retries", 3),
            ],
        )

        self.patrol_duration_sec = float(
            self.get_parameter("patrol_duration_sec").value
        )
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.obstacle_distance_m = float(
            self.get_parameter("obstacle_distance_m").value
        )
        self.front_sector_deg = float(
            self.get_parameter("front_sector_deg").value
        )
        self.side_sector_deg = float(
            self.get_parameter("side_sector_deg").value
        )
        self.max_action_retries = int(
            self.get_parameter("max_action_retries").value
        )

        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
            callback_group=self.work_callback_group,
        )

        self.subscriber_scan = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
            callback_group=self.work_callback_group,
        )

        self.undock_action_client = ActionClient(
            self,
            Undock,
            "/undock",
            callback_group=self.work_callback_group,
        )

        self.dock_action_client = ActionClient(
            self,
            Dock,
            "/dock",
            callback_group=self.work_callback_group,
        )

        self.phase = ShowPhase.UNDOCK
        self.latest_scan: Optional[LaserScan] = None
        self.patrol_start_time: Optional[float] = None

        self.action_status_undock: Optional[int] = None
        self.action_status_dock: Optional[int] = None

        self.undock_goal_sent = False
        self.dock_goal_sent = False

        self.undock_retries = 0
        self.dock_retries = 0

        self.timer_state = self.create_timer(
            0.1,
            self.state_machine_step,
            callback_group=self.work_callback_group,
        )

        self.timer_patrol = self.create_timer(
            0.1,
            self.patrol_control_step,
            callback_group=self.work_callback_group,
        )
        self.timer_patrol.cancel()

        self.get_logger().info("tb4_patrol node started")

    def state_machine_step(self) -> None:
        if self.phase == ShowPhase.UNDOCK:
            self._run_undock_phase()
            return

        if self.phase == ShowPhase.PATROL:
            self._run_patrol_phase()
            return

        if self.phase == ShowPhase.DOCK:
            self._run_dock_phase()
            return

        if self.phase in (ShowPhase.FINISHED, ShowPhase.FAILED):
            self.stop_robot()

    def _run_undock_phase(self) -> None:
        if not self.undock_goal_sent:
            if not self.undock_action_client.wait_for_server(timeout_sec=0.0):
                self.get_logger().info(
                    "Waiting for /undock action server...",
                    throttle_duration_sec=5.0,
                )
                return

            self.send_goal_undock()
            self.undock_goal_sent = True
            return

        if self.action_status_undock is None:
            return

        if self.action_status_undock == GoalStatus.STATUS_SUCCEEDED:
            self.phase = ShowPhase.PATROL
            self.patrol_start_time = self.get_clock().now().nanoseconds / 1e9
            self.timer_patrol.reset()
            self.get_logger().info("Undock done, switching to patrol")
            return

        self.undock_retries += 1
        if self.undock_retries > self.max_action_retries:
            self.phase = ShowPhase.FAILED
            self.get_logger().error("Undock failed after max retries")
            return

        self.get_logger().warn("Undock failed, retrying...")
        self.action_status_undock = None
        self.undock_goal_sent = False

    def _run_patrol_phase(self) -> None:
        if self.patrol_start_time is None:
            self.patrol_start_time = self.get_clock().now().nanoseconds / 1e9

        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - self.patrol_start_time

        if elapsed >= self.patrol_duration_sec:
            self.timer_patrol.cancel()
            self.stop_robot()
            self.phase = ShowPhase.DOCK
            self.get_logger().info(
                "Patrol duration is over, switching to dock"
            )

    def _run_dock_phase(self) -> None:
        if not self.dock_goal_sent:
            if not self.dock_action_client.wait_for_server(timeout_sec=0.0):
                self.get_logger().info(
                    "Waiting for /dock action server...",
                    throttle_duration_sec=5.0,
                )
                return

            self.send_goal_dock()
            self.dock_goal_sent = True
            return

        if self.action_status_dock is None:
            return

        if self.action_status_dock == GoalStatus.STATUS_SUCCEEDED:
            self.phase = ShowPhase.FINISHED
            self.get_logger().info("Dock done, show cycle finished")
            return

        self.dock_retries += 1
        if self.dock_retries > self.max_action_retries:
            self.phase = ShowPhase.FAILED
            self.get_logger().error("Dock failed after max retries")
            return

        self.get_logger().warn("Dock failed, retrying...")
        self.action_status_dock = None
        self.dock_goal_sent = False

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def patrol_control_step(self) -> None:
        if self.phase != ShowPhase.PATROL:
            return

        if self.latest_scan is None:
            self.get_logger().warn(
                "No /scan messages yet", throttle_duration_sec=5.0
            )
            self.stop_robot()
            return

        cmd_msg = Twist()

        front_min = self._sector_min_distance(
            self.latest_scan, center_deg=0.0, width_deg=self.front_sector_deg
        )
        left_min = self._sector_min_distance(
            self.latest_scan, center_deg=90.0, width_deg=self.side_sector_deg
        )
        right_min = self._sector_min_distance(
            self.latest_scan, center_deg=-90.0, width_deg=self.side_sector_deg
        )

        if front_min < self.obstacle_distance_m:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = (
                self.turn_speed if left_min > right_min else -self.turn_speed
            )
        else:
            cmd_msg.linear.x = self.linear_speed
            cmd_msg.angular.z = 0.0

        self.publisher_cmd_vel.publish(cmd_msg)

    def _sector_min_distance(
        self,
        scan_msg: LaserScan,
        center_deg: float,
        width_deg: float,
    ) -> float:
        if not scan_msg.ranges:
            return math.inf

        if scan_msg.angle_increment == 0.0:
            return math.inf

        half_width = math.radians(width_deg) / 2.0
        center_rad = math.radians(center_deg)
        start = center_rad - half_width
        end = center_rad + half_width

        min_range = math.inf

        for i, value in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            if angle < start or angle > end:
                continue

            if math.isinf(value) or math.isnan(value):
                continue

            if value < scan_msg.range_min or value > scan_msg.range_max:
                continue

            if value < min_range:
                min_range = value

        return min_range

    def send_goal_undock(self) -> None:
        goal_msg = Undock.Goal()
        send_goal_future = self.undock_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.undock_response_callback)
        self.get_logger().info("Sending undock goal")

    def undock_response_callback(self, future: Future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Undock goal was rejected")
            self.action_status_undock = GoalStatus.STATUS_ABORTED
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future: Future) -> None:
        result = future.result()
        self.action_status_undock = result.status

    def send_goal_dock(self) -> None:
        goal_msg = Dock.Goal()
        send_goal_future = self.dock_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.dock_response_callback)
        self.get_logger().info("Sending dock goal")

    def dock_response_callback(self, future: Future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Dock goal was rejected")
            self.action_status_dock = GoalStatus.STATUS_ABORTED
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.dock_result_callback)

    def dock_result_callback(self, future: Future) -> None:
        result = future.result()
        self.action_status_dock = result.status

    def stop_robot(self) -> None:
        cmd_msg = Twist()
        self.publisher_cmd_vel.publish(cmd_msg)

    def __enter__(self) -> Self:
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.stop_robot()


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads=2)

    with Tb4Patrol() as tb4_patrol:
        try:
            executor.add_node(tb4_patrol)
            executor.spin()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            tb4_patrol.get_logger().warn("Killing tb4_patrol node...")
        finally:
            tb4_patrol.stop_robot()
            executor.remove_node(tb4_patrol)
            executor.shutdown()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
