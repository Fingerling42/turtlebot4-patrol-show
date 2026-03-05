import math
from enum import Enum, auto
from typing import Any, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock
from nav_msgs.msg import Odometry
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from typing_extensions import Self


class DemoPhase(Enum):
    UNDOCK = auto()
    DRIVE_FORWARD = auto()
    ROTATE_FOR_CIRCLE = auto()
    CIRCLE = auto()
    POST_CIRCLE_FORWARD = auto()
    ROTATE_TO_DOCK = auto()
    DRIVE_BACK = auto()
    DOCK = auto()
    WAIT_NEXT_CYCLE = auto()
    FAILED = auto()


class Tb4Patrol(Node):
    """Simple scripted TurtleBot4 Pro demo with periodic cycles."""

    def __init__(self) -> None:
        super().__init__("tb4_patrol")

        self.work_callback_group = ReentrantCallbackGroup()

        self.declare_parameters(
            namespace="",
            parameters=[
                ("cycle_period_sec", 180.0),
                ("undock_settle_sec", 1.0),
                ("forward_distance_m", 0.25),
                ("forward_speed_mps", 0.10),
                ("rotate_before_circle_deg", 90.0),
                ("rotate_speed_radps", 0.6),
                ("post_circle_forward_distance_m", 0.10),
                ("circle_diameter_m", 1.0),
                ("circle_linear_speed_mps", 0.12),
                ("max_action_retries", 3),
                ("control_rate_hz", 10.0),
                ("telemetry_log_period_sec", 2.0),
                ("obstacle_log_warn_distance_m", 0.40),
            ],
        )

        self.cycle_period_sec = float(self.get_parameter("cycle_period_sec").value)
        self.undock_settle_sec = float(self.get_parameter("undock_settle_sec").value)
        self.forward_distance_m = float(self.get_parameter("forward_distance_m").value)
        self.forward_speed_mps = float(self.get_parameter("forward_speed_mps").value)
        self.rotate_before_circle_deg = float(
            self.get_parameter("rotate_before_circle_deg").value
        )
        self.rotate_speed_radps = float(
            self.get_parameter("rotate_speed_radps").value
        )
        self.post_circle_forward_distance_m = float(
            self.get_parameter("post_circle_forward_distance_m").value
        )
        self.circle_diameter_m = float(self.get_parameter("circle_diameter_m").value)
        self.circle_linear_speed_mps = float(
            self.get_parameter("circle_linear_speed_mps").value
        )
        self.max_action_retries = int(self.get_parameter("max_action_retries").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.telemetry_log_period_sec = float(
            self.get_parameter("telemetry_log_period_sec").value
        )
        self.obstacle_log_warn_distance_m = float(
            self.get_parameter("obstacle_log_warn_distance_m").value
        )

        if self.forward_speed_mps <= 0.0:
            raise ValueError("forward_speed_mps must be > 0.0")
        if self.circle_linear_speed_mps <= 0.0:
            raise ValueError("circle_linear_speed_mps must be > 0.0")
        if self.rotate_speed_radps <= 0.0:
            raise ValueError("rotate_speed_radps must be > 0.0")
        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be > 0.0")

        self.forward_duration_sec = self.forward_distance_m / self.forward_speed_mps
        self.rotate_before_circle_rad = math.radians(self.rotate_before_circle_deg)
        self.rotate_duration_sec = (
            abs(self.rotate_before_circle_rad) / self.rotate_speed_radps
        )
        self.post_circle_forward_duration_sec = (
            self.post_circle_forward_distance_m / self.forward_speed_mps
        )
        self.rotate_direction = (
            1.0 if self.rotate_before_circle_rad >= 0.0 else -1.0
        )
        # After a full circle heading remains (initial + rotate_before_circle_rad).
        # Rotate to face dock direction (initial + pi).
        rotate_to_dock_rad = self.normalize_angle(
            math.pi - self.rotate_before_circle_rad
        )
        self.rotate_to_dock_duration_sec = (
            abs(rotate_to_dock_rad) / self.rotate_speed_radps
        )
        self.rotate_to_dock_direction = (
            1.0 if rotate_to_dock_rad >= 0.0 else -1.0
        )
        self.rotate_to_dock_deg = math.degrees(rotate_to_dock_rad)
        self.circle_radius_m = max(self.circle_diameter_m / 2.0, 0.05)
        self.circle_angular_speed_abs_rps = (
            self.circle_linear_speed_mps / self.circle_radius_m
        )
        # Keep the circle away from dock:
        # after pre-rotation, turn in the opposite direction.
        self.circle_angular_speed_rps = (
            -self.rotate_direction * self.circle_angular_speed_abs_rps
        )
        self.circle_duration_sec = (
            2.0 * math.pi
        ) / self.circle_angular_speed_abs_rps

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
            qos_profile_sensor_data,
            callback_group=self.work_callback_group,
        )

        self.subscriber_odom = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
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

        self.phase = DemoPhase.UNDOCK
        self.phase_start_sec = self.now_sec()
        self.next_cycle_time_sec: Optional[float] = None

        self.action_status_undock: Optional[int] = None
        self.action_status_dock: Optional[int] = None
        self.undock_goal_sent = False
        self.dock_goal_sent = False
        self.undock_retries = 0
        self.dock_retries = 0

        self.odom_linear_speed_mps: Optional[float] = None
        self.odom_yaw_rate_rps: Optional[float] = None
        self.scan_nearest_m: Optional[float] = None
        self.scan_nearest_deg: Optional[float] = None
        self.last_telemetry_log_sec = 0.0

        control_period_sec = 1.0 / self.control_rate_hz
        self.timer_control = self.create_timer(
            control_period_sec,
            self.control_step,
            callback_group=self.work_callback_group,
        )

        self.get_logger().info(
            (
                "tb4_patrol started: forward=%.2fm @ %.2fm/s, "
                "rotate=%.1fdeg @ %.2frad/s, "
                "post_circle_forward=%.2fm, "
                "return_rotate=%.1fdeg, "
                "circle_diam=%.2fm @ %.2fm/s, cycle_period=%.0fs"
            )
            % (
                self.forward_distance_m,
                self.forward_speed_mps,
                self.rotate_before_circle_deg,
                self.rotate_speed_radps,
                self.post_circle_forward_distance_m,
                self.rotate_to_dock_deg,
                self.circle_diameter_m,
                self.circle_linear_speed_mps,
                self.cycle_period_sec,
            )
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed_phase_sec(self) -> float:
        return self.now_sec() - self.phase_start_sec

    def transition_to(self, new_phase: DemoPhase, message: str) -> None:
        self.phase = new_phase
        self.phase_start_sec = self.now_sec()
        self.get_logger().info(message)

    def control_step(self) -> None:
        now = self.now_sec()

        if self.phase == DemoPhase.UNDOCK:
            self.run_undock_phase()
            return

        if self.phase == DemoPhase.DRIVE_FORWARD:
            self.publish_forward()
            self.log_telemetry(now)
            if self.elapsed_phase_sec() >= self.forward_duration_sec:
                self.stop_robot()
                self.transition_to(
                    DemoPhase.ROTATE_FOR_CIRCLE,
                    "Forward step done, rotating before circle",
                )
            return

        if self.phase == DemoPhase.ROTATE_FOR_CIRCLE:
            self.publish_rotation()
            self.log_telemetry(now)
            if self.elapsed_phase_sec() >= self.rotate_duration_sec:
                self.stop_robot()
                self.transition_to(
                    DemoPhase.CIRCLE,
                    "Rotation done, starting circle motion",
                )
            return

        if self.phase == DemoPhase.CIRCLE:
            self.publish_circle()
            self.log_telemetry(now)
            if self.elapsed_phase_sec() >= self.circle_duration_sec:
                self.stop_robot()
                self.transition_to(
                    DemoPhase.POST_CIRCLE_FORWARD,
                    "Circle done, moving a bit forward before docking turn",
                )
            return

        if self.phase == DemoPhase.POST_CIRCLE_FORWARD:
            self.publish_forward()
            self.log_telemetry(now)
            if self.elapsed_phase_sec() >= self.post_circle_forward_duration_sec:
                self.stop_robot()
                self.transition_to(
                    DemoPhase.ROTATE_TO_DOCK,
                    "Post-circle move done, rotating back toward dock",
                )
            return

        if self.phase == DemoPhase.ROTATE_TO_DOCK:
            self.publish_rotation_to_dock()
            self.log_telemetry(now)
            if self.elapsed_phase_sec() >= self.rotate_to_dock_duration_sec:
                self.stop_robot()
                self.transition_to(
                    DemoPhase.DRIVE_BACK,
                    "Facing dock, driving forward back to station",
                )
            return

        if self.phase == DemoPhase.DRIVE_BACK:
            self.publish_forward()
            self.log_telemetry(now)
            if self.elapsed_phase_sec() >= self.forward_duration_sec:
                self.stop_robot()
                self.transition_to(
                    DemoPhase.DOCK,
                    "Return step done, starting dock action",
                )
            return

        if self.phase == DemoPhase.DOCK:
            self.run_dock_phase()
            return

        if self.phase == DemoPhase.WAIT_NEXT_CYCLE:
            self.stop_robot()
            if self.next_cycle_time_sec is None:
                self.next_cycle_time_sec = now + self.cycle_period_sec
            if now >= self.next_cycle_time_sec:
                self.reset_cycle_state()
                self.transition_to(DemoPhase.UNDOCK, "Starting next demo cycle")
            return

        if self.phase == DemoPhase.FAILED:
            self.stop_robot()

    def run_undock_phase(self) -> None:
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
            if self.elapsed_phase_sec() >= self.undock_settle_sec:
                self.transition_to(
                    DemoPhase.DRIVE_FORWARD,
                    "Undock done, moving forward from station",
                )
            return

        self.undock_retries += 1
        if self.undock_retries > self.max_action_retries:
            self.transition_to(DemoPhase.FAILED, "Undock failed after max retries")
            return

        self.get_logger().warn("Undock failed, retrying...")
        self.action_status_undock = None
        self.undock_goal_sent = False

    def run_dock_phase(self) -> None:
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
            self.next_cycle_time_sec = self.now_sec() + self.cycle_period_sec
            self.transition_to(
                DemoPhase.WAIT_NEXT_CYCLE,
                "Dock done, waiting before next cycle",
            )
            return

        self.dock_retries += 1
        if self.dock_retries > self.max_action_retries:
            self.transition_to(DemoPhase.FAILED, "Dock failed after max retries")
            return

        self.get_logger().warn("Dock failed, retrying...")
        self.action_status_dock = None
        self.dock_goal_sent = False

    def publish_forward(self) -> None:
        msg = Twist()
        msg.linear.x = self.forward_speed_mps
        self.publisher_cmd_vel.publish(msg)

    def publish_circle(self) -> None:
        msg = Twist()
        msg.linear.x = self.circle_linear_speed_mps
        msg.angular.z = self.circle_angular_speed_rps
        self.publisher_cmd_vel.publish(msg)

    def publish_rotation(self) -> None:
        msg = Twist()
        msg.angular.z = self.rotate_direction * self.rotate_speed_radps
        self.publisher_cmd_vel.publish(msg)

    def publish_rotation_to_dock(self) -> None:
        msg = Twist()
        msg.angular.z = self.rotate_to_dock_direction * self.rotate_speed_radps
        self.publisher_cmd_vel.publish(msg)

    def normalize_angle(self, angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def stop_robot(self) -> None:
        if not rclpy.ok():
            return
        try:
            self.publisher_cmd_vel.publish(Twist())
        except Exception:
            # Context may already be shutting down (Ctrl+C via launch).
            return

    def scan_callback(self, msg: LaserScan) -> None:
        nearest = math.inf
        nearest_deg = 0.0

        for i, distance in enumerate(msg.ranges):
            if math.isnan(distance) or math.isinf(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            if distance < nearest:
                nearest = distance
                angle = msg.angle_min + i * msg.angle_increment
                nearest_deg = math.degrees(angle)

        if math.isinf(nearest):
            self.scan_nearest_m = None
            self.scan_nearest_deg = None
        else:
            self.scan_nearest_m = nearest
            self.scan_nearest_deg = nearest_deg

    def odom_callback(self, msg: Odometry) -> None:
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular

        self.odom_linear_speed_mps = math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        )
        self.odom_yaw_rate_rps = angular.z

    def log_telemetry(self, now_sec: float) -> None:
        if now_sec - self.last_telemetry_log_sec < self.telemetry_log_period_sec:
            return

        speed_text = "n/a"
        if self.odom_linear_speed_mps is not None:
            speed_text = "%.3f m/s" % self.odom_linear_speed_mps

        yaw_text = "n/a"
        if self.odom_yaw_rate_rps is not None:
            yaw_text = "%.3f rad/s" % self.odom_yaw_rate_rps

        obstacle_text = "n/a"
        obstacle_distance = self.scan_nearest_m
        if obstacle_distance is not None and self.scan_nearest_deg is not None:
            obstacle_text = "%.3f m at %.1f deg" % (
                obstacle_distance,
                self.scan_nearest_deg,
            )

        log_msg = (
            "phase=%s | odom_speed=%s | yaw_rate=%s | nearest_scan=%s"
            % (self.phase.name, speed_text, yaw_text, obstacle_text)
        )

        if (
            obstacle_distance is not None
            and obstacle_distance < self.obstacle_log_warn_distance_m
        ):
            self.get_logger().warn(log_msg)
        else:
            self.get_logger().info(log_msg)

        self.last_telemetry_log_sec = now_sec

    def reset_cycle_state(self) -> None:
        self.action_status_undock = None
        self.action_status_dock = None
        self.undock_goal_sent = False
        self.dock_goal_sent = False
        self.undock_retries = 0
        self.dock_retries = 0

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

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future: Future) -> None:
        result = future.result()
        status = result.status
        is_docked = bool(getattr(result.result, "is_docked", False))

        if status == GoalStatus.STATUS_SUCCEEDED and is_docked:
            self.action_status_undock = GoalStatus.STATUS_ABORTED
            self.get_logger().warn("Undock action ended but robot is still docked")
            return

        self.action_status_undock = status

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

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.dock_result_callback)

    def dock_result_callback(self, future: Future) -> None:
        result = future.result()
        status = result.status
        is_docked = bool(getattr(result.result, "is_docked", False))

        if status == GoalStatus.STATUS_SUCCEEDED and not is_docked:
            self.action_status_dock = GoalStatus.STATUS_ABORTED
            self.get_logger().warn("Dock action reported success but robot is not docked")
            return

        self.action_status_dock = status

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
            if rclpy.ok():
                tb4_patrol.get_logger().warn("Killing tb4_patrol node...")
        finally:
            tb4_patrol.stop_robot()
            executor.remove_node(tb4_patrol)
            executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
