from threading import Event
from typing import List

import time
import rclpy
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from avr_pcc_2023_interfaces.srv import SetLedStrip
from std_srvs.srv import Trigger
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from avr_vmc_2023_auton_drop_interfaces.action import AutonDrop


class AutonDropNode(Node):
    def __init__(self) -> None:
        super().__init__('auton_drop', namespace='auton_drop')

        self.declare_parameter('delay', 0.75)
        self.delay = float(self.get_parameter('delay').value)

        self.enabled = False
        self.should_drop = False
        self.apriltags: List[AprilTagDetection] = []
        self.drop_event = Event()

        self.auton_trigger_client = ActionServer(
            self,
            AutonDrop,
            'trigger',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )

        self.trigger_client = self.create_client(
            Trigger,
            '/bdu/trigger'
        )

        self.led_client = self.create_client(
            SetLedStrip,
            '/led_strip/set'
        )

        self.apriltag_subscriber = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.apriltag_callback,
            10
        )

        self.drop_timer = self.create_timer(
            callback=self.do_drop,
            timer_period_sec=self.delay,
        )
        self.drop_timer.cancel()

        self.get_logger().info('Waiting for servo trigger service')
        self.trigger_client.wait_for_service()

        self.get_logger().info('Waiting for set LED strip service')
        self.trigger_client.wait_for_service()

        self.get_logger().info('Started')

    def goal_callback(self, _: AutonDrop.Goal) -> GoalResponse:
        if not self.enabled:
            self.get_logger().info('Started auton drop')
        else:
            self.get_logger().warn('Tried to trigger an auton drop while already running')
        return GoalResponse.ACCEPT if not self.enabled else GoalResponse.REJECT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        self.enabled = True
        goal_handle.execute()

    def cancel_callback(self, _) -> CancelResponse:
        self.get_logger().debug('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # ToDo: Add Barriers or Events or something to synchronize two callbacks

        self.get_logger().info('Executing auton drop...')
        request_msg: AutonDrop.Goal = goal_handle.request
        feedback_msg = AutonDrop.Feedback()
        result_msg = AutonDrop.Result()
        success = True

        while True:
            time.sleep(1 / 12)
            if not goal_handle.is_active:
                success = False
                self.get_logger().info('Drop aborted')
                break

            if goal_handle.is_cancel_requested:
                success = False
                goal_handle.canceled()
                self.get_logger().info('Drop canceled')
                break

            detections = self.apriltags

            if len(detections) >= 1:
                feedback_msg.apriltag_id = detections[0].id

                goal_handle.publish_feedback(feedback_msg)

                self.get_logger().info('Apriltag found, flashing LEDs')

                led_request = SetLedStrip.Request()
                led_request.mode = SetLedStrip.Request.MODE_FLASH
                led_request.argument = 3
                led_request.color.r = 255
                led_request.color.g = 255
                led_request.color.b = 0

                led_future = self.led_client.call_async(led_request)

                if request_msg.should_drop:
                    led_future.add_done_callback(lambda _: self.drop_timer.reset())

                    self.drop_event.wait(5 + self.delay)
                    self.drop_event.clear()

                self.apriltags = []
                break

        if success:
            goal_handle.succeed()
        self.enabled = False
        return result_msg

    def apriltag_callback(self, detections: AprilTagDetectionArray) -> None:
        self.apriltags = detections.detections

    def do_drop(self) -> None:
        self.drop_timer.cancel()

        self.get_logger().info('Triggering drop')
        trigger_request = Trigger.Request()
        future = self.trigger_client.call_async(trigger_request)
        future.add_done_callback(lambda _: self.drop_event.set())


def main() -> None:
    rclpy.init()
    node = AutonDropNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(node, executor)


if __name__ == '__main__':
    main()
