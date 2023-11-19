from typing import List

import time
import rclpy
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSPresetProfiles
from avr_pcc_2023_interfaces.srv import SetLedStrip
from std_srvs.srv import Trigger
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from avr_vmc_2023_auton_drop_interfaces.action import AutonDrop


class AutonDropNode(Node):
    def __init__(self) -> None:
        super().__init__('auton_drop', namespace='auton_drop')

        # delay parameter
        self.declare_parameter('delay', 0.75)
        self.delay = float(self.get_parameter('delay').value)

        self.enabled = False
        self.should_drop = False
        self.apriltag: List[AprilTagDetection] = []

        self.auton_trigger_client = ActionServer(
            self,
            AutonDrop,
            'trigger',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )

        # when you call it, it will start searching for a tag,

        self.trigger_client = self.create_client(
            Trigger,
            '/bdu/trigger'
        )

        self.led_client = self.create_client(
            SetLedStrip,
            '/led_strips/set'
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

    # send one empty feedback message when we find an apriltag,

    # send result after we move servo

    def goal_callback(self, goal_request: AutonDrop.Goal) -> GoalResponse:
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

        while True:
            time.sleep(1 / 12)
            if not goal_handle.is_active:
                self.get_logger().info('Drop aborted')
                break

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Drop canceled')
                break

            if len(self.apriltag) >= 1:
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
                    led_future.add_done_callback(lambda: self.drop_timer.reset())  # <- in this callback add the timer

                break

        goal_handle.succeed()
        self.enabled = False
        return AutonDrop.Result()

    def apriltag_callback(self, detections: AprilTagDetectionArray) -> None:
        self.apriltag = detections.detections

    def do_drop(self) -> None:
        self.drop_timer.cancel()

        self.get_logger().info('Triggering drop')
        trigger_request = Trigger.Request()
        self.trigger_client.call_async(trigger_request)


def main() -> None:
    rclpy.init()
    node = AutonDropNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(node, executor)


if __name__ == '__main__':
    main()
