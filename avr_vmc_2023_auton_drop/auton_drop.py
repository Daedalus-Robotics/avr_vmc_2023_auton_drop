import rclpy
from rclpy.node import Node
# make custom srv type that includes:
# bool enable
# bool do_drop
from avr_pcc_2023_interfaces.srv import SetLedStrip
from std_srvs.srv import Trigger, SetBool
from apriltag_msgs.msg import AprilTagDetectionArray


class AutonDropNode(Node):
    def __init__(self) -> None:
        super().__init__('auton_drop', namespace='auton_drop')

        # delay parameter
        self.declare_parameter('delay', 0.75)
        self.delay = float(self.get_parameter('delay').value)

        self.enabled = False
        self.should_drop = False

        self.enable_service = self.create_service(
            # custom message type,
            SetBool,
            'enable',
            self.enable
        )

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
            self.apriltag_callback
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

    def enable(self, request: SetBool, response: SetBool) -> SetBool:
        enable_request = request.Request
        enable_response = response.Response

        self.enabled = enable_request.data
        self.should_drop = enable_request.data

        enable_response.success = True

    def apriltag_callback(self, detections: AprilTagDetectionArray) -> None:
        if self.enabled:
            if len(detections.detections) >= 1:
                # detection = detections.detections[0] # I don't think we even need this right now

                # blink LEDs 3 times
                led_request = SetLedStrip.Request()

                led_request.mode = SetLedStrip.Request.MODE_FLASH
                led_request.argument = 3
                led_request.color.r = 255
                led_request.color.g = 255
                led_request.color.b = 0

                led_future = self.led_client.call_async(led_request)

                if self.should_drop:
                    led_future.add_done_callback(lambda: self.drop_timer.reset()) # <- in this callback add the timer
                    trigger_request = Trigger.Request()
                    self.trigger_client.call_async(trigger_request)

                # send trigger through client (don't need to worry about full or half state here)

    def do_drop(self) -> None:
        self.drop_timer.cancel()
