#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from commands import PandaCommander
from gpt import gpt_api
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import json


class GPTController(Node):
    def __init__(self):
        super().__init__('llm_executor_node')

        # CV bridge & storage
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.image_ready = False

        # GPT & robot commander
        self.gpt = gpt_api()
        self.sawyer = PandaCommander()

        # Subscriptions
        self.create_subscription(Image, '/panda/rgb_camera/image', self.rgb_cb, 10)
        self.create_subscription(Image, '/panda/depth_camera/image', self.depth_cb, 10)

        # Periodic check for main loop
        self.create_timer(1.0, self.main)  # every 1s

        self.get_logger().info('GPTController node started.')

    def rgb_cb(self, img_msg: Image):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            self.image_ready = True
        
        except Exception as e:
            self.get_logger().error(f"RGB convert error: {e}")

    def depth_cb(self, img_msg: Image):
        try:
            # depth in meters
            self.depth_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        
        except Exception as e:
            self.get_logger().error(f"Depth convert error: {e}")

    def main(self):
        # Example prompt; replace with subscriber or service as you like
        prompt = "move to blue color"
        if not self.image_ready:
            return

        self.get_logger().info("Performing action from prompt...")
        args = self.extract_args(prompt)
        output = self.gpt.get_vlm_output(self.rgb_image, args)
        self.sawyer.execute_sequence(output, self.depth_image)

    def extract_args(self, action_str: str) -> dict:
        try:
            parsed = json.loads(action_str)

            # Validate
            if (not isinstance(parsed.get('position'), list)
                    or len(parsed['position']) != 2):
                self.get_logger().warning(
                    "Invalid or missing 'position'; defaulting to [0,0]"
                )
                parsed['position'] = [0.0, 0.0]

            if not isinstance(parsed.get('lift_height'), (int, float)):
                self.get_logger().warning(
                    "Invalid or missing 'lift_height'; defaulting to 0"
                )
                parsed['lift_height'] = 0.0

            if not isinstance(parsed.get('actions'), list):
                self.get_logger().warning(
                    "Invalid or missing 'actions'; defaulting to []"
                )
                parsed['actions'] = []

            return parsed

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON parse error: {e}")
            return {
                "position": [0.0, 0.0],
                "lift_height": 0.0,
                "actions": []
            }

    def chk_bounds(self, point):
        u, v, z = point
        if z <= 0.0 or np.isnan(z):
            self.get_logger().warning(
                f"Invalid depth at pixel ({u},{v}): {z}"
            )
            return False

        h, w = self.depth_image.shape
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().error(
                f"Pixel out of bounds: u={u}, v={v}"
            )
            return False

        return True


def main(args=None):
    rclpy.init(args=args)
    node = GPTController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
