import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None


DISPLAY_WINDOW_NAME = 'AICAR Drive Direction'


class DriveDirectionDisplayNode(Node):
    def __init__(self):
        super().__init__('drive_direction_display_node')
        self.get_logger().warn('Display-only drive direction node started. Motors are not controlled here.')

        self.last_direction = None
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_display')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.subscription = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.get_logger().info(f'Subscribed to display command topic: {cmd_vel_topic}')

        if cv2 is None:
            self.get_logger().warn('OpenCV is not available, so direction will be logged only.')

    def cmd_vel_callback(self, msg):
        direction, color = self.get_direction(msg)

        if direction != self.last_direction:
            self.get_logger().info(f'Drive direction: {direction}')
            self.last_direction = direction

        if cv2 is None:
            return

        canvas = np.full((260, 520, 3), 245, dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 2.4
        thickness = 6
        text_size, baseline = cv2.getTextSize(direction, font, font_scale, thickness)
        text_x = max(10, (canvas.shape[1] - text_size[0]) // 2)
        text_y = (canvas.shape[0] + text_size[1]) // 2 - baseline
        cv2.putText(canvas, direction, (text_x, text_y), font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.imshow(DISPLAY_WINDOW_NAME, canvas)
        cv2.waitKey(1)

    def get_direction(self, msg):
        angular_threshold = 0.05
        stop_threshold = 0.01

        if abs(msg.linear.x) < stop_threshold and abs(msg.angular.z) < stop_threshold:
            return 'STOP', (0, 0, 255)
        if msg.angular.z > angular_threshold:
            return 'LEFT', (255, 120, 0)
        if msg.angular.z < -angular_threshold:
            return 'RIGHT', (0, 180, 0)
        return 'STRAIGHT', (80, 80, 80)

    def destroy_node(self):
        if cv2 is not None:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriveDirectionDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
