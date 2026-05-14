import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import lgpio
except ImportError:
    lgpio = None

# --- 하드웨어 핀 정의 ---
PWMA = 18; AIN1 = 22; AIN2 = 27
PWMB = 23; BIN1 = 25; BIN2 = 24
GPIOCHIP = 4; PWM_FREQ = 1000
DISPLAY_WINDOW_NAME = 'AICAR Drive Direction'

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.get_logger().info('Motor Node (Standard Twist) started.')
        
        # --- 튜닝 파라미터 ---
        # wheel_separation: 왼쪽 바퀴 중심과 오른쪽 바퀴 중심 사이의 거리 [미터]
        self.declare_parameter('enable_motors', False)
        self.declare_parameter('wheel_separation', 0.106)
        self.declare_parameter('speed_gain', 150.0) # m/s -> PWM 변환 비율
        self.declare_parameter('display_only', False) # 테스트용: 모터 대신 화면 표시
        self.declare_parameter('cmd_timeout', 0.5) # 명령 끊김 시 자동 정지 시간

        self.enable_motors = self.get_parameter('enable_motors').get_parameter_value().bool_value
        self.wheel_sep = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.speed_gain = self.get_parameter('speed_gain').get_parameter_value().double_value
        self.display_only = self.get_parameter('display_only').get_parameter_value().bool_value
        self.cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        self.last_display_direction = None
        self.last_cmd_time = self.get_clock().now().nanoseconds / 1e9
        self.watchdog_stopped = True

        self.h = None
        if self.display_only:
            self.get_logger().warn('Display-only mode enabled. GPIO motor output is disabled.')
            if cv2 is None:
                self.get_logger().warn('OpenCV is not available, so direction will be logged only.')
        elif not self.enable_motors:
            self.get_logger().warn('Motors are disabled. Set enable_motors:=true to allow GPIO output.')
        else:
            if lgpio is None:
                raise RuntimeError('lgpio is required when display_only is false.')

            # GPIO 초기화
            self.h = lgpio.gpiochip_open(GPIOCHIP)
            for pin in [PWMA, AIN1, AIN2, PWMB, BIN1, BIN2]:
                lgpio.gpio_claim_output(self.h, pin)
            self.motor_stop()

        # 구독 변경: /drive (Ackermann) -> /cmd_vel (Twist)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.1, self.watchdog_callback)

    def cmd_vel_callback(self, msg):
        if self.display_only:
            self.show_drive_direction(msg)
            return
        if not self.enable_motors or self.h is None:
            return

        self.last_cmd_time = self.get_clock().now().nanoseconds / 1e9
        self.watchdog_stopped = False

        # 1. Twist 메시지에서 선속도(v), 각속도(w) 추출
        linear_x = -msg.linear.x   # m/s
        angular_z = -msg.angular.z # rad/s

        angular_z_gain = 1.85  # 필요에 따라 이 값을 조절하세요.
        angular_z_amplified = angular_z * angular_z_gain

        # 2. 차동 구동 역기구학(Inverse Kinematics) 공식 적용
        # v_left  = v - (w * L / 2)
        # v_right = v + (w * L / 2)
        vl_ms = linear_x - (angular_z_amplified * self.wheel_sep / 2.0)
        vr_ms = linear_x + (angular_z_amplified * self.wheel_sep / 2.0)

        # 3. m/s를 PWM 듀티비(0~100)로 변환
        duty_l = np.clip(vl_ms * self.speed_gain, -100, 100)
        duty_r = np.clip(vr_ms * self.speed_gain, -100, 100)

        # 4. 모터 하드웨어 제어
        self.set_motor(AIN1, AIN2, PWMA, duty_r)
        self.set_motor(BIN1, BIN2, PWMB, duty_l)

    def watchdog_callback(self):
        if self.display_only or self.h is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_cmd_time > self.cmd_timeout and not self.watchdog_stopped:
            self.motor_stop()
            self.watchdog_stopped = True
            self.get_logger().warn('No /cmd_vel received recently. Motors stopped by watchdog.')

    def show_drive_direction(self, msg):
        angular_threshold = 0.05
        stop_threshold = 0.01

        if abs(msg.linear.x) < stop_threshold and abs(msg.angular.z) < stop_threshold:
            direction = 'STOP'
            color = (0, 0, 255)
        elif msg.angular.z > angular_threshold:
            direction = 'LEFT'
            color = (255, 120, 0)
        elif msg.angular.z < -angular_threshold:
            direction = 'RIGHT'
            color = (0, 180, 0)
        else:
            direction = 'STRAIGHT'
            color = (80, 80, 80)

        if direction != self.last_display_direction:
            self.get_logger().info(f'Display-only drive direction: {direction}')
            self.last_display_direction = direction

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

    def set_motor(self, in1, in2, pwm_pin, duty):
        if duty >= 0:   # 전진
            lgpio.gpio_write(self.h, in1, 0)
            lgpio.gpio_write(self.h, in2, 1)
        else:           # 후진
            lgpio.gpio_write(self.h, in1, 1)
            lgpio.gpio_write(self.h, in2, 0)
        lgpio.tx_pwm(self.h, pwm_pin, PWM_FREQ, abs(duty))

    def motor_stop(self):
        self.set_motor(AIN1, AIN2, PWMA, 0)
        self.set_motor(BIN1, BIN2, PWMB, 0)

    def destroy_node(self):
        if self.h:
            self.motor_stop()
            lgpio.gpiochip_close(self.h)
        if self.display_only and cv2 is not None:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
