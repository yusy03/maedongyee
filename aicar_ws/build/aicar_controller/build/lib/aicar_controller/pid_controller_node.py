import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import deque
import lgpio
# sys, termios, tty, select (키보드 관련 모듈은 제거됨)

# --- 상태 상수 정의 ---
STATE_WAITING_FOR_SYSTEM = 'WAITING_FOR_SYSTEM' # AI 모델 준비 대기
STATE_NORMAL = 'NORMAL'
STATE_STOP_WAIT = 'STOP_WAIT'         # 정지 후 대기
STATE_TURNING = 'TURNING'
STATE_POST_TURN_STRAIGHT = 'POST_TURN_STRAIGHT'
STATE_FINISHED = 'FINISHED'           # 완주 후 정지

BUZZER_PIN = 12
BUZZER_FREQ = 1500

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        self.get_logger().info('PID Controller Node (FINAL ROBUST VERSION) started.')

        self.bridge = CvBridge()

        # --- 1. 파라미터 ---
        self.declare_parameter('vehicle_speed', 0.3)
        self.declare_parameter('ym_per_pix', 0.01)
        self.declare_parameter('xm_per_pix', 0.005)
        self.declare_parameter('half_track_width_pixels', 250)
        self.declare_parameter('kp', 0.85)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('lookahead_row_offset', 1)
        self.declare_parameter('cmd_delay', 2.0)

        self.base_speed = self.get_parameter('vehicle_speed').get_parameter_value().double_value
        self.xm_per_pix = self.get_parameter('xm_per_pix').get_parameter_value().double_value
        self.half_track_width_px = self.get_parameter('half_track_width_pixels').get_parameter_value().integer_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.row_offset = self.get_parameter('lookahead_row_offset').get_parameter_value().integer_value
        self.cmd_delay = self.get_parameter('cmd_delay').get_parameter_value().double_value

        self.prev_error = 0.0
        self.integral_error = 0.0
        self.prev_time = self.get_clock().now().nanoseconds / 1e9

        # --- 주행 상태 머신 변수 ---
        self.drive_state = STATE_NORMAL # 라인 추종/PID 테스트용으로 즉시 주행 시작

        self.state_start_time = 0.0
        self.turn_direction = 0.0
        self.stop_wait_time = 0.0
        self.next_state_after_stop = STATE_NORMAL

        # --- 표지판 및 이벤트 로직 변수 ---
        self.detected_signs = set()
        self.current_sign = None
        self.slow_mode_end_time = 0.0
        self.slow_sign_name = 'slow'
        self.last_buzzer_time = 0.0 # 초기화 복구

        self.h = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(self.h, BUZZER_PIN)

        self.cmd_buffer = deque()

        # --- 구독 설정 ---
        self.subscription = self.create_subscription(
            Image, '/image_bev_binary', self.bev_callback, 10)
        # 빨간 종료선 정지 테스트를 잠시 비활성화함.
        # self.red_subscription = self.create_subscription(
        #     Image, '/image_red_bev', self.red_bev_callback, 10)
        self.sign_subscription = self.create_subscription(
            String, '/sign_detection', self.sign_callback, 10)
        self.status_subscription = self.create_subscription(
            String, '/system_status', self.status_callback, 10)

        self.publisher_drive = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.create_timer(0.01, self.timer_callback)

    def cleanup_after_sign(self, current_sign_name):
        """ 동작 완료 후 해당 표지판을 detected_signs Set에서 제거 (반복 가능하게 함) """
        if current_sign_name in self.detected_signs:
            self.detected_signs.discard(current_sign_name)
            self.current_sign = None

    def set_state(self, new_state):
        self.drive_state = new_state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f'State changed to: {new_state}')

        # [강건성] Stop, Turn, Finish 상태 진입 시 명령 버퍼를 비워 overshoot 방지
        if new_state in [STATE_STOP_WAIT, STATE_TURNING, STATE_FINISHED]:
            self.cmd_buffer.clear()

    def status_callback(self, msg):
        if self.drive_state == STATE_WAITING_FOR_SYSTEM:
            if msg.data == "system_ready":
                self.get_logger().info(">>> System Ready Signal Received! STARTING DRIVE.")
                self.set_state(STATE_NORMAL)

    # def red_bev_callback(self, msg):
    #     # 종료 로직은 NORMAL 상태에서만 작동
    #     if self.drive_state != STATE_NORMAL:
    #         return
    #
    #     try:
    #         red_bev = self.bridge.imgmsg_to_cv2(msg, "mono8")
    #         h, w = red_bev.shape
    #     except:
    #         return
    #
    #     # 하단 중앙 영역의 빨간색 밀도를 보고 종료선을 판단
    #     h_start = int(h * 0.80)
    #     w_start = int(w * 0.20)
    #     w_end = int(w * 0.80)
    #     detection_zone = red_bev[h_start:h, w_start:w_end]
    #
    #     total_white_pixels = np.sum(detection_zone) / 255.0
    #     zone_area = detection_zone.size
    #     white_density = total_white_pixels / zone_area
    #
    #     if white_density > 0.40:
    #         self.get_logger().warn("FINISH LINE DETECTED! Stopping Robot.")
    #         self.set_state(STATE_FINISHED)

    def sign_callback(self, msg):
        new_sign = msg.data
        now = self.get_clock().now().nanoseconds / 1e9

        if self.drive_state != STATE_NORMAL: return
        if new_sign in self.detected_signs: return

        # [수정] Horn은 독립적인 동작이므로, 쿨다운만 체크하고 바로 실행
        if new_sign == 'horn':
            if now - self.last_buzzer_time > 2.0:
                self.beep_buzzer() # <--- 이 함수가 없어서 오류남
                self.last_buzzer_time = now
            return

        # --- NEW SIGN DETECTION START ---
        self.get_logger().info(f'>>> NEW SIGN DETECTED: {new_sign}')
        self.detected_signs.add(new_sign)
        self.current_sign = new_sign

        if new_sign == 'stop':
            self.stop_wait_time = 5.0
            self.next_state_after_stop = STATE_NORMAL
            self.set_state(STATE_STOP_WAIT)

        elif new_sign == 'traffic_light_red':
            self.stop_wait_time = 3.0
            self.next_state_after_stop = STATE_TURNING
            self.turn_direction = -1.0 # 우회전
            self.set_state(STATE_STOP_WAIT)

        elif new_sign == 'traffic_light_green' or new_sign == 'traffic_light':
            self.turn_direction = -1.0 # 우회전
            self.set_state(STATE_TURNING)

        elif new_sign == self.slow_sign_name:
            self.slow_mode_end_time = now + 5.0

        elif new_sign in ['left_turn', 'right_turn']:
            self.turn_direction = 1.0 if new_sign == 'left_turn' else -1.0
            self.set_state(STATE_TURNING)

    def beep_buzzer(self):
        """ [복구됨] 부저 실행 함수 (self 인자 포함) """
        self.get_logger().info('BEEP! (1 sec)')
        lgpio.tx_pwm(self.h, BUZZER_PIN, BUZZER_FREQ, 50)
        self.create_timer(1.0, self.buzzer_off_callback)

    def buzzer_off_callback(self):
        """ [복구됨] 부저 정지 함수 (self 인자 포함) """
        lgpio.tx_pwm(self.h, BUZZER_PIN, BUZZER_FREQ, 0)

    # --- [복구됨] 모터 명령 큐 발행 함수 ---
    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.drive_state in [STATE_FINISHED, STATE_WAITING_FOR_SYSTEM]:
            self.cmd_buffer.clear()
            stop_msg = Twist()
            self.publisher_drive.publish(stop_msg)
            return

        while self.cmd_buffer:
            cmd_time, twist_msg = self.cmd_buffer[0]
            if now - cmd_time >= self.cmd_delay:
                self.publisher_drive.publish(twist_msg)
                self.cmd_buffer.popleft()
            else:
                break

    def bev_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        state_duration = now - self.state_start_time

        if self.drive_state in [STATE_FINISHED, STATE_WAITING_FOR_SYSTEM]:
            return

        try:
            bev_binary = self.bridge.imgmsg_to_cv2(msg, "mono8")
            h, w = bev_binary.shape
        except: return

        linear_vel = 0.0
        angular_vel = 0.0

        # --- 상태 머신 로직 ---
        if self.drive_state == STATE_NORMAL:
            # 1. PID 조향각 먼저 계산
            steering_angle = self.calculate_pid(bev_binary, h, w, now)

            reduction_factor = 0.05
            target_speed = self.base_speed - (abs(steering_angle) * reduction_factor)

            # 최소 속도 제한 (너무 느려서 멈추지 않게 0.12m/s 정도는 유지)
            linear_vel = max(target_speed, 0.12)

            # 표지판 감속 로직 (가변 속도와 중첩 적용)
            if self.slow_sign_name in self.detected_signs:
                if now < self.slow_mode_end_time:
                    linear_vel *= 0.5
                else:
                    self.get_logger().info("Slow mode 5s expired. Resuming full speed.")
                    self.cleanup_after_sign(self.slow_sign_name)

            angular_vel = steering_angle

        elif self.drive_state == STATE_STOP_WAIT:
            # (이하 동일)
            linear_vel = 0.0
            angular_vel = 0.0
            if state_duration >= self.stop_wait_time:
                self.cleanup_after_sign(self.current_sign)
                self.set_state(self.next_state_after_stop)

        elif self.drive_state == STATE_TURNING:
            # (이하 동일)
            linear_vel = 0.0
            angular_vel = self.turn_direction * 2.0
            if state_duration >= 1.9:
                self.set_state(STATE_POST_TURN_STRAIGHT)

        elif self.drive_state == STATE_POST_TURN_STRAIGHT:
            # (이하 동일)
            linear_vel = self.base_speed
            angular_vel = 0.0
            if state_duration >= 1.0:
                self.cleanup_after_sign(self.current_sign)
                self.set_state(STATE_NORMAL)

        twist = Twist()
        twist.linear.x = float(linear_vel)
        twist.angular.z = float(angular_vel)
        self.cmd_buffer.append((now, twist))

    def calculate_pid(self, bev_binary, h, w, now):
        y_row = h - self.row_offset
        if y_row < 0: y_row = 0
        lane_slice = bev_binary[int(y_row), :]
        indices = np.nonzero(lane_slice)[0]

        center_offset = -15.0
        robot_center_px = (w / 2.0) + center_offset
        target_px = robot_center_px

        if len(indices) > 0:
            min_x, max_x = indices[0], indices[-1]
            if min_x < robot_center_px and max_x > robot_center_px:
                target_px = (min_x + max_x) / 2.0
            elif min_x < robot_center_px:
                target_px = max_x + self.half_track_width_px
            elif min_x > robot_center_px:
                target_px = min_x - self.half_track_width_px
        else:
            target_px = robot_center_px - (self.prev_error / self.xm_per_pix)

        error_meters = (robot_center_px - target_px) * self.xm_per_pix
        dt = now - self.prev_time
        if dt <= 0: dt = 0.03
        p = self.kp * error_meters
        self.integral_error += error_meters * dt
        i = self.ki * self.integral_error

        d_error = 0.0
        if len(indices) > 0:
             d_error = (error_meters - self.prev_error)

        d = self.kd * (d_error) / dt

        self.prev_error = error_meters
        self.prev_time = now
        return p + i + d

    def destroy_node(self):
        lgpio.tx_pwm(self.h, BUZZER_PIN, BUZZER_FREQ, 0)
        if self.h: lgpio.gpiochip_close(self.h)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
