import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio
import time

# === GPIO Setup ===
PIN_SERVO = 18   # PWM0 output
PIN_ESC   = 13   # PWM1 output

FREQ = 50        # 50Hz for both servo and ESC
PERIOD_US = 20000  # 20ms


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # Open main GPIO chip on Raspberry Pi 5 (ALWAYS gpiochip4)
        self.chip = lgpio.gpiochip_open(4)

        # Claim outputs
        lgpio.gpio_claim_output(self.chip, PIN_SERVO)
        lgpio.gpio_claim_output(self.chip, PIN_ESC)

        # Start stable PWM at neutral positions
        # Servo neutral: 1500 us
        # ESC neutral (armed state): 1500 us
        lgpio.tx_pwm(self.chip, PIN_SERVO, FREQ, 7.5)  # 7.5% = 1500us
        lgpio.tx_pwm(self.chip, PIN_ESC,   FREQ, 7.5)

        # Calibrate ESC once
        self.calibrate_esc()

        # Subscribe
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10
        )

        self.get_logger().info("PWM initialized; servo+ESC ready")

    def calibrate_esc(self):
        self.get_logger().info("Calibrating ESC...")

        # Full throttle (2000 us = 10%)
        lgpio.tx_pwm(self.chip, PIN_ESC, FREQ, 10)
        time.sleep(2)

        # Zero throttle (1000 us = 5%)
        lgpio.tx_pwm(self.chip, PIN_ESC, FREQ, 5)
        time.sleep(2)

        # Neutral throttle (1500 us = 7.5%)
        lgpio.tx_pwm(self.chip, PIN_ESC, FREQ, 7.5)
        time.sleep(2)

        self.get_logger().info("ESC calibrated and armed.")

    def listener_callback(self, msg: Twist):
        # === Servo (steering) ===
        angle = msg.angular.z
        servo_us = self.map_range(angle, -0.56, 0.56, 1000, 2000)
        servo_duty = servo_us / PERIOD_US * 100

        # Send servo pulse
        lgpio.tx_pwm(self.chip, PIN_SERVO, FREQ, servo_duty)

        # === ESC (throttle) ===
        throttle = msg.linear.x  # 0 → 1
        esc_us = self.map_range(throttle, 0, 1, 1000, 2000)
        esc_duty = esc_us / PERIOD_US * 100

        lgpio.tx_pwm(self.chip, PIN_ESC, FREQ, esc_duty)

    def map_range(self, x, a, b, c, d):
        return (x - a) * (d - c) / (b - a) + c


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
