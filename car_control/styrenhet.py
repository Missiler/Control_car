import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio
import time

#GPIO Setup
PIN_SERVO = 18  
PIN_ESC   = 13   

FREQ = 50        
PERIOD_US = 20000  


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        self.chip = lgpio.gpiochip_open(4)

        lgpio.gpio_claim_output(self.chip, PIN_SERVO)
        lgpio.gpio_claim_output(self.chip, PIN_ESC)

        lgpio.tx_pwm(self.chip, PIN_SERVO, FREQ, 7.5)  # 7.5% = 1500us
        lgpio.tx_pwm(self.chip, PIN_ESC,   FREQ, 7.5)

        self.calibrate_esc()

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10
        )

        self.get_logger().info("PWM initialized; servo+ESC ready")

    def calibrate_esc(self):
        
        self.get_logger().info("Calibrating ESC...")
        
        #Calibrates with max and min value
        lgpio.tx_pwm(self.chip, PIN_ESC, FREQ, 10)
        time.sleep(2)
        lgpio.tx_pwm(self.chip, PIN_ESC, FREQ, 5)
        time.sleep(2)
        lgpio.tx_pwm(self.chip, PIN_ESC, FREQ, 7.5)
        time.sleep(2)

        self.get_logger().info("ESC calibrated and armed.")
        

    def listener_callback(self, msg: Twist):
        
        #Servo
        angle = msg.angular.z
        servo_us = self.map_range(angle, -0.56, 0.56, 1000, 2000)
        servo_duty = servo_us / PERIOD_US * 100

        lgpio.tx_pwm(self.chip, PIN_SERVO, FREQ, -servo_duty)

        #ESC
        throttle = msg.linear.x  # 0 → 1
        esc_us = int(self.map_range(throttle, 0, 1, 1400, 1600))
        esc_duty = esc_us / PERIOD_US * 100

        lgpio.tx_pwm(self.chip, PIN_ESC, FREQ, esc_duty)

    def map_range(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#0.25 är rakt fram

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
