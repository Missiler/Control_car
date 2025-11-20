import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import lgpio


#Vi testar börja med lgpio

PIN_SERVO = 18
chip = lgpio.gpiochip_open(4)
freq = 50

lgpio.gpio_claim_output(chip,PIN_SERVO)

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,          # message type
            'cmd_vel',      # topic name
            self.listener_callback,
            10              # QoS
        )
        self.subscription

    def listener_callback(self, msg: Twist):
        lin = msg.linear
        ang = msg.angular
        
        servo_max = 100
        servo_min = 0
        servo_center = (servo_max + servo_min)/2
        
        servo_duty1 = map_range(-0.56, -0.56, 0.56, servo_min, servo_max)
        servo_duty2 = map_range(0.56, -0.56, 0.56, servo_min, servo_max)

        lgpio.tx_pwm(chip,PIN_SERVO,freq,servo_duty1)
        self.get_logger().info(f'angle: {servo_duty1}')
        
        time.sleep(1)
        
        lgpio.tx_pwm(chip,PIN_SERVO,freq,servo_duty2)
        self.get_logger().info(f'angle: {servo_duty2}')
        
        time.sleep(1)

        self.get_logger().info(
            f"Linear: x={lin.x:.2f}, y={lin.y:.2f}, z={lin.z:.2f} | "
            f"Angular: x={ang.x:.2f}, y={ang.y:.2f}, z={ang.z:.2f}"
        )

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber ()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
    
#NOTES:
# Angular: x = [0 0] y = [0 0] z= ([-0.56 0.56] [-45 45])
# Linear = [0.5]

#PWM-pins: 12 13 14 15
#12: PWM0_CHAN0
#13: PWMO_CHAN1
#14: PWM0_CHAN2
#15: PWM0_CHAN3
#18: PWMO0_CHAN2
#19: PWM0_CHAN3 
