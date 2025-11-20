import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import lgpio


#Vi testar börja med lgpio

PIN_SERVO = 18
PIN_ESC = 13
chip = lgpio.gpiochip_open(4)
f_servo = 50
calibrated = False

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
        self.calibrate_esc()
    
    def calibrate_esc(self):
        esc_max = 2000
        esc_min = 1000
        esc_neutral = 1500
        
        duty_max = (esc_max / 20000) * 100
        duty_min = (esc_min / 20000) * 100
        duty_mid = (esc_neutral / 20000) * 100

        self.get_logger().info("Calibrating ESC...")

        lgpio.tx_pwm(chip, PIN_ESC, 50, duty_max)
        time.sleep(2)
        lgpio.tx_pwm(chip, PIN_ESC, 50, duty_min)
        time.sleep(2)
        lgpio.tx_pwm(chip, PIN_ESC, 50, duty_mid)
        time.sleep(2)
        
        self.get_logger().info("ESC calibrated.")
        

    def listener_callback(self, msg: Twist):
        global calibrated
        lin = msg.linear
        ang = msg.angular
        '''
        servo_max = 2000
        servo_min = 1000
        period_servo = int(1000000/(f_servo)) #(50hz)
        
        #Servo-----------------
        #The angle is given by the pulse width of the period. For our case, it is 50Hz.
        currentAngle = round(map_range(msg.angular.z, -0.56, 0.56, -45, 45))
        
        #-45 --> Turn Left
        #+45 --> Turn Right
        
        servo_duty = round(map_range(currentAngle, -45, 45, servo_min, servo_max))

        lgpio.tx_pulse(chip,PIN_SERVO,servo_duty, period_servo-servo_duty)
        self.get_logger().info(f'angle: {servo_duty}')
        #--------------------------------------------------------
        
        '''
        #ESC--------------
        lgpio.tx_pwm(chip, PIN_ESC, 50, 10)
        time.sleep(1)
        lgpio.tx_pwm(chip, PIN_ESC, 50, 5)
        time.sleep(1)
        
        
        self.get_logger().info(
            f"Linear: x={lin.x:.2f}, y={lin.y:.2f}, z={lin.z:.2f} | "
            f"Angular: x={ang.x:.2f}, y={ang.y:.2f}, z={ang.z:.2f}"
        )

def map_range(x, in_min, in_max, out_min, out_max):
    return (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min


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
