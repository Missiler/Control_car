import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import lgpio


#Vi testar börja med lgpio

PIN_SERVO = 18
PIN_ESC = 15
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

    def listener_callback(self, msg: Twist):
        lin = msg.linear
        ang = msg.angular
        
        servo_max = 2000
        servo_min = 1000
        period_servo = 1000000/(f_servo) #(50hz)
        
        esc_max = 1600
        esc_min = 1400
        esc_calib_max = 2000
        esc_calib_min = 1000
        
        f_esc = 50
        period_esc = int(1000000/f_esc)
        
        #Servo-----------------
        #The angle is given by the pulse width of the period. For our case, it is 50Hz.
        currentAngle = round(map_range(msg.angular.z, -0.56, 0.56, -45, 45))
        
        #-45 --> Turn Left
        #+45 --> Turn Right
        
        servo_duty = round(map_range(currentAngle, -45, 45, servo_min, servo_max))

        lgpio.tx_pulse(chip,PIN_SERVO,servo_duty, period_servo-servo_duty)
        self.get_logger().info(f'angle: {servo_duty}')
        #--------------------------------------------------------
        
        #ESC----------------
        #Works just like a servo-motor.
        
        #Has to be calibrated first.
        if calibrated == False:
            lgpio.tx_pulse(4,PIN_ESC, int(esc_calib_max), int(period_esc-esc_calib_max))
            time.sleep(2)
            lgpio.tx_pulse(4,PIN_ESC,int(esc_calib_min), int(period_esc-esc_calib_min))   
            time.sleep(2)
            lgpio.tx_pulse(4,PIN_ESC, int((esc_calib_max - esc_calib_min)-2), int(period_esc-((esc_calib_max - esc_calib_min)-2)))
            calibrated = True
        
        esc_duty = int(round(map_range(msg.linear.x, 0, 0.5, esc_max, esc_min)))
        lgpio.tx_pulse(4,PIN_ESC, esc_duty, int(period_esc-esc_duty))
        
        
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
