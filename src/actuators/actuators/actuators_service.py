import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import RPi.GPIO as GPIO
import time

class ActuatorsService(Node):
    def __init__(self):
        super().__init__('actuator_service')

        self.servo_pin = 19

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        # PWM configuration
        self.pwm = GPIO.PWM(self.servo_pin, 50)  # 50 Hz

        self.subscription = self.create_subscription(
            Int32,
            'servo_angle',
            self.angle_callback,
            10)
        self.subscription  

    def angle_to_duty_cycle(self, angle):
        duty_cycle = (angle / 18) + 2
        return duty_cycle

    def set_angle(self, angle):
        duty_cycle = self.angle_to_duty_cycle(angle)
        self.pwm.start(duty_cycle)
        time.sleep(1)  
        self.pwm.stop()

    def angle_callback(self, msg):
        angle = msg.data
        self.set_angle(angle)

def main(args=None):
    rclpy.init(args=args)
    actuator_service = ActuatorService()
    rclpy.spin(actuator_service)
    actuator_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





# import RPi.GPIO as GPIO
# import time

# servo_pin = 19

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(servo_pin, GPIO.OUT)

# # PWM configuration
# pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz

# def angle_to_duty_cycle(angle):
#     duty_cycle = (angle / 18) + 2
#     return duty_cycle

# def set_angle(angle):
#     duty_cycle = angle_to_duty_cycle(angle)
#     pwm.start(duty_cycle)
#     time.sleep(1)  
#     pwm.stop()

# set_angle(135)
# time.sleep(90) # Match timer
# set_angle(0)

# GPIO.cleanup()
