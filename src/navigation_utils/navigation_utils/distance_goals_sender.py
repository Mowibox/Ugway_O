import rclpy
from rclpy.node import Node 
from rclpy.action import ActionClient 
from phidget_stepper_controllers_msgs.action import WheelsDistance


class StepGoalSender(Node):
    def __init__(self):
        super().__init__('step_goal_sender')
        self.client = ActionClient(self, WheelsDistance, 'WheelsDistance')
        self.client.wait_for_server()
    
    def send_goal(self, distance_left, distance_right, velocity_limit):
        goal_msg = WheelsDistance.Goal()
        goal_msg.distance_goal_left = distance_left
        goal_msg.distance_goal_right = distance_right
        goal_msg.velocity_limit = velocity_limit

        self.client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    goal_sender = StepGoalSender()

    distance_left = 100
    distance_right = 100
    velocity_limit = 1
    goal_sender.send_goal(distance_left, distance_right, velocity_limit)

    rclpy.spin_once(goal_sender)

    goal_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

