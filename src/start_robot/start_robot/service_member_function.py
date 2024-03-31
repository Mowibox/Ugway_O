from start_robot_interface.srv import StrategyInt                                                          
import rclpy
from rclpy.node import Node


class StartService(Node):

    def __init__(self):
        super().__init__('start_service')
        self.srv = self.create_service(StrategyInt, 'start', self.strategy_callback)      
    
    # Récupération de la stratégie
    def strategy_callback(self, request, response):
        response.strategy = request.strategy                                          
        self.get_logger().info('Incoming strategy: %d' % (request.strategy))  

        return response

def main(args=None):

    rclpy.init(args=args)
    start_service = StartService()
    rclpy.spin(start_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()