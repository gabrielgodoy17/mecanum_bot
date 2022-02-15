import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DirectionMapper(Node):

    def __init__(self):
        super().__init__('direction_mapper')
        self.subscription = self.create_subscription(String, 'directions', self.listener_callback, 10)
        self.subscription
        self.publisher_ = self.create_publisher(String, 'motors', 10)
        

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        to_send = String()
        to_send.data = dict.get(msg.data)
        if to_send.data is None:
            to_send.data = dict.get("STOP")
        self.get_logger().info('Publishing: "%s"' % to_send)
        self.publisher_.publish(to_send)  


dict = {
    "FWD" : "M1+,M2+,M3+,M4+",
    "BKWD" : "M1-,M2-,M3-,M4-",
    "L" : "M1-,M2+,M3+,M4-",
    "R" : "M1+,M2-,M3-,M4+",
    "FWD_L" : "M10,M2+,M3+,M40",
    "FWD_R" : "M1+,M20,M30,M4+",
    "BKWD_L" : "M1-,M20,M30,M4-",
    "BKWD_R" : "M10,M2-,M3-,M40",
    "CCW" : "M1+,M2-,M3+,M4-",
    "CW" : "M1-,M2+,M3-,M4+",
    "STOP" : "M10,M20,M30,M40",
    "EXIT" : "EXIT"
}

def main(args=None):
    
    rclpy.init(args=args)

    direction_mapper = DirectionMapper()

    rclpy.spin(direction_mapper)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    direction_mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
