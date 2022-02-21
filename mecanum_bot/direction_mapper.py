import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mecanum_interfaces.msg import WheelSpeed

POS_SPEED = 25.0
NEG_SPEED = -25.0
ZERO_SPEED = 0.0

class DirectionMapper(Node):

    def __init__(self):
        super().__init__('direction_mapper')
        self.subscription = self.create_subscription(String, 'directions', self.listener_callback, 10)
        self.subscription
        self.publisher_ = self.create_publisher(WheelSpeed, 'motors', 10)
        

    def listener_callback(self, msg):
        self.get_logger().debug('Received: "%s"' % msg.data)
        to_send = WheelSpeed()
        to_send = dict.get(msg.data)
        if to_send is None:
            to_send = generate_msg(ZERO_SPEED, ZERO_SPEED, ZERO_SPEED, ZERO_SPEED)
        self.get_logger().debug('Publishing: "%s"' % to_send)
        self.publisher_.publish(to_send)  

def generate_msg(w1, w2, w3, w4):
    msg = WheelSpeed()
    msg.w1 = w1
    msg.w2 = w2
    msg.w3 = w3
    msg.w4 = w4
    return msg

dict = {
    "FWD" : generate_msg(POS_SPEED, POS_SPEED, POS_SPEED, POS_SPEED), #M1+,M2+,M3+,M4+
    "BKWD" : generate_msg(NEG_SPEED, NEG_SPEED, NEG_SPEED, NEG_SPEED), #M1-,M2-,M3-,M4-
    "L" : generate_msg(NEG_SPEED, POS_SPEED, POS_SPEED, NEG_SPEED), #M1-,M2+,M3+,M4-
    "R" : generate_msg(POS_SPEED, NEG_SPEED, NEG_SPEED, POS_SPEED), #M1+,M2-,M3-,M4+
    "FWD_L" : generate_msg(ZERO_SPEED, POS_SPEED, POS_SPEED, ZERO_SPEED), #M10,M2+,M3+,M40
    "FWD_R" : generate_msg(POS_SPEED, ZERO_SPEED, ZERO_SPEED, POS_SPEED), #M1+,M20,M30,M4+
    "BKWD_L" : generate_msg(NEG_SPEED, ZERO_SPEED, ZERO_SPEED, NEG_SPEED), #M1-,M20,M30,M4-
    "BKWD_R" : generate_msg(ZERO_SPEED, NEG_SPEED, NEG_SPEED, ZERO_SPEED), #M10,M2-,M3-,M40
    "CCW" : generate_msg(POS_SPEED, NEG_SPEED, POS_SPEED, NEG_SPEED), #M1+,M2-,M3+,M4-
    "CW" : generate_msg(NEG_SPEED, POS_SPEED, NEG_SPEED, POS_SPEED), #M1-,M2+,M3-,M4+
    "STOP" : generate_msg(ZERO_SPEED, ZERO_SPEED, ZERO_SPEED, ZERO_SPEED), #M10,M20,M30,M40
}

def main(args=None):
    
    rclpy.init(args=args)

    direction_mapper = DirectionMapper()
    direction_mapper.get_logger().info("Direction Mapper ready")
    rclpy.spin(direction_mapper)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    direction_mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
