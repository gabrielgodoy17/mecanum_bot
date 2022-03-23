import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf_transformations import quaternion_from_euler

from mecanum_interfaces.msg import WheelSpeed

#Initial config
wheel_radius = 0.03 #m
length_x = 0.1 #m
length_y = 0.15 #m

class InvKinematics(Node):

	def __init__(self):
		super().__init__('inv_kinematics')
		self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
		self.subscription #prevent unused variable warning
		self.publisher_ = self.create_publisher(WheelSpeed, 'motors', 10)
		self.w1 = 0.0
		self.w2 = 0.0
		self.w3 = 0.0
		self.w4 = 0.0

	def listener_callback(self, msg):
		
		#Separate wheel speeds
		vx = msg.linear[0]
		vy = msg.linear[1]
		w = msg.angular[2]

		#Wheel velocities
		
		self.w1 = (1/wheel_radius) * (vx - vy - (length_x + length_y) * w)
		self.w2 = (1/wheel_radius) * (vx + vy + (length_x + length_y) * w)
		self.w3 = (1/wheel_radius) * (vx + vy - (length_x + length_y) * w)
		self.w4 = (1/wheel_radius) * (vx - vy + (length_x + length_y) * w)
		
		setPoint = WheelSpeed()
		setPoint.w1 = radsToRpm(self.w1)
		setPoint.w2 = radsToRpm(self.w2)
		setPoint.w3 = radsToRpm(self.w3)
		setPoint.w4 = radsToRpm(self.w4)

		#Publish odometry
		self.publisher_.publish(setPoint)

def radsToRpm(rads):
	return rads * (30/math.pi)

def main(args=None):

	rclpy.init(args=args)

	inv_kinematics = InvKinematics()

	inv_kinematics.get_logger().info('Inv Kinematics Ready')
	rclpy.spin(inv_kinematics)

	inv_kinematics.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
