import rclpy
import math
import re
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from mecanum_interfaces.msg import WheelSpeed

#Initial config
wheel_radius = 0.03 #m
length_x = 0.1 #m
length_y = 0.15 #m

class FwdKinematics(Node):

	def __init__(self):
		super().__init__('fwd_kinematics')
		self.subscription = self.create_subscription(WheelSpeed, 'wheels_speed', self.listener_callback, 10)
		self.subscription #prevent unused variable warning
		self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
		self.old_time = self.get_clock().now()
		self.br = TransformBroadcaster(self)
		self.x = 0.0
		self.y = 0.0
		self.th = 0.0

	def listener_callback(self, msg):

		
		new_time = self.get_clock().now()  #time.time_ns()
		old_time = self.old_time

		#Delta time
		dt = ((new_time.__sub__(old_time)).nanoseconds)/1000000000
		
		self.get_logger().debug('dt: ' + str(dt))

		#Transform to rad/s
		w1 = rpmToRads(msg.w1)
		w2 = rpmToRads(msg.w2)
		w3 = rpmToRads(msg.w3)
		w4 = rpmToRads(msg.w4)

		#Robot velocities
		vx = (wheel_radius/4) * (w1 + w2 + w3 + w4)
		vy = (wheel_radius/4) * (-w1 + w2 + w3 - w4)
		w = (wheel_radius/(4 * (length_x + length_y))) * (-w1 + w2 - w3 + w4)
		
		#Postition deltas
		dx = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
		dy = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
		dth = w * dt

		#Position estimation
		self.x = self.x + dx
		self.y = self.y + dy
		self.th = self.th + dth

		#Quaternion calculation
		quat = quaternion_from_euler(0, 0, self.th)

		# tf_broadcaster = TransformBroadcaster()

		# #Create and fill transform
		odom_tf = TransformStamped()

		odom_tf.header.stamp.nanosec = new_time.to_msg()
		odom_tf.header.frame_id = 'odom'
		odom_tf.child_frame_id = 'base_link'

		odom_tf.transform.translation.x = self.x
		odom_tf.transform.translation.y = self.y
		odom_tf.transform.translation.z = 0.0
		odom_tf.transform.rotation = quat

		# #Send transaform
		self.br.sendTransform(odom_tf)

		#Create and fill odometry
		odom = Odometry()
		odom.header.frame_id = "odom"
		odom.header.stamp =  self.get_clock().now().to_msg()
		odom.child_frame_id = "base_link"

		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation.x = quat[0]
		odom.pose.pose.orientation.y = quat[1]
		odom.pose.pose.orientation.z = quat[2]
		odom.pose.pose.orientation.w = quat[3]

		odom.twist.twist.linear.x = vx
		odom.twist.twist.linear.y = vy
		odom.twist.twist.angular.z = w

		#Publish odometry
		self.publisher_.publish(odom)

		self.old_time = new_time
		 
		
		

def rpmToRads(rpm):
	return rpm*math.pi/30

def main(args=None):

	rclpy.init(args=args)

	fwd_kinematics = FwdKinematics()

	fwd_kinematics.get_logger().info('Fwd Kinematics Ready')
	rclpy.spin(fwd_kinematics)

	fwd_kinematics.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
