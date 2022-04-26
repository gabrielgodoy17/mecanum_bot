import rclpy
import math
import re
from rclpy.node import Node
from std_msgs.msg import String

from mecanum_interfaces.msg import WheelSpeed, MsgStats

#Initial config
wheel_radius = 0.03 #m
length_x = 0.1 #m
length_y = 0.15 #m

class WheelSpeedBroadcaster(Node):

	def __init__(self):
		super().__init__('wheel_speed_broadcaster')
		self.subscription = self.create_subscription(String, 'wheels_speed_raw', self.listener_callback, 10)
		self.subscription #prevent unused variable warning
		self.publisher_ = self.create_publisher(WheelSpeed, 'wheels_speed_filtered', 10)
		self.publisher_2 = self.create_publisher(MsgStats, 'wheels_speed_stats', 10)
		self.old_time = self.get_clock().now()
		self.w1 = 0.0
		self.w2 = 0.0
		self.w3 = 0.0
		self.w4 = 0.0
		self.msg_count = 0
		self.msg_success_count = 0

	def listener_callback(self, msg):
		self.msg_count = self.msg_count + 1

		#Read topic
		wheel_speed=str(msg.data)

		#verify data pattern with regex
		regex= "[^+\-0-9w]"
		result = re.search(regex,wheel_speed)
		if result is None and len(wheel_speed) == 20: 
			self.msg_success_count = self.msg_success_count + 1

			#Separate wheel speeds
			self.w1 = float(wheel_speed[2:5])
			self.w2 = float(wheel_speed[7:10])
			self.w3 = float(wheel_speed[12:15])
			self.w4 = float(wheel_speed[17:])

		to_send = WheelSpeed()

		to_send.w1 = self.w1
		to_send.w2 = self.w2
		to_send.w3 = self.w3
		to_send.w4 = self.w4

		#Publish odometry
		self.publisher_.publish(to_send)

		#Publish stats
		self.publish_stats()

	def publish_stats(self):
		error_count = self.msg_count - self.msg_success_count
		if self.msg_count != 0:
			error_rate = error_count / self.msg_count
			success_rate = self.msg_success_count / self.msg_count
		else:
			error_rate = 0.0
			success_rate = 0.0
		msg_stats = MsgStats()
		msg_stats.total = self.msg_count
		msg_stats.success_count = self.msg_success_count
		msg_stats.error_count = error_count
		msg_stats.error_rate = error_rate
		msg_stats.success_rate = success_rate
		self.publisher_2.publish(msg_stats)
		
		

def rpmToRads(rpm):
	return rpm*math.pi/30

def main(args=None):

	rclpy.init(args=args)

	wheel_speed_broadcaster = WheelSpeedBroadcaster()

	wheel_speed_broadcaster.get_logger().info('Wheel Speed Broadcaster Ready')
	rclpy.spin(wheel_speed_broadcaster)

	wheel_speed_broadcaster.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
