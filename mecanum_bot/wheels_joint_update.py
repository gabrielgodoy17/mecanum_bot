import rclpy
import math
import re
from rclpy.node import Node
from sensor_msgs.msg import JointState
from mecanum_interfaces.msg import WheelSpeed



class WheelsJointUpdate(Node):

	def __init__(self):
		super().__init__('wheels_joint_update')
		self.subscription = self.create_subscription(WheelSpeed, 'wheels_speed', self.listener_callback, 10)
		self.subscription #prevent unused variable warning
		self.publisher_ = self.create_publisher(JointState, 'wheels_joint_state', 10)
		self.old_time = self.get_clock().now()
		
		# initial parameters for angular position
		self.theta1 = 0.0
		self.theta2 = 0.0
		self.theta3 = 0.0
		self.theta4 = 0.0 

	def listener_callback(self, msg):

		new_time = self.get_clock().now()  #time.time_ns()
		old_time = self.old_time

		#Delta time
		dt = ((new_time.__sub__(old_time)).nanoseconds)/1000000000
	
		#Read topic
		wheel_speed=str(msg.data)

		#verify data pattern with regex
		regex= "[^+\-0-9w]"
		result = re.search(regex,wheel_speed)
		if result is None : 
		
			#Separate wheel speeds
			w1 = int(wheel_speed[2:5])
			w2 = int(wheel_speed[7:10])
			w3 = int(wheel_speed[12:15])
			w4 = int(wheel_speed[17:])

			#Transaform to rad/s
			w1 = rpmToRads(w1)
			w2 = rpmToRads(w2)
			w3 = rpmToRads(w3)
			w4 = rpmToRads(w4)

			self.theta1 = self.theta1 + w1 * dt
			self.theta2 = self.theta2 + w2 * dt
			self.theta3 = self.theta3 + w3 * dt
			self.theta4 = self.theta4 + w4 * dt
			
			jointStates = JointState()
			jointStates.header.frame_id = "odom"
			jointStates.header.stamp =  self.get_clock().now().to_msg()

			jointStates.name = ['drivewhl_1_joint', 'drivewhl_2_joint', 'drivewhl_3_joint', 'drivewhl_4_joint']
			jointStates.position = [self.theta1, self.theta2, self.theta3, self.theta4]
			jointStates.velocity = [w1, w2, w3, w4]

			#Publish jointStates
			self.publisher_.publish(jointStates)

		self.old_time = new_time
		 
		
		

def rpmToRads(rpm):
	return rpm*math.pi/30

def main(args=None):

	rclpy.init(args=args)

	wheels_joint_update = WheelsJointUpdate()

	print('wheels_joint_update ---> READY')
	rclpy.spin(wheels_joint_update)

	wheels_joint_update.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
