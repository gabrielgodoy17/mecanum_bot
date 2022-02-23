from socket import MsgFlag
import spidev
import rclpy
import time
import re
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import LED

from mecanum_interfaces.msg import WheelSpeed

#Initial config

slave_select_1 = LED(17)
slave_select_2 = LED(27)
sent = False
last_msg_sent = " "

spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=244000
spi.mode = 0b00

class SpiInterface(Node):

	def __init__(self):
		super().__init__('spi_interface')
		self.subscription = self.create_subscription(WheelSpeed, 'motors', self.listener_callback, 10)
		self.subscription #prevent unused variable warning
		self.publisher_ = self.create_publisher(WheelSpeed, 'wheels_speed', 10)
		timer_period = 0.1  # seconds   
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		self.to_send_slave1 = ":w1000;:w2000;" #intialize with wheels stopped
		self.to_send_slave2 = ":w3000;:w4000;" #intialize with wheels stopped

	def timer_callback(self):
		msg = WheelSpeed()
		#regex_w1 = re.compile(r'(?<=w1\+)(\d{2})|(?<=w1)(\d{3})|(?<=w1-)(\d{2})')
		#regex_w2 = re.compile(r'(?<=w2\+)(\d{2})|(?<=w2)(\d{3})|(?<=w2-)(\d{2})')
		#to_send_slave1 = dict.get(to_send_spi[0:3]) + dict.get(to_send_spi[4:7])
		#Slave 1 spi
		slave_select_1.off()
		response = spi.xfer2(bytearray(self.to_send_slave1.encode()))
		#Process slave 1 response
		slave_1 = ''.join([str(chr(elem)) for elem in response])
		self.get_logger().info("recieve from slave1: "+slave_1)
		msg.w1 = float(slave_1[3:6])
		self.get_logger().info("w1: (only nums)"+slave_1[3:6])
		self.get_logger().info("w1: (float()) %f" %float(slave_1[3:6]))
		self.get_logger().info("w1: %f"%msg.w1)
		msg.w2 = float(slave_1[10:13])
		self.get_logger().info("w2: (only nums)"+slave_1[10:13])
		self.get_logger().info("w2: (float()) %f" %float(slave_1[10:13]))
		self.get_logger().info("w2: %f"%msg.w2)
		slave_select_1.on()
		#Slave 2 spi
		slave_select_2.off()
		response2 = spi.xfer2(bytearray(self.to_send_slave2.encode()))		
		#Process slave 2 response
		slave_2 = ''.join([str(chr(elem)) for elem in response2])
		self.get_logger().info("recieve from slave2: "+slave_2)
		msg.w3 = float(slave_2[3:6])
		self.get_logger().info("w3: (only nums)"+slave_2[3:6])
		self.get_logger().info("w3: (float()) %f" %float(slave_2[3:6]))
		self.get_logger().info("w3: %f"%msg.w3)

		msg.w4 = float(slave_2[10:13])
		self.get_logger().info("w4: (only nums)"+slave_2[10:13])
		self.get_logger().info("w4: (float()) %f" %float(slave_2[10:13]))
		self.get_logger().info("w4: %f"%msg.w4)
		slave_select_2.on()
		
		self.publisher_.publish(msg)
		self.get_logger().info('Publishing: "%s"' % msg)

	def listener_callback(self, msg):

		self.get_logger().info('Received from topic: %s' % msg)

		#Message slave 1

		self.to_send_slave1 = generate_command(msg.w1, 1) + generate_command(msg.w2, 2)
		self.get_logger().info('To send slave1: ' + self.to_send_slave1)

		#Message slave 2

		self.to_send_slave2 = generate_command(msg.w3, 1) + generate_command(msg.w4, 2)
		self.get_logger().info('To send slave2: ' + self.to_send_slave2)

def generate_command(wheel_speed, wheel_num):
	wheel_speed_int = int(round(wheel_speed))
	wheel_speed_string = str(abs(wheel_speed_int))
	sign = "0"

	if wheel_speed_int < 0:
		sign = "-"
	elif wheel_speed_int > 0:
		sign = "+"
	
	if len(wheel_speed_string) == 1:
		wheel_speed_string = "0" + wheel_speed_string
	elif len(wheel_speed_string) > 2: #Should never happen, just in case
		sign = "0"
		wheel_speed_string = "00"

	return ":w" + str(wheel_num) + sign + wheel_speed_string + ";"

def main(args=None):

	slave_select_1.on()
	slave_select_2.on()
	rclpy.init(args=args)

	spi_interface = SpiInterface()

	spi_interface.get_logger().info('Spi Interface READY')
	rclpy.spin(spi_interface)
	spi_interface.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
