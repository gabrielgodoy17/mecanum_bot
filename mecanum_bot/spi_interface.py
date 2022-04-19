import spidev
import rclpy
import re
from rclpy.node import Node
from gpiozero import LED
from std_msgs.msg import String

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

# regex_w1 = re.compile(r'(?<=w1)(\+\d{2})|(?<=w1)(\d{3})|(?<=w1)(-\d{2})')
# regex_w2 = re.compile(r'(?<=w2)(\+\d{2})|(?<=w2)(\d{3})|(?<=w2)(-\d{2})')

class SpiInterface(Node):

	def __init__(self):
		super().__init__('spi_interface')
		self.subscription = self.create_subscription(WheelSpeed, 'motors', self.listener_callback, 10)
		self.subscription #prevent unused variable warning
		self.publisher_ = self.create_publisher(String, 'wheels_speed', 10)
		timer_period = 0.005  # seconds   
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		self.to_send_slave1 = ":w1000;:w2000;" #intialize with wheels stopped
		self.to_send_slave2 = ":w3000;:w4000;" #intialize with wheels stopped

	def timer_callback(self):
		msg = WheelSpeed()

		# #Slave 1 spi
		# self.get_logger().debug("to_send_slave1: "+self.to_send_slave1)
		# slave_select_1.off()
		# response = spi.xfer2(bytearray(self.to_send_slave1.encode(encoding='UTF-8')))
		# #Process slave 1 response
		# slave_1 = ''.join([str(chr(elem)) for elem in response])
		# self.get_logger().debug("recieve from slave1: "+slave_1)
		# w1 = regex_w1.search(slave_1)
		# w2 = regex_w2.search(slave_1)
		# slave_select_1.on()

		# #Slave 2 spi
		# self.get_logger().debug("to_send_slave2: "+self.to_send_slave2)
		# slave_select_2.off()
		# response2 = spi.xfer2(bytearray(self.to_send_slave2.encode(encoding='UTF-8')))		
		# #Process slave 2 response
		# slave_2 = ''.join([str(chr(elem)) for elem in response2])
		# self.get_logger().debug("recieve from slave2: "+slave_2)
		# w3 = regex_w1.search(slave_2)
		# w4 = regex_w2.search(slave_2)
		# slave_select_2.on()


		#Slave 1 spi
		#self.get_logger().debug("to_send_slave1: "+self.to_send_slave1)
		slave_select_1.off()
		response = spi.xfer2(bytearray(self.to_send_slave1.encode(encoding='UTF-8')))
		slave_1 = ''.join([str(chr(elem)) for elem in response])
		slave_1 = slave_1.replace(":","")
		slave_1 = slave_1.replace(";","")
		#self.get_logger().debug(slave_1)
		slave_select_1.on()

		#Slave 2 spi
		#self.get_logger().debug("to_send_slave2: "+self.to_send_slave2)
		slave_select_2.off()
		response2 = spi.xfer2(bytearray(self.to_send_slave2.encode(encoding='UTF-8')))		
		slave_2 = ''.join([str(chr(elem)) for elem in response2])
		slave_2 = slave_2[1:2] + "3" + slave_2[3:6] + slave_2[8:9] + "4" + slave_2[10:13]
		#self.get_logger().debug(slave_2)
		slave_select_2.on()

		to_send_pub = slave_1 + slave_2

		msg = String()
		msg.data= to_send_pub
		self.publisher_.publish(msg)
		#self.get_logger().debug('Publishing: "%s"' % msg.data)
		
		# if w1 and w2 and w3 and w4:
		# 	msg.w1 = float(w1.group())
		# 	msg.w2 = float(w2.group())
		# 	msg.w3 = float(w3.group())
		# 	msg.w4 = float(w4.group())
		# 	self.publisher_.publish(msg)
		# 	self.get_logger().debug('Publishing: "%s"' % msg)

	def listener_callback(self, msg):

		self.get_logger().debug('Received from topic: %s' % msg)

		#Message slave 1
		self.to_send_slave1 = generate_command(msg.w1, 1,self) + generate_command(msg.w2, 2,self)

		#Message slave 2
		self.to_send_slave2 = generate_command(msg.w3, 1,self) + generate_command(msg.w4, 2,self)
		


def generate_command(wheel_speed, wheel_num,self):
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
