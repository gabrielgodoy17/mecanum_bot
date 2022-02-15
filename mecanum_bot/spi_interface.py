import spidev
import rclpy
import time
import re
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import LED

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
		self.subscription = self.create_subscription(String, 'motors', self.listener_callback, 10)
		self.subscription #prevent unused variable warning
		self.publisher_ = self.create_publisher(String, 'wheels_speed', 10)

	def listener_callback(self, msg):

		to_send_spi=msg.data
		self.get_logger().info('Received from topic: %s' % to_send_spi)

		#Message slave 1

		to_send_slave1 = dict.get(to_send_spi[0:3]) + dict.get(to_send_spi[4:7])
		self.get_logger().info('To send slave1: %s' % to_send_slave1)

		#Message slave 2

		to_send_slave2 = dict.get(to_send_spi[8:11]) + dict.get(to_send_spi[12:15])
		self.get_logger().info('To send slave2: %s' % to_send_slave2)

		#Slave 1 spi
		slave_select_1.off()
		response = spi.xfer2(bytearray(to_send_slave1.encode()))
		slave_1 = ''.join([str(chr(elem)) for elem in response])
		slave_1 = slave_1.replace(":","")
		slave_1 = slave_1.replace(";","")
		self.get_logger().info(slave_1)
		slave_select_1.on()

		#Slave 2 spi
		slave_select_2.off()
		response2 = spi.xfer2(bytearray(to_send_slave2.encode()))
		slave_2 = ''.join([str(chr(elem)) for elem in response2])
		slave_2 = slave_2[1:2] + "3" + slave_2[3:6] + slave_2[8:9] + "4" + slave_2[10:13]
		self.get_logger().info(slave_2)
		slave_select_2.on()

		to_send_pub = slave_1 + slave_2

		msg = String()
		msg.data= to_send_pub
		self.publisher_.publish(msg)
		self.get_logger().info('Publishing: "%s"' % msg.data)


dict = {
	"M1+":":w1+25;",
	"M10":":w1000;",
	"M1-":":w1-25;",

	"M2+":":w2+25;",
	"M20":":w2000;",
	"M2-":":w2-25;",

	"M3+":":w1+25;",
	"M30":":w1000;",
	"M3-":":w1-25;",

	"M4+":":w2+25;",
	"M40":":w2000;",
	"M4-":":w2-25;"
}


def main(args=None):

	slave_select_1.on()
	slave_select_2.on()
	rclpy.init(args=args)

	spi_interface = SpiInterface()

	print('READY')
	rclpy.spin(spi_interface)
	spi_interface.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
