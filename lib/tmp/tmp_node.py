#!/usr/bin/env python3
import time
from tmp102 import TMP102
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TmpPublisher(Node):

    def __init__(self):
        super().__init__('tmp_publisher')

        period = self.declare_parameter("period")
        self.timer_period = period.value
        units = self.declare_parameter("units")
        self.tmp_units = units.value

        self.publisher_ = self.create_publisher(String, 'tmp', 10)
        self.tmp = TMP102('C', 0x48, 1)
        self.tmp.setUnits(self.tmp_units)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "Current temp: {:.1f}deg".format(self.tmp.readTemperature()) + self.tmp_units
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    tmp_publisher = TmpPublisher()
    rclpy.spin(tmp_publisher)
    tmp_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
