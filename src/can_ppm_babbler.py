#!/usr/bin/env python3

# Short program for driving the robot arm using a keyboard.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from ros2_j1939_babbler_msgs.msg import ServoControl0

MIN_GRIPPER = 0.0
MAX_GRIPPER = 180.0

class CanPPMController(Node):
    def __init__(self):
        super().__init__('can_ppm_controller')
        self.declare_parameter("name", "device")
        self.declare_parameter("babbler_servo_id", 0)
        self.declare_parameter("src_addr", 0) 
        

        self.name = self.get_parameter('name').get_parameter_value().string_value
        self.cmd_subscribers = [None, None, None]
        # receive values between 0 and 1
        self.cmd_subscribers[0] = self.create_subscription(Float64, f'/ppm/{self.name}/ch0/command', lambda f: self.callback(0, f), 10)
        self.cmd_subscribers[1] = self.create_subscription(Float64, f'/ppm/{self.name}/ch1/command', lambda f: self.callback(1, f), 10)
        self.cmd_subscribers[2] = self.create_subscription(Float64, f'/ppm/{self.name}/ch2/command', lambda f: self.callback(2, f), 10)
        self.ppm_publisher = self.create_publisher(ServoControl0, '/arm/babbler/ServoControl0/tx', 10)
        self.cmd_state = [0.0, 0.0, 0.0]
        self.msg_count = [0,0,0]

    def callback(self, n: int, val: Float64):
        self.cmd_state[n] = val.data

        self.cmd_publish(n);

    def cmd_publish(self, n: int):
        msg = ServoControl0()
        msg.header.frame_id = "body"
        msg.header.stamp = self.get_clock().now().to_msg();
        msg.src_addr = 0 # TODO: What is a good source address?
        msg.index = n
        msg.message_counter = self.msg_count[n]
        self.msg_count[n] = (self.msg_count[n]+1) % 0xFB
        msg.crc = 0xFF
        msg.set_angle = self.cmd_state[n]

        self.ppm_publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    ppm = CanPPMController()
    rclpy.spin(ppm)
    ppm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()