#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedPanelState
from my_robot_interfaces.srv import SetLed
import colorama
from colorama import Fore, Style, Back

colorama.init(autoreset=True)


class LEDPanelNode(Node): 
    def __init__(self):
        super().__init__("led_panel")
        self.declare_parameter("led_states", [0, 0, 0])
        self.led_states_ = self.get_parameter("led_states").value
        self.led_states_pub_ = self.create_publisher(LedPanelState, "led_panel_state", 10)
        self.led_states_timer_ = self.create_timer(5.0, self.publish_led_states)
        self.set_led_service_ = self.create_service(SetLed, "set_led", self.callback_set_led)
        self.get_logger().info(Fore.YELLOW + "LED panel node has been started")


    def publish_led_states(self):
        msg = LedPanelState()
        msg._led_states = self.led_states_
        self.led_states_pub_.publish(msg)

    def callback_set_led(self, request: SetLed.Request, response: SetLed.Response):
        led_number = request.led_number
        state = request.led_state

        if(led_number < 0 or led_number > 2):
            response.success = False
            self.get_logger().info(Fore.RED + Style.BRIGHT + "LED number out of range [0,2]")
            return response
        
        if(state not in [0, 1]):
            response.success = False
            self.get_logger().info(Fore.RED + Style.BRIGHT + "Invalid state input (should be 0/1)")
            return response

        self.led_states_[led_number] = state
        self.publish_led_states()
        response.success = True
        self.get_logger().info(Fore.GREEN + "LED state has been changed to " + str(state))
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LEDPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
