#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from functools import partial
import colorama
from colorama import Fore, Style, Back

class MyBatteryClient(Node): 
    def __init__(self):
        super().__init__("my_battery_client")
        self.battery_state_ = "full"
        self.last_time_battery_state_changed_ = self.get_current_time_seconds()
        self.battery_timer_ = self.create_timer(0.1, self.check_battery_state)
        self.set_led_client_ = self.create_client(SetLed, "set_led")
        self.get_logger().info(Fore.YELLOW + "My battery state client has been started.")

    def get_current_time_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1000000000.0    

    def check_battery_state(self):
        time_now = self.get_current_time_seconds()
        if self.battery_state_ == "full":
            if time_now - self.last_time_battery_state_changed_ > 4.0 :
                self.battery_state_ = "empty"
                self.get_logger().info(Fore.RED + Style.BRIGHT + "Battery is empty. Charging...")
                self.call_set_led(2, 1)
                self.last_time_battery_state_changed_ = time_now
        elif self.battery_state_ == "empty":
            if time_now - self.last_time_battery_state_changed_ > 6.0 :
                self.battery_state_ = "full"
                self.get_logger().info(Fore.GREEN + "Battery is full!")
                self.call_set_led(2, 0)
                self.last_time_battery_state_changed_ = time_now

        
    def call_set_led(self, lednumber, ledstate):
        while not self.set_led_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Set Led service...")
        
        request = SetLed.Request()
        request.led_number = lednumber
        request.led_state = ledstate

        future = self.set_led_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_set_led, request=request))

    def callback_call_set_led(self, future, request):
        response: SetLed.Response = future.result()
        self.get_logger().info("Has LED number" + str(request.led_number) + " changed to " +
                               str(request.led_state) + " ?  Response: " + str(response.success))

def main(args=None):
    rclpy.init(args=args)
    node = MyBatteryClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
