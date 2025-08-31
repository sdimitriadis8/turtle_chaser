#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed


class MyLedPanelServerNode(Node): 
    def __init__(self):
        super().__init__("my_led_panel_server")        
        self.server_ = self.create_service(
            SetLed, "set_led", self.callback_call_my_led_panel)
        self.get_logger().info("My LED Panel server has been started.")


    def callback_call_my_led_panel(self, request: SetLed.Request, response: SetLed.Response):
        if(request.led_state == True):
            if(request.led_number == 3):
                response.success =  True
                response.debug_message = " LED light number 3 was turned on"
        else:
            if(request.led_number == 3):
                response.success =  False
                response.debug_message = " LED light number 3 was turned off"
        self.get_logger().info("End of callback")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MyLedPanelServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
