import rclpy
import random
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from functools import partial
import colorama
from colorama import Fore, Style, Back
import math
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node):
    def __init__ (self):
        super().__init__("turtle_spawner")
        self.spawn_turtle_client_ = self.create_client(Spawn, "spawn")
        self.get_logger().info(Fore.GREEN + "Turtle spawner node has been started." + Fore.RESET)
        self.alive_turtles_ = []
        self.turtle_counter = 1
        self.declare_parameter("turtle_name_prefix","turtle")
        self.turtle_name_prefix = self.get_parameter("turtle_name_prefix").value
        self.declare_parameter("spawn_frequency",1.0)
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        
        self.turtle_spawn_timer_ = self.create_timer(1.0/self.spawn_frequency_, self.create_turtle)
        self.alive_turtles_publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10)

        self.kill_turtle_client = self.create_client(Kill, "/kill")
        
        self.catch_turtle_service_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)


    def callback_catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        
        self.kill_turtle(request.name)
               
        response.success = True 
        return response
        
    def kill_turtle(self, turtle_name):
        while not self.kill_turtle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        
        request = Kill.Request()
        request.name = turtle_name
    
        future = self.kill_turtle_client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill_turtle, request=request))
    
    def callback_call_kill_turtle(self, future, request: Kill.Request):
        for (i, turtle) in enumerate(self.alive_turtles_):
            if turtle.turtle_name == request.name:
                self.get_logger().info("Turtle " + str(request.name) + " has been killed successfully")
                del self.alive_turtles_[i]
                self.publish_alive_turtles_array
                break

    def publish_alive_turtles_array(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    def create_turtle(self):
        self.turtle_counter += 1
        name = self.turtle_name_prefix + str(self.turtle_counter)
        x_ = round(random.uniform(0.0, 11.0), 6)
        y_ = round(random.uniform(0.0, 11.0), 6)
        theta_ = round(random.uniform(-math.pi, math.pi), 6)
        self.call_spawn_turtle(x_,y_,theta_,name)


    def call_spawn_turtle(self, x_coord, y_coord, theta_orient, name_par):
        while not self.spawn_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service spawn to request...")
        
        request = Spawn.Request()
        request.x = x_coord
        request.y = y_coord
        request.theta = theta_orient
        request.name = name_par

        future = self.spawn_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn_turtle, request=request))

    def callback_call_spawn_turtle(self, future, request: Spawn.Request):
        response: Spawn.Response = future.result()
        if response.name != "":
            self.get_logger().info("Spawning new turtle at (" + str(request.x) + "," + str(request.y) + "," +
                               str(request.theta) + ") with name " + response.name)
            new_turtle = Turtle()
            new_turtle.turtle_x = request.x
            new_turtle.turtle_y = request.y
            new_turtle.turtle_theta = request.theta
            new_turtle.turtle_name = response.name

        self.alive_turtles_.append(new_turtle)
        self.publish_alive_turtles_array()
        
    def call_kill_turtle(self, turtle_name):
        request = turtle_name
        while not self.spawn_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service spawn to request...")

        future = self.spawn_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn_turtle, request=request))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
