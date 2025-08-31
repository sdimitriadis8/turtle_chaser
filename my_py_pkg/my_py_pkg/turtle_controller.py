import rclpy
import random
from rclpy.node import Node
from turtlesim.srv import Spawn
from functools import partial
import colorama
from colorama import Fore, Style, Back
import math
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleControllerNode(Node):
    def __init__ (self):
        super().__init__("turtle_controller")
        self.get_logger().info(Fore.GREEN + "Turtle controller node has been started." + Fore.RESET)
        self.alive_turtles_atm_: TurtleArray = None
        self.current_pose_: Pose = None
        self.target_: Pose
        self.turtle_to_catch_: Turtle = None
        self.declare_parameter("catch_closest_turtle_first_","True")
        self.catch_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first_").value
        self.pose_subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_get_pose, 10)
        self.turtle_array_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_check_turtle_list, 10)
        # self.find_closest_turtle(self.current_pose_,self.alive_turtles_atm_)

        self.vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

        self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")
        

    def control_loop(self):
        if self.current_pose_ == None or self.turtle_to_catch_ == None:
            return
        
        dist_x = self.turtle_to_catch_.turtle_x - self.current_pose_.x
        dist_y = self.turtle_to_catch_.turtle_y - self.current_pose_.y

        distance = math.sqrt(math.pow(dist_x, 2) + math.pow(dist_y, 2))
        
        cmd = Twist()

        if distance > 0.5:
            
            goal_theta = math.atan2(dist_y,dist_x)
            ang_dif = goal_theta - self.current_pose_.theta

            # Transform angles within [-pi,pi]
            if ang_dif > math.pi:
                ang_dif -= 2*math.pi
            elif ang_dif < -math.pi:
                ang_dif += 2*math.pi

            cmd.linear.x = 2*distance
            cmd.angular.z = 6*ang_dif
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle(self.turtle_to_catch_.turtle_name)
            self.turtle_to_catch_ = None

        self.vel_publisher_.publish(cmd)

    def callback_check_turtle_list(self, turtle_list: TurtleArray):
        if len(turtle_list.turtles) > 0:
            self.get_logger().info("Retrieving list of alive turtles...")
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = None
                
                for turtle in turtle_list.turtles:
                    dist_x = turtle.turtle_x - self.current_pose_.x
                    dist_y = turtle.turtle_y - self.current_pose_.y
                    distance = math.sqrt(math.pow(dist_x, 2) + math.pow(dist_y, 2))
                    if closest_turtle_distance == None or closest_turtle_distance > distance:
                        closest_turtle_distance = distance
                        closest_turtle = turtle
                self.turtle_to_catch_ = closest_turtle

            else:
                self.alive_turtles_atm_ = turtle_list.turtles
                self.turtle_to_catch_ = self.alive_turtles_atm_[0]
            
        else:
            self.get_logger().info("The list of alive turtles is empty!")

    def callback_get_pose(self, pose: Pose):
        self.current_pose_ = pose
        self.get_logger().info("Getting current pose")
        print(self.current_pose_)

    def call_catch_turtle(self, turtle_name):
        while not self.catch_turtle_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for catch service...')
        
        request = CatchTurtle.Request()
        request.name = turtle_name
    
        future = self.catch_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_catch_turtle, request=request))
    
    def callback_call_catch_turtle(self, future, request: CatchTurtle.Request):
        response = CatchTurtle.Response()
        if not response.success:
                self.get_logger().error("Turtle " + str(request.name) + "could not be killed")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()