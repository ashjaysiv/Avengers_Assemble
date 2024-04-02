import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
import math
from rclpy.node import Node
from rclpy.time import Duration
from launch import LaunchDescription
from launch_ros.actions import Node as RosNode
import os
import subprocess
import time

class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')

    def launch_turtlesim(self):
        ld = LaunchDescription()
        ld.add_action(RosNode(package='turtlesim', executable='turtlesim_node', output='screen'))
        return ld

    def close_turtlesim(self):
        os.system('ros2 node kill /turtlesim')

class TurtleLineFollower:
    def __init__(self):
        self.goalie_pose = None
        self.ball_pose = None
        self.player_1_pose = None

        self.node = rclpy.create_node('turtle_line_follower')
        self.spawn_client = self.node.create_client(Spawn, '/spawn')
        self.kill_client = self.node.create_client(Kill, '/kill')
        
        self.goalie_velocity_publisher = self.node.create_publisher(Twist, '/goalie/cmd_vel', 10)
        self.goalie_pose_subscriber = self.node.create_subscription(Pose, '/goalie/pose', self.goalie_pose_callback, 10)

        self.ball_pose_subscriber = self.node.create_subscription(Pose, '/ball/pose', self.ball_pose_callback, 10)
        self.ball_velocity_publisher = self.node.create_publisher(Twist, '/ball/cmd_vel', 10)

        self.player_1_pose_subscriber = self.node.create_subscription(Pose, '/player_1/pose', self.player_1_pose_callback, 10)
        self.player_1_velocity_subscriber = self.node.create_subscription(Twist, '/player_1/cmd_vel', self.player_1_vel_callback, 10)


        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('spawn service not available, waiting again...')

        self.kill_turtle('turtle1')
        self.spawn_player(3.5, 1.0, math.pi/2, 'goalie')
        self.spawn_player(5.500, 8.5, -math.pi/2, 'player_1')
        self.spawn_player(9.0, 3.0, -math.pi/2, 'player_2')
        self.spawn_player(5.500, 5.5, -math.pi/2, 'ball')
    
    def restart_game(self):
        self.kill_turtle('goalie')
        self.kill_turtle('player_1')
        self.kill_turtle('player_2')
        self.kill_turtle('ball')

        cmd = ['ros2', 'service', 'call', '/clear', 'std_srvs/srv/Empty']
        subprocess.run(cmd)


        self.spawn_player(3.5, 1.0, math.pi/2, 'goalie')
        self.spawn_player(5.5, 8.5, -math.pi/2, 'player_1')
        self.spawn_player(9.0, 3.0, -math.pi/2, 'player_2')
        self.spawn_player(5.5, 5.5, -math.pi/2, 'ball')

    def spawn_player(self, x, y, t, name):
        request = Spawn.Request()
        request.x, request.y, request.theta, request.name = x, y, t, name
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info('Turtle spawned successfully')
        else:
            self.node.get_logger().error('Failed to spawn turtle')

    def kill_turtle(self, name):
        client = self.node.create_client(Kill, '/kill')
        request = Kill.Request()
        request.name = name

        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)



    def goalie_pose_callback(self, pose_msg):
        self.goalie_pose = pose_msg

    def ball_pose_callback(self, pose_msg):
        self.ball_pose = pose_msg

    def player_1_pose_callback(self, pose_msg):
        self.player_1_pose = pose_msg

    def player_1_vel_callback(self, vel_msg):
        self.player_1_vel = vel_msg

    def move_goalie(self, twist):

        if self.goalie_pose is not None:

            twist.linear.y = 1.0 * self.direction
            # twist.angular.z = 0.0

            # Publish the twist message
            self.goalie_velocity_publisher.publish(twist)

            # Check if the turtle has reached close to the target
            if self.goalie_pose.x <= 3.5 or self.goalie_pose.x >= 7.5:
                # self.make_180_degree_tur/n(twist)
                self.direction *= -1
                while self.goalie_pose.x <= 3.5 or self.goalie_pose.x >= 7.5:
                    twist.linear.y = 1.0 * self.direction
                    self.goalie_velocity_publisher.publish(twist)
                    rclpy.spin_once(self.node)

        rclpy.spin_once(self.node)

    

    def ball_follower(self):

        dist_player_1_ball = math.sqrt((self.player_1_pose.x - self.ball_pose.x)**2 + (self.player_1_pose.y - self.ball_pose.y)**2)
        if dist_player_1_ball < 1:
            self.ball_velocity_publisher.publish(self.player_1_vel)
            rclpy.spin_once(self.node)

    def goalie_catch(self):
        dist_goalie_ball = math.sqrt((self.goalie_pose.x - self.ball_pose.x)**2 + (self.goalie_pose.y - self.ball_pose.y)**2)
        if dist_goalie_ball < 1:
            self.restart_game()


    def goal_success(self):
        if self.ball_pose.y < 1:
            self.node.get_logger().info('You win!')
            return True

    def run(self):

        twist = Twist()
        self.direction = 1.0

        while rclpy.ok():
            self.move_goalie(twist)
            if self.player_1_pose and self.ball_pose and self.goalie_pose:
                self.ball_follower()
                self.goalie_catch()
                if self.goal_success():
                    cmd = ['pkill', 'turtlesim']
                    subprocess.run(cmd)
                    break

        # Shutdown the node
        self.node.destroy_node()
        rclpy.shutdown()

def main(args=None):

    cmd = ['ros2', 'run', 'turtlesim', 'turtlesim_node']
    subprocess.Popen(cmd)
    
    rclpy.init(args=args)

    turtle_line_follower = TurtleLineFollower()
    turtle_line_follower.run()

    


if __name__ == '__main__':
    main()
