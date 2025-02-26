import math
import rclpy
from rclpy.node import Node
from px4msgs.msg import Command
from std_msgs.msg import Bool
from rclpy.clock import Clock
from rclpy.time import Time

class ExampleTraj(Node):
    def __init__(self):
        super().__init__('traj_node')
        
        # Create publisher for position commands
        self.cmd_pub = self.create_publisher(
            Command, 
            '/drone/poscmd', 
            10
        )
        
        # Create subscriber for allow commands
        self.allow_sub = self.create_subscription(
            Bool,
            '/drone/allow_cmd',
            self.allow_callback,
            10
        )
        
        # Initialize variables
        self.allow_cmd = False
        self.start_time = None
        self.traj_started = False
        
        # Create timer for control loop (100Hz)
        self.timer = self.create_timer(0.01, self.run_loop)
        
        # Initialize trajectory generator
        self.current_traj = self.example_circle([0.0, 0.0], 3.0, 1.0)
        
        self.get_logger().info("Trajectory node initialized")

    def allow_callback(self, msg):
        """Handle allow command callback"""
        self.allow_cmd = msg.data
        if self.allow_cmd and not self.traj_started:
            self.get_logger().info("Trajectory started!")
            self.start_time = self.get_clock().now()
            self.traj_started = True
        elif not self.allow_cmd and self.traj_started:
            self.get_logger().info("Trajectory stopped")
            self.traj_started = False

    def run_loop(self):
        """Main control loop"""
        # self.get_logger().info(f"Current time: {self.get_clock().now().nanoseconds * 1e-9}")
        if self.allow_cmd and self.traj_started:
            # Calculate elapsed time
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            
            # Generate trajectory point
            x, y, z, vx, vy, vz, yaw = self.current_traj(elapsed_time)
            
            # Create and publish command
            cmd = Command()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.type = Command.DESIRED_POS
            cmd.pos = [float(x), float(y), float(z)]
            cmd.vel = [float(vx), float(vy), float(vz)]
            cmd.yaw = float(yaw)
            self.cmd_pub.publish(cmd)
        else:
            # Publish zero commands when not allowed
            cmd = Command()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.type = Command.THRUST_BODYRATE
            cmd.u = [0.0, 0.0, 0.0, 0.0]
            self.cmd_pub.publish(cmd)

    @staticmethod
    def example_circle(initpos, radius, height):
        """Generate circular trajectory"""
        start = [radius, 0.0, 1.0]
        offset_x = initpos[0] - start[0]
        offset_y = initpos[1] - start[1]
        
        def circle(t):
            x = offset_x + radius * math.cos(t)
            y = offset_y + radius * math.sin(t)
            z = height
            vx = -radius * math.sin(t)
            vy = radius * math.cos(t)
            vz = 0.0
            yaw = 0.0
            return x, y, z, vx, vy, vz, yaw
        
        return circle

    @staticmethod
    def example_lemniscate(initpos, radius, height):
        """Generate lemniscate (figure-8) trajectory"""
        start = [radius, 0.0, 1.0]
        offset_x = initpos[0] - start[0]
        offset_y = initpos[1] - start[1]
        
        def lemniscate(t):
            x = radius * math.cos(t) + offset_x
            y = (radius * math.sin(2 * t)) / 2 + offset_y
            z = height
            vx = -radius * math.sin(t)
            vy = radius * math.cos(2 * t)
            vz = 0.0
            yaw = 0.0
            return x, y, z, vx, vy, vz, yaw
        
        return lemniscate

def main(args=None):
    rclpy.init(args=args)
    
    try:
        traj_node = ExampleTraj()
        rclpy.spin(traj_node)
    except KeyboardInterrupt:
        pass
    finally:
        traj_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()