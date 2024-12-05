"""
subscribes to x, y, theta of the robot these will be updated in real time for the robot
subscribes to target_x, target_y we can publish ourselves

whenever target coordintes change:
    find a straight line from x,y to target_x, target_y
    align the robot with theta information
    drive from x,y to target_x, target_y
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, PoseStamped
from std_msgs.msg import Float32MultiArray
import math
from scipy.spatial.transform import Rotation as R
from gazebo_msgs.msg import ModelStates

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self.current_pose_subscription = self.create_subscription(
            PoseArray,
            '/gazebo/robot_pose',
            self.current_pose_callback,
            10
        )
        self.target_pose_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.target_callback,
            10
        )
        self.diff_drive_publisher = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)

        self.current_pose = None # (x, y, theta)
        self.target_pose = None # (target_x, target_y), ros2 topic pub /target_pose std_msgs/msg/Float32MultiArray "data: [target_x, target_y]"
       
        self.reached_target = True # if True we don't send any control signals

        self.k_theta = 1.0 # proportional controller constant omega = k_theta = k_theta * theta_error
        self.k_v = 3 # linear controller constant v = k_v * d_err
        # MAKE THESE CONTROLLER NON LINEAR X AXIS 0-1 AS PERCENTAGE OF REACHING TARGET AND Y AXIS 0 TO 1 AS PERCENTAGE OF MAX_SPEED
        print("initialised")
       

    def current_pose_callback(self, msg):
        pose = msg.poses[0]
        x = pose.position.x
        y = pose.position.y
           
        q = pose.orientation # Orientation is in quaternion coordinates
        rot = R.from_quat([q.x, q.y, q.z, q.w])
        euler = rot.as_euler('xyz', degrees=False)
        theta = euler[2] # Z-axis rotation (yaw)
               
        self.current_pose = (x, y, theta)
        self.get_logger().info(f"Robot Position: x={x}, y={y}, theta={theta}")
        return
   
    def target_callback(self, msg):
        target = msg.pose.position
        print(f"new target location received: {target}")
        self.target_pose = (target.x, target.y)
        self.reached_target = False # Since new target is set

    def control_loop(self):
        if self.target_pose is None:
            #self.get_logger().info("No target pose data received")
            return
        if self.current_pose is None:
            self.get_logger().info("ERROR: --------------- No current pose data")
            return
        self.get_logger().info("Processing target pose data")
       
        x, y, theta = self.current_pose
        x_target, y_target = self.target_pose

        dx = x_target - x
        dy = y_target - y
       
        # Check if we're at target
        dist_from_target = math.sqrt(dx*2 + dy*2)
        print(f"distance: {dist_from_target}")
        if dist_from_target < 0.1:
            self.stop_robot()
            self.reached_target = True
            return
       
        # Computing angular and linear velocity commands
        theta_desired = math.atan2(dy, dx)
        angle_error = self.normalize_angle(theta_desired - theta - 1.57079632)
        print(f"angle error: {angle_error}")
        omega = self.k_theta * angle_error

        v = self.k_v * dist_from_target if abs(angle_error) < 0.1 else 0.0 # Move only when aligned with target location

        self.get_logger().info(f"Sending twist message - x: {v} angle: {-omega}")
        # Publish command
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = -omega
        self.diff_drive_publisher.publish(twist)
       

    def stop_robot(self):
        self.get_logger().info("Stopping robot")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.diff_drive_publisher.publish(twist)

    # Normalize angle between +/-pi
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def spin(self):
        print("spinning")
        while rclpy.ok():
            rclpy.spin_once(self)
            self.control_loop()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
    try:
        node.spin()
    except KeyboardInterrupt:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

