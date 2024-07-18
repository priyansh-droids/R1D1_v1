import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/slider_controller/joint_trajectory', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('JointTrajectory publisher node has been started.')
        self.start_time = time.time()

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = ['slider']  # Replace with your joint names
        
        point = JointTrajectoryPoint()
        current_time = time.time() - self.start_time
        amplitude = 0.6  # (0.5 - (-0.7)) / 2
        offset = -0.1  # (0.5 + (-0.7)) / 2

        position = amplitude * math.sin(current_time) + offset
        point.positions = [position ]  # Apply the same oscillation to both joints
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 second

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing JointTrajectory with positions: {point.positions}')

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_publisher = JointTrajectoryPublisher()
    rclpy.spin(joint_trajectory_publisher)

    # Destroy the node explicitly (optional)
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
