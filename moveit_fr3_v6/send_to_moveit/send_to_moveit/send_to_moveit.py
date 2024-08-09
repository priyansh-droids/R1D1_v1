import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander

class MoveItNode(Node):

    def __init__(self):
        super().__init__('hello_moveit')
        self.logger = self.get_logger()

        # Initialize MoveIt components
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.move_group = MoveGroupCommander('agv_arm')

        # Create a subscriber to the "agv_arm_pose" topic
        self.subscription = self.create_subscription(
            Pose,
            'agv_arm_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        # Set the target pose
        self.move_group.set_pose_target(msg)

        # Create a plan to that target pose
        success, plan, planning_time, error_code = self.move_group.plan()

        # Execute the plan
        if success:
            self.move_group.execute(plan, wait=True)
        else:
            self.logger.error('Planning failed!')

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)

    # Create the node
    node = MoveItNode()

    # Spin the node to process callbacks
    rclpy.spin(node)

    # Shutdown ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()
