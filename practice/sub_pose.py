import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv

class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('pose_subscriber')
        self.get_logger().info('Started Pose Subscriber')

        init_pose = PoseStamped()
        init_pose.pose.position.x = 0.2
        init_pose.pose.position.z = 0.2

        self.fixed_orient=[0.0037, -1.0000,  0.0000,  0.0000]


        self.save_pose_to_csv(init_pose.pose)

        self.get_logger().info('Saved initial pose')


        self.subscription = self.create_subscription(
            PoseStamped,
            'ee_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.previous_pose = None


    def listener_callback(self, msg):
        current_pose = msg.pose
        if self.previous_pose is None or not self.is_pose_equal(self.previous_pose, current_pose):
            self.get_logger().info(f'Received new pose: {current_pose}')
            self.save_pose_to_csv(current_pose)
            self.previous_pose = current_pose

    def is_pose_equal(self, pose1, pose2):
        return (pose1.position.x == pose2.position.x and
                pose1.position.y == pose2.position.y and
                pose1.position.z == pose2.position.z and
                pose1.orientation.x == pose2.orientation.x and
                pose1.orientation.y == pose2.orientation.y and
                pose1.orientation.z == pose2.orientation.z and
                pose1.orientation.w == pose2.orientation.w)

    def save_pose_to_csv(self, pose):
        with open('/workspace/isaaclab/goal.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            #writer.writerow([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            writer.writerow([pose.position.x, pose.position.y, pose.position.z, self.fixed_orient[0], self.fixed_orient[1], self.fixed_orient[2], self.fixed_orient[3]])

        self.get_logger().info('Pose saved to goal.csv')

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()