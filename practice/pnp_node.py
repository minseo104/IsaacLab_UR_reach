#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
#from gripper_srv.srv import GripperService
import time
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException

from std_msgs.msg import String, Int32

import threading


class PnpNode(Node):

    def __init__(self):
        super().__init__('pnp_node')

        # Create a service client for /gripper_service
        #self.gripper_client = self.create_client(GripperService, '/gripper_service')

        # Initialize tf_buffer and tf_listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.color = "red"

        #while not self.gripper_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('/gripper_service not available, waiting...')
        # Create a publisher for send_gripper_cmd topic
        self.gripper_cmd_publisher = self.create_publisher(Int32, 'send_gripper_cmd', 10)

        # Create a publisher for ee_pose topic
        self.end_pose = PoseStamped()
        self.ee_pose_publisher = self.create_publisher(PoseStamped, 'ee_pose', 10)
        # Create a timer to periodically call the callback function for getting the object pose
        self.timer = self.create_timer(1.0, self.timer_callback)
        # Start the pick and place sequence
        #self.pick_and_place_sequence()
        # Create a subscriber for /object_color topic
        self.color_subscriber = self.create_subscription(
            String,
            '/object_color',
            self.color_callback,
            10
        )

    def color_callback(self, msg):
        self.color = msg.data
        #self.get_logger().info(f'Received object color: {msg.data}')
    def timer_callback(self):
        #self.get_logger().info('Timer callback...')
        self.get_object_pose()



    def get_object_pose(self):

        #while rclpy.ok():
            try:

                self.get_logger().info('Getting object pose...')

                # Lookup the transform from base_link to object
                transform = self.tf_buffer.lookup_transform('tool0_controller', 'object', rclpy.time.Time())
                translation = transform.transform.translation

                # Publish the pose with the translation and zero orientation
                self.update_pick_pose(translation.x, translation.y, translation.z)
                #return [translation.x-0.18, translation.y, 0.069]
            except (LookupException, ConnectivityException, ExtrapolationException):
                # If there is no transform, set everything to zero
                self.update_pick_pose(0.0, 0.0, 0.069)
                #return [0.0-0.18, 0.0, 0.069]
            #time.sleep(0.5)


    def call_gripper_service(self, position):
    
        # Publish the gripper command
        gripper_msg = Int32()
        gripper_msg.data = position
        self.gripper_cmd_publisher.publish(gripper_msg)
        self.get_logger().info(f'Gripper command sent: {position}')

        time.sleep(2.0)

    def update_pick_pose(self, x, y, z):
        msg = PoseStamped()
        msg.pose.position.x = x-0.18
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.end_pose = msg
        #self.get_logger().info(f'Published Object pose: {self.end_pose.pose}')


    def publish_pose(self, x, y, z):
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.get_logger().info(f'Published EE pose: {msg.pose}')
        self.ee_pose_publisher.publish(msg)

    def pick_and_place_sequence(self):
        while rclpy.ok():
            delay = 3
            # 1) Set home pose and gripper position

#Gripper position, max value 1100 - open, min value -0 - close
#change this
            self.red_grasp = 270
            self.yellow_grasp = 350
            self.blue_grasp = 250

            self.home_grasp = 500
            self.open_grasp = 1100
            self.close_grasp = 0


            self.get_logger().warn('Going to home pose...')
            self.call_gripper_service(self.open_grasp)
            time.sleep(delay)
            self.publish_pose(0.436, 0.014, 0.4)
            time.sleep(delay)

            # 2) Goto pick position
            self.get_logger().info('Going to PICK pose...')

            #x_pos =  -(self.end_pose.pose.position.x)
            #y_pos = (self.end_pose.pose.position.y)

            x_pos = 0.43
            y_pos = 0.014
            z_pose = 0.0691

            self.get_logger().info(f'Pick position: x={x_pos}, y={y_pos}, z=0.0691')

            #self.publish_pose(x_pos, y_pos, z_pose)
            #time.sleep(delay)

            self.get_logger().warn('Going to PICK pose...')

            self.pick_color = self.color

            # 3) Close gripper
            if self.pick_color == "red":
                self.get_logger().info('Picking red object...')

                self.publish_pose(0.40, -0.02, 0.28)
                time.sleep(delay)

                self.publish_pose(0.40, -0.02, 0.275)
                time.sleep(delay+3)
                self.call_gripper_service(self.red_grasp)

                #self.publish_pose(0.52, 0.22, 0.28)
                #time.sleep(delay)


            elif self.pick_color == "yellow":
                self.get_logger().info('Picking yellow object...')

                self.publish_pose(0.56, 0.14, 0.27)
                time.sleep(delay)

                self.publish_pose(0.56, 0.14, 0.26)
                time.sleep(delay+3)
                self.call_gripper_service(self.yellow_grasp)

                #self.publish_pose(0.38, 0.1, 0.3)
                #time.sleep(delay)


            elif self.pick_color == "blue":
                self.get_logger().info('Picking blue object...')

                self.publish_pose(0.38, -0.22, 0.275)
                time.sleep(delay)

                self.publish_pose(0.38, -0.22, 0.265)
                time.sleep(delay+3)
                self.call_gripper_service(self.blue_grasp)

                #self.publish_pose(0.45, -0.12, 0.26)
                #time.sleep(delay)


            else:
                self.get_logger().warn('Unknown color, default gripper action...')
                self.call_gripper_service(self.home_grasp)
            #time.sleep(delay-3)

            self.get_logger().warn('Going to Home pose...')

            # 4) Goto home position
            #self.publish_pose(0.436, 0.014, 0.4)
            #time.sleep(delay)

            self.get_logger().warn('Going to Place pose...')

            # 5) Goto placing position
            if self.pick_color == "red":
                self.get_logger().info('Going to RED place pose...')
                self.publish_pose(0.87, -0.2, 0.35)

            elif self.pick_color == "yellow":
                self.get_logger().info('Going to Yellow place pose...')
                self.publish_pose(0.82, 0.0, 0.38)

            elif self.pick_color == "blue":
                self.get_logger().info('Going to Blue place pose...')
                self.publish_pose(0.87, 0.2, 0.4)

            else:
                self.get_logger().info('Going to default place pose...')
                self.publish_pose(0.7, 0.0, 0.3)
                

               
            #self.publish_pose(0.636, 0.014, 0.2)
            time.sleep(delay+6)

            # 6) Open gripper
            self.call_gripper_service(self.open_grasp)
            time.sleep(delay+3)

            # 7) Goto home position
            #self.publish_pose(0.436, 0.014, 0.3)
            #time.sleep(8)

def main(args=None):
    rclpy.init(args=args)
    pnp_node = PnpNode()
    #tf_node = TFlisten()

    # Use MultiThreadedExecutor to handle callbacks concurrently
    #executor = MultiThreadedExecutor(num_threads=4)
    #executor.add_node(pnp_node)
    #executor.add_node(tf_node)

    try:
        thread = threading.Thread(target=pnp_node.pick_and_place_sequence)
        thread.start()
        #while rclpy.ok():
            #pnp_node.get_object_pose()
            #print('Object pose:', pnp_node.end_pose)
        #    rclpy.spin_once(pnp_node, timeout_sec=0.1)

        rclpy.spin(pnp_node)  # Keeps processing timer callbacks
    except KeyboardInterrupt:
        pass
    
    pnp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
