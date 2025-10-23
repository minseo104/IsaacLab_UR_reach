#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
from geometry_msgs.msg import Point
#import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('object_detection')

        self.get_logger().info('Object detection node started')
        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()
        self.centroid = None  # Set this to the centroid value as needed

        # Subscribers and Publishers
        self.contour_image_pub = self.create_publisher(Image, '/contour_image', 10)
        self.threshold_image_pub = self.create_publisher(Image, '/threshold_image', 10)
        self.contour_image_pub = self.create_publisher(Image, '/contour_image', 10)
        self.color_pub = self.create_publisher(String, '/object_color', 10)

        self.pose_pub = self.create_publisher(Point, '/object_3d_pose', 10)

        # Timer for publishing static TF
        self.create_timer(1.0, self.publish_static_tf)


        self.subscription1 = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)

        self.subscription2 = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10)


        self.subscription1  # prevent unused variable warning
        self.subscription2  # prevent unused variable warning

    def get_quaternion_from_euler(self,roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        hsv_img_red = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_img_green = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_img_blue = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_img_yellow = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask1_yellow = cv2.inRange(hsv_img_yellow, (20, 100, 100), (30, 255, 255))
        mask2_yellow = cv2.inRange(hsv_img_yellow, (30, 100, 100), (40, 255, 255))

        mask1_red = cv2.inRange(hsv_img_red, (0, 120, 70), (10, 255, 255))
        mask2_red = cv2.inRange(hsv_img_red, (170, 120, 70), (180, 255, 255))

        mask1_green = cv2.inRange(hsv_img_green, (35, 100, 100), (70, 255, 255))
        mask2_green = cv2.inRange(hsv_img_green, (70, 100, 100), (90, 255, 255))


        mask1_blue = cv2.inRange(hsv_img_blue, (100, 150, 0), (120, 255, 255))
        mask2_blue = cv2.inRange(hsv_img_blue, (120, 150, 0), (140, 255, 255))

        combined_mask_red = mask1_red | mask2_red
        combined_mask_green = mask1_green | mask2_green
        combined_mask_blue = mask1_blue | mask2_blue
        combined_mask_yellow = mask1_yellow | mask2_yellow

        contours_red, _ = cv2.findContours(combined_mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(combined_mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(combined_mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv2.findContours(combined_mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)



        self.detected_color = "none"
        cx = 0
        cy = 0        
        if contours_red or contours_green or contours_blue or contours_yellow:
            largest_contour_red = max(contours_red, key=cv2.contourArea, default=None)
            largest_contour_green = max(contours_green, key=cv2.contourArea, default=None)
            largest_contour_blue = max(contours_blue, key=cv2.contourArea, default=None)
            largest_contour_yellow = max(contours_yellow, key=cv2.contourArea, default=None)


            # Draw contours on the original image
            cv2.drawContours(img, largest_contour_red, -1, (0, 255, 0), 5)
            cv2.drawContours(img, largest_contour_green, -1, (0, 0, 255), 5)
            cv2.drawContours(img, largest_contour_blue, -1, (0, 255, 0), 5)
            cv2.drawContours(img, largest_contour_yellow, -1, (255,0, 0), 5)



            m_red = cv2.moments(largest_contour_red) if largest_contour_red is not None else None
            m_green = cv2.moments(largest_contour_green) if largest_contour_green is not None else None
            m_blue = cv2.moments(largest_contour_blue) if largest_contour_blue is not None else None
            m_yellow = cv2.moments(largest_contour_yellow) if largest_contour_yellow is not None else None

            area_red = m_red['m00'] if m_red is not None else 0
            area_green = m_green['m00'] if m_green is not None else 0
            area_blue = m_blue['m00'] if m_blue is not None else 0
            area_yellow = m_yellow['m00'] if m_yellow is not None else 0

            largest_area = max(area_red, area_green, area_blue, area_yellow)


            print("LARGEST AREA: ", largest_area)

            if(largest_area > 1000):

                try:
                    if largest_area == area_red:
                        self.detected_color = "red"
                        cx, cy = int(m_red['m10'] / m_red['m00']), int(m_red['m01'] / m_red['m00'])
                    elif largest_area == area_green:
                        self.detected_color = "green"
                        cx, cy = int(m_green['m10'] / m_green['m00']), int(m_green['m01'] / m_green['m00'])
                    elif largest_area == area_blue:
                        self.detected_color = "blue"
                        cx, cy = int(m_blue['m10'] / m_blue['m00']), int(m_blue['m01'] / m_blue['m00'])
                    elif largest_area == area_yellow:
                        self.detected_color = "yellow"
                        cx, cy = int(m_yellow['m10'] / m_yellow['m00']), int(m_yellow['m01'] / m_yellow['m00'])
                    else:
                        self.detected_color = "none"
                        cx, cy = 0, 0


                    cv2.circle(img, (cx, cy), 15, (0, 255, 0), -1)
                    cv2.putText(img, self.detected_color, (1280 // 2, 720 // 2), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 3)
                except:
                    #self.centroid = None  # If division by zero occurs, just
                    print("Exception") 
                    cx, cy = 0, 0

                #pass

        self.centroid = (cx, cy)

        contour_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.contour_image_pub.publish(contour_msg)

        if self.detected_color == "red":
            threshold_msg = self.bridge.cv2_to_imgmsg(combined_mask_red, "mono8")
        elif self.detected_color == "green":
            threshold_msg = self.bridge.cv2_to_imgmsg(combined_mask_green, "mono8")
        elif self.detected_color == "blue":
            threshold_msg = self.bridge.cv2_to_imgmsg(combined_mask_blue, "mono8")
        elif self.detected_color == "yellow":
            threshold_msg = self.bridge.cv2_to_imgmsg(combined_mask_yellow, "mono8")
        else:
            threshold_msg = self.bridge.cv2_to_imgmsg(combined_mask_red, "mono8")  # Default to red mask if none detected

        color_msg = String()
        color_msg.data = self.detected_color
        
        self.color_pub.publish(color_msg)
        self.threshold_image_pub.publish(threshold_msg)
        #self.contour_image_pub.publish(contour_msg)




    def pointcloud_callback(self, msg):
        self.get_logger().warn('Point Cloud Callback')

        if self.centroid is not None:
            row = self.centroid[1]
            col = self.centroid[0]

            # Access the data in the point cloud
            point_index = row * msg.row_step + col * msg.point_step
            data = msg.data[point_index:point_index + msg.point_step]

            if len(data) >= msg.point_step:
                x, y, z = struct.unpack_from('fff', data)

                if not (np.isnan(x) or np.isinf(x) or np.isnan(y) or np.isinf(y) or np.isnan(z) or np.isinf(z)):
                    pt = Point()
                    pt.x = x
                    pt.y = y
                    pt.z = z

                    self.get_logger().info(f'3D Point: x={pt.x}, y={pt.y}, z={pt.z}')

                    if(pt.z < 0.8):

                        # Publish the TF
                        self.publish_tf_object(pt.x, pt.y, pt.z)
                else:
                    self.get_logger().warn('Unable to find points in point cloud')
            else:
                self.get_logger().error('Buffer size is less than point step size')
        else:
            self.get_logger().error('Null pointer detected for point data.')

    def publish_tf_object(self, x, y, z):
        # Implement the TF publishing logic here
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = z
        self.pose_pub.publish(point_msg)
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_depth_optical_frame'
        t.child_frame_id = 'object'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        quat = self.get_quaternion_from_euler(np.radians(140), 0, 0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)


    def publish_static_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        t.transform.translation.x = 0.774954
        t.transform.translation.y = -0.4572
        t.transform.translation.z = 0.5334

        #quat = tf_transformations.quaternion_from_euler(0, np.radians(45), 0)
        quat = [0.0, 0.383, 0.0, 0.924]
        quat =self. get_quaternion_from_euler(0, np.radians(50), np.radians(90))

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
