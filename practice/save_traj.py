import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import csv
from geometry_msgs.msg import TransformStamped

class TransformSaver(Node):
    def __init__(self):
        super().__init__('transform_saver')
        self.get_logger().info('TransformSaver node has started.')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.csv_file = open('/home/meow/ur_traj/transform_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'translation_x', 'translation_y', 'translation_z', 'rotation_x', 'rotation_y', 'rotation_z', 'rotation_w'])

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'tool0', now)
            self.save_transform(trans)
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

    def save_transform(self, trans: TransformStamped):
        timestamp = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9
        translation = trans.transform.translation
        rotation = trans.transform.rotation
        self.get_logger().info(f'Saving transform: timestamp={timestamp}, translation=({translation.x}, {translation.y}, {translation.z}), rotation=({rotation.x}, {rotation.y}, {rotation.z}, {rotation.w})')
        self.csv_writer.writerow([timestamp, translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TransformSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
