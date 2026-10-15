import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import os
import re
from tf2_ros import Buffer, TransformListener
import glob

class MeshMarkerPublisher(Node):
    def __init__(self, mesh_root="demo_data"):
        super().__init__('mesh_marker_auto_sync_tf')
        self.mesh_root = mesh_root
        self.mesh_name = None
        self.mesh_path = None
        self.mesh_extensions = ['.obj', '.stl', '.STL']

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(1.0, self.check_tf_frames_and_publish)

    def check_tf_frames_and_publish(self):
        try:
            all_frames = self.tf_buffer.all_frames_as_yaml()
        except Exception as e:
            self.get_logger().warn(f"TF read error: {e}")
            return

        matches = re.findall(r"(\w+)_frame_base", all_frames)
        if not matches:
            return

        newest = matches[-1]

        # Always delete the currently active marker (if any)
        if self.mesh_name:
            self.delete_previous_marker()

        # Try to load mesh for the new object
        if newest != self.mesh_name:
            self.get_logger().info(f"Looking for mesh file for object: {newest}")
            mesh_path = self.find_mesh_file(newest)
            if mesh_path:
                self.mesh_name = newest
                self.mesh_path = mesh_path
                self.get_logger().info(f"Loaded mesh: {self.mesh_path}")
            else:
                self.get_logger().warn(f"Mesh not found for: {newest}")
                self.mesh_name = None
                self.mesh_path = None
                return
        else:
            self.get_logger().info(f"Reselected existing object: {newest}")

        self.publish_marker()




    def find_mesh_file(self, name):
        for ext in self.mesh_extensions:
            pattern = os.path.join(self.mesh_root, '**', f'{name}{ext}')
            matches = glob.glob(pattern, recursive=True)
            if matches:
                return matches[0]
        return None

    def delete_previous_marker(self):
        if not self.mesh_name:
            return

        marker = Marker()
        marker.header.frame_id = f"{self.mesh_name}_frame_base"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "mesh"
        marker.id = 0
        marker.action = Marker.DELETE

        self.marker_pub.publish(marker)
        self.get_logger().info(f"Deleted marker for: {self.mesh_name}")


    def publish_marker(self):
        if not self.mesh_name or not self.mesh_path:
            return

        marker = Marker()
        marker.header.frame_id = f"{self.mesh_name}_frame_base"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "mesh"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.mesh_resource = f"file://{os.path.abspath(self.mesh_path)}"
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MeshMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
