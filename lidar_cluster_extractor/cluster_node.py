import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from sklearn.cluster import DBSCAN
import math

class LidarClusterExtractor(Node):
    def __init__(self):
        super().__init__('cluster_node')

        # Subscriber for lidar data:
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Publisher: communicates with Rviz2:
        self.publisher_ = self.create_publisher(MarkerArray, '/clusters_markers', 10)

        # eps = distance in meters; min_samples = minimum number of points
        self.dbscan = DBSCAN(eps = 0.2, min_samples = 3)
        self.get_logger().info('Node started, Waiting for data...')

    def scan_callback(self, msg):
        points = []
        for i, range_val in enumerate(msg.ranges):
            # Stores only valid point readings in points
            if math.isinf(range_val) or math.isnan(range_val) or range_val < msg.range_min or range_val > msg.range_max:
                continue
            # Angle = minimum angle + (i * increment)
            angle = msg.angle_min + i * msg.angle_increment

            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)

            points.append([x, y])

        if not points:
            return
        
        points_mat = np.array(points)
        # Application of DBSCAN on the points array, an array of labels will be returned.
        labels = self.dbscan.fit_predict(points_mat)

        # Disclaimer: this part was AI generated since I had difficulties handling markers on rviz2:
        unique_labels = set(labels)
        marker_array = MarkerArray()
        marker_id = 0
        for label in unique_labels:
            # Label -1 means "noise" (isolated points).
            
            if label == -1:
                continue 

            # Filters the points matrix to get only the (X, Y) coordinates of this specific obstacle
            cluster_points = points_mat[labels == label]

            # Finds the "center of mass" of the obstacle by taking the average of X and Y
            centroid_x = np.mean(cluster_points[:, 0])
            centroid_y = np.mean(cluster_points[:, 1])

            #3D cylinder for RViz:
            marker = Marker()
            marker.header.frame_id = msg.header.frame_id # Draws using the Lidar's own reference frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "clusters"
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Positions the cylinder exactly at the centroid we just calculated
            marker.pose.position.x = float(centroid_x)
            marker.pose.position.y = float(centroid_y)
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            # Size of the virtual obstacle in RViz (e.g., 20cm x 20cm base, 50cm height)
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.5

            # Obstacle color (Bright green, to contrast with the red/black map of the Costmap)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8 # Leaves the cylinder 80% opaque

            # Lifetime: if the Lidar makes a full turn and no longer sees this object, the cylinder disappears from the screen
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000 

            marker_array.markers.append(marker)
            marker_id += 1

        
        self.publisher_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterExtractor()
    
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()