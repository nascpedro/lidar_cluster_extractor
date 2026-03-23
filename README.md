# Lidar Cluster Extractor

An advanced perception node in ROS2 developed for @Home category robots. This package acts as a spatial translator, receiving raw data from a 2D Lidar and using Machine Learning to group points into isolated semantic obstacles, publishing their centroids in real-time and drawing them as cilinder markers on Rviz2.

## Architecture and Operation

While the `nav2_costmap_2d` treats the environment in a binary way (free/occupied) to avoid collisions, this node extracts **geometric semantics**. It allows the robot to stop seeing just "a wall of points" and possibly start identifying "Objects" (e.g., trash cans, chair legs, moving humans). Further testing required for this purpose.

### The Algorithm (DBSCAN):
We use **DBSCAN** (Density-Based Spatial Clustering of Applications with Noise) from the `scikit-learn` library. 
1. **Translation:** Polar coordinates (`ranges` and `angle_increment`) from the `/scan` topic are converted into a Cartesian matrix (X, Y).
2. **Clustering:** The algorithm groups spatially close points (`eps=0.2m`) and requires a minimum number of points (`min_samples=3`) to form a valid group, automatically filtering sensor noise (label `-1`).
3. **Center of Mass:** The geometric centroid of each valid cluster is calculated using `numpy`.
4. **Visualization:** 3D cylinders are generated at the centroid coordinates and sent to RViz2.

## Dependencies:

This package was built using Python (`ament_python`) and depends on the following libraries:
* `rclpy`
* `sensor_msgs`
* `visualization_msgs`
* `numpy`
* `scikit-learn` (Installed via rosdep as `python3-sklearn`)

## How to Install and Build:

At the root of your ROS2 workspace, install the OS dependencies using `rosdep`:

```bash
rosdep install --from-paths src -y --ignore-src
```

Then, build specifically this package:

```bash
colcon build --packages-select lidar_cluster_extractor
```

## How to Run

Do not forget to source your workspace before running:

```bash
source install/setup.bash
```

**To run on the physical robot:**

```bash
ros2 run lidar_cluster_extractor cluster_node
```

**To run in simulation (Gazebo):**
Synchronize the node's clock with the simulator's clock so that the markers appear correctly in the RViz TF Tree:

```bash
ros2 run lidar_cluster_extractor cluster_node --ros-args -p use_sim_time:=true
```

## ROS2 Topics

| Direction | Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Sub** | `/scan` | `sensor_msgs/msg/LaserScan` | Raw Lidar reading data (Configured with flexible QoS). |
| **Pub** | `/clusters_markers` | `visualization_msgs/msg/MarkerArray` | Array of geometric cylinders representing the center of the obstacles. |


