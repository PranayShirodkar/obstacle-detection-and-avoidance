# Obstacle Detection and Avoidance for an Unmanned Surface Vehicle

![](USV_Sharky.jpg)

The objective of this project is to equip the above unmanned surface vehicle (USV) with the ability to operate autonomously. This project contains 2 sections:
  1. obstacle detection using stereo vision and point cloud processing
  1. path planning using Voronoi-Dijkstra algorithm

This project utilizes the following technologies: Ubuntu, ROS, OpenCV, Point Cloud Library
This project utilizes the following algorithms: stereo vision block matching, RANSAC plane segmentation, A* , Dijkstra.

## Part 1: Obstacle Detection

Refer to the [Final Report](./reports/Obstacle_Detection_for_an_Unmanned_Surface_Vehicle.pdf) for full details.

Before using the cameras, we must perform camera calibration:

<img src="./reports/camera_calibration.png" alt="drawing" width="400"/>

Using the following obstacle environment:

<img src="./reports/left_camera.png" alt="drawing" width="400"/>

To determine the location of the obstacles relative to the USV, we use 2 calibrated cameras mounted on the USV and stereo vision block matching algorithm. This is a technique in computer vision to determine how far away image features are from the cameras. This algorithm generates a disparity map:

<img src="./reports/disparity_map.png" alt="drawing" width="400"/>

This disparity map can be converted into a 3D point cloud:

<img src="./reports/3D_point_cloud.png" alt="drawing" width="400"/>

RANSAC plane segmentation algorithm is used to identify the largest plane of point clouds. If done correctly, it should identify the floor or ground. This plane is then removed from the point cloud, leaving only the obstacles:

<img src="./reports/3D_obstacle_map.png" alt="drawing" width="400"/>

Finally, take the top down view of the remaining point cloud to obtain a 2D obstacle map for path planning purposes:

<img src="./reports/2D_obstacle_map.png" alt="drawing" width="400"/>


## Part 2: Path Planning

Refer to the [Final Report](./reports/Obstacle_Avoidance_for_an_Unmanned_Surface_Vehicle.pdf) for full details.

Using the following obstacle environment:

<img src="./reports/simulated_obstacle_map.png" alt="drawing" width="300"/>

If A* algorithm is used for shortest path planning:

<img src="./reports/A_star_path.png" alt="drawing" width="300"/>

A* algorithm does not consider the proximity of the path to obstacles. In a maritime environment the USV should maintain a safe distance from obstacles. For this we use Voronoi diagrams to find safe paths (safe paths in blue, unsafe paths in grey), and Dijkstra path planning to find the shortest path available:

<img src="./reports/VD_path.png" alt="drawing" width="300"/>
