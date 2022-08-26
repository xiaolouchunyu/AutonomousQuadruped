<div align='center' ><font size='70'>Quadruped</font></div>

# 0.Catalog

[toc]

# 1.Team members



| Name        
| ----------- 
| Yang Xu     
| Xinwen Liao 
| Zihao Wang  
| Guanran Pei 





# 2.Overview of Ros Packages and Graph

## 2.1 Ros Packages

| Node             | Function                               | Contributor                                  |
| ---------------- | -------------------------------------- | -------------------------------------------- |
| Depth_image_proc | transfer depth_image to point cloud    | Yang Xu                                      |
| Octomap          | transfer point cloud to occupancy grid | Yang Xu                                      |
| Move_base        | navigate and generate a path           | Xinwen Liao, Yang Xu                         |
| Controller       | control the dog to follow up the path  | Yang Xu, Xinwen Liao, GuanranPei, Zihao Wang |

## 2.2 Ros Graph

![](https://gitlab.lrz.de/ge23ged/intro2ros_iron-dog/-/raw/master/Results/rosgraph.png)

# 3.Algorithms

## 3.1 Perception

The launch file for realizing Perception is located in `/src/simulation/launch/occupancy_map.launch`.

Two nodes will be run in this launch file: 1.point_cloud_xyz and 2.octomap_server

1. point_cloud_xyz:

   - This node will generate point cloud from depth_image.
   - Here we use the nodelet to run the node.

   - It will subscribe depth image from the topic "/realsense/depth/image" and "/realsense/depth/camera_info"
   - It will publish point cloud through the topic "/point_cloud"

2. octomap_server:

   - This node will generate occupancy grid from the point cloud

   - It will subscribe the point cloud from the topic "/point_cloud",which is published by the node  point_cloud_xyz.

   - It will publish topic "/octomap_binary","/octomap_full" and "map/projected_map". With the help of these topics we can show the occupancy grid in the rviz.

   - To get rid of the useless occupancy grid of the floor, we try to change the params "pointcloud_min_z" and "filter_ground". 

     Finally we find that,when "pointcloud_min_z" is set to 0.1, the floor's occupancy grid will disappear.

After the occupancy grid successfully created, the node map_saver is used to save the map in the `/src/simulation/map/`



## 3.2 Path_planning

The launch file for realizing Perception is located in `/src/simulation/launch/path.launch`.

To run the node move_base,we need to set the the following 4 .yaml files:

1. costmap_common_params.yaml

   We set the robot size, safe distance from obstacles, sensor information in this file.

   By the way, we don't have the .urdf file of the dog. So we just give imagined values for the robot size.

2. global_costmap_params.yaml

   We set the frequency of global costmap updating/publish in this file.

3. local_costmap_params.yaml

   We set the frequency of local costmap updating/publish in this file. 

   We also define the local costmap's size here.

4. base_local_planner_params.yaml

   We set the max and min velocity of the robot in this file.



## 3.3 Controller

### 3.3.1 Time Control

The code for realizing Controller is located in `/src/controller_pkg/src/controller_node_test.cpp`

The controller is used to help us be familiar with the map.

We set different time point to change the direction, so that the robot can walk through the road. 

### 3.3.2 Signal Control

The code for realizing Controller is located in `/src/controller_pkg/src/controller_node.cpp`

This is our main controller. We want to let the robot can follow up the path generated from the Path_planning. BUt it still can't work well.

To realize what we want, four topics are subscribed:

1. `move_base/TrajectoryPlannerROS/global_plan`

   This topic gives us the coordinates of the path's points. So we calculate the difference between the current state and the next path point. Depending on this difference we can know the robot should go forward or backward and turn left or right.

   But it has a problem: when it turns 90 degrees, the standard to judge the direction should change. For example, at first, the difference in the axis y is used to judge go forward or backward and the difference in the axis x is used to judge turn left or right. But after turning 90 degrees, we should use axis Y to judge turn left or right and axis x to judge go forward or backward.

   To solve the problem, we use the same idea as Time Control. We add a ROS::Time to calculate how long has the robot moved. Depending on the ROS::Time the   usage of axis x and y can be definite

2. `current_state_est`

3. `cmd_vel`

   As other tutorials said, the controller should be depending on the topic cmd_vel. This topic can tell us the velocity at which the robot should arrive at this point.

   But when we use the command `rostopic echo /cmd_vel`, we found that the values of x and y will become 0 soon. So we abandon using this topic to control our robot.

4. `current_goal`

   At first we want to use the goal's coordinate to determine the direction of the robot. But it seems to useless.





# 4.Problems and Future

## 4.1 Problems

1. Can't climb the steps.
1. The controller can't follow up the trajectory well.
1. Because of unknown part in the map, we need to click the 2D Nav Goal 2 times.



## 4.2 Future

1. If possible, can we get the urdf of the robot? This way all joints of the robot can be controlled.

   In my opinion(Yang Xu), it is impossible using the three variables in the controller_node.cpp to climb the steps.

   

2. We should read more control books. The hardest part in this project is controller. By now we had no enough knowledge to deal the problem well.

   

3. We should read the orb_slam's or other vslam's code. The robot has rgb camera, but we don't know how to use it.

   In my opinion(Yang Xu), it is possible using rgb camera to get more information, which can help us to control the robot

   

# 5.Results


## 5.1 Path planning
Because of unknown part in the map, we need to click the 2D Nav Goal 2 times.
1. First:
   ![](https://gitlab.lrz.de/ge23ged/intro2ros_iron-dog/-/raw/master/Results/Result2.png)
2. Second:
   ![](https://gitlab.lrz.de/ge23ged/intro2ros_iron-dog/-/raw/master/Results/Result3.png)
## 5.2 Movement
![](https://gitlab.lrz.de/ge23ged/intro2ros_iron-dog/-/raw/master/Results/Result.png)

# 6.Bibliography

- All Ros pkgs' pages: http://wiki.ros.org

- Two series Tutorials:

  - https://husarion.com/tutorials/ros-tutorials/7-path-planning/
  - http://www.autolabor.com.cn/book/ROSTutorials/
  
- How to use depth_image_proc:https://gist.github.com/bhaskara/2400165

- One oben source that using Orb_slam2 to generate occupancy grid:https://github.com/Ewenwan/ORB_SLAM2_SSD_Semantic

  

