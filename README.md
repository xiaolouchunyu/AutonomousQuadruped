# Steps to run the Project
This is AutonomousQuadruped to finish work
Successfully working perception pipeline 
Successfully working path planning
Successfully working trajectory planning
Successfully working planner for coordinated motion 
Successfully crossing the parkour

## 1.Download the project

1. Git clone the code from our [repository](https://gitlab.lrz.de/ge23ged/intro2ros_iron-dog).
2. run `cd ~/ws_dog`
3. run `catkin build`
4. Download Quadruped.zip from this [link](https://syncandshare.lrz.de/getlink/fi3YWNfndrPTpXXkCT9sKvM9/).
5. Unzip the Unity file and copy the files to ws_dog/devel/lib/simulation/

## 2.Download the external ROS package

Because we don't need change the code of these Packags,so we can directly download their binary files.

1. pkg:`depth_image_proc`

   ```
   sudo apt-get install ros-noetic-image-pipeline 
   ```

2. pkg:`Octomap`

   ```
   sudo apt-get install ros-noetic-octomap
   sudo apt-get install ros-noetic-octomap-server
   sudo apt-get install ros-noetic-octomap-plugins
   ```

3. pkg:`move_base`

   ```
   sudo apt-get install ros-noetic-move-base
   ```

4. pkg:`map_server`

   ```
   sudo apt-get install ros-noetic-map-server
   ```

## 3.Run the project

Please run the following commands in the shell:

1. Add environment: `vim ~/.bashrc`

   Then add `source /home/$(your pc name)/intro2ros_iron-dog/ws_dog/devel/setup.bash`to the file bottom

2. Run the Project: `roslaunch simulation run.launch`

   - We have two controller, the default controller file is "controller_node_test.cpp"

   - If want to use the another one, please change the line8 in run.launch from `<node pkg="controller_pkg" type="controller_node_test" name="controller_node" />`

     to

     `<node pkg="controller_pkg" type="controller_node" name="controller_node" />`

3. Use the button `2D Nav Goal` in the Rviz to choose an location, where we want dog go.
