# marineMechatronicsMir
code for marine mecatronics class Master Mir

## Visual servoing code

The visual servoing class will be based on two steps :
* visual blob tracking to ge 2D points position 
* visual servoing

### Install 
go to you catkin workspace :
  ```
  cd ~/catkin_ws
  cd src
  ```
 
clone the repo : 
  ```
  git clone https://github.com/cosmer-admin/autonomous_rov.git
  ```

source your catkin workspace : 
  ```
  source devel/setup.bash
  ```
  
test it on a bag
1. open a new terminal and run the bag testtracking.bag with loop option so that it never stops publishing
  ```
  cd ~/catkin_w/src/autonomous_rov/bags
  rosbag play -l  testtracking.bag
  ```
  
2. run the launch file
   ```
   roslaunch autonomous_rov run_visual_servoing.launch
   ```
 
3. points tracking monitoring

The image displays 
  - the current tracked points in green 
  - the desired point in red that will be used in visual servoing

The order of the points is set at the begining of the tracking algorithm depending on the first point detected by the algorithm.

Tracked point and desired point might be ordered identically.
That is why when you reset the tracking, the desired points are also tracked. 

The tracking is monitored by clicking in the image.
  - Right click on the image to update the desired points
  - Left click on the image to initialise the tracked points and reset the desired point

![Tracking ok  : current tracked points are in green, desired points are in red](images/trackingok.png)

If there is a too big shift between to successive detection, the algorithm launch a warning and the publisher stops until you click on the image to reset the desired and tracked point.


![Tracking failed : you have to LEFT click to reinit the tracking](images/trackingko.png)



4. look at the topic : you should see two new topics : 
  - /br5/tracked_points de type std_msgs/Float64MultiArray
  - /br5/desired_points de type std_msgs/Float64MultiArray
They are published by the node blob_tracker_mir.py. They contained the point position in the image in pixels : \[u1,v1,u2,v2,u3,v3,....,uN, vN\]
They are listened in the node visual_servoing_mir.py.
 
 
 
### Real robot application
You have to change the group name in the launch file to adapt it to your robot topic
1. Open the file run_visual_servoing_launch
```
  gedit ~/catkin_ws/src/autonomous_rov/launch/run_visual_servoing.launch
 ``` 
2. Change Line 11 with your rov number. Replace br5 with br1 , br2 , br3 or br4 ...
  ```
  line 11 <group ns="br5">
```
3. run the launch file
   ```
   roslaunch autonomous_rov run_visual_servoing.launch
   ```
4. see following steps of the install part
