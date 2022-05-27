# marineMechatronicsMir
code for marine mecatronics class Master Mir

## Visual servoing class

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
 
3. control the tracking 
  - right click on the image to update the desired points
  - left click on the image to initialise the tracked points and reset the desired point
 
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
4. control the tracking 
  - right click on the image to update the desired points
  - left click on the image to initialise the tracked points and reset the desired point
