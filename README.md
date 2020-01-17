<!-- 
What doesn't kill you makes you stronger
Ronghou Ha (Timothy)
 -->
Package Description
---

### Genius Pros - UGV Project
### Update Logs

##### v1.5 - Adapted to New Base Structure with New Framework <br>
* Project are running on new base which faster and heavier.
* Adding base communication node that directly control motor RPM via RS232.
* Changing local obstacle avoidance algorithm to path-based frame clean path search. Similar to Automation vehicle avoidance method without lane detection requirement.
* Update map server and map editor node (map editor is a separate project now).
* Note: Current version is ONLY a temporary version for testing new base.

##### v1.0 - Brand New Structure with New Navigation System <br>
* The new system are no longer require move base or any other ROS Navigation Stack packages except Map server. 
* New planner based on Node-Graph Map search by using a modified version of Astar Algorithm.
* Adding new map editor and map shift package. 
* Map editor search lines in 2D occupancy grid to make a Node Graph contain node ID, Node position and there neighbors.
* Map shift read a junction file to load all junction point where map would be change according to Goal and Vehicle location.
* Temperately using v0.3's controller method which are Pure-Pursuit combine VFF for obstacle avoidance and trajectory generate.


##### v0.3 - Pure-Pursuit Plus VFF Version <br>
* Using Pure-Pursuit Method to follow a static path, which planed base on navigation map's costmap.
* Recording global path only once then set it to static until finish or receive new goal.
* User could us RViz "Publish Point" button to publish a simulation obstacle when running test mode.
* Using Obstacle Avoidance Algorithm base on VFF (Virtual Force Field).
* Currently version are no longer using costmap and TEB as local planner due to slow process speed.

##### v0.2 - Way-point Visualization Version <br>
* Analyze global path extract way point by chosen lookahead distances.
* Display way-point estimate pass-through speed on each way-point based on curvature.
* Adding "super control" mode to XBOX in case facing narrow path algorithm would allow robot to pass.
* Android App is able to sending preset goal to robot. 

##### v0.1 - Initial Version <br>
* Include base control node allows both planner, XBOX and Android.
* Simple log system, record and print log in 4 level (Initialization, State, Warning, Error) with changeable color and period.
* Basic obstacle avoidance system, based on Virtual Force Field (VFF) combine with path prediction to optimize 
efficiency.
* Navigation planner is totally base on ROS Navigation Stack, only slightly change for report planner state at each stage.
---

### QuickStart Guide
1. Source "setup.bash" at the devel folder of your workspace.
2. Launch sensor.launch. And check if all data have been received in right frequency.
3. Lanucn base.launch. The log would be print in terminal or saved as a .log file.
4. Using RViz or Xbox controller to send command or task. 

Note: Data received in low frequency would caused localization failure. And there might have hardware issues caused vehicle unable to drive. There are also other reasons might affect localization accuracy and lead to navigation failure. 

---
### Installation
>TODO

---

### Table of Contents
  * [Parameter](#h1)
  * [Base](#h2)
  * [Navigation](#h3)
  * [Sensor](#h4)

---

### Parameter <a name="h1"></a>
Include all parameter,launch files and other loaded files. 

#### Launch
Launch files explanation(`**`necessary for running robot `*`requiring for navigation)

* `*` `base.launch` <br>
	* Map loading and Shifting package, Node Graph Map searching based planner and mission control package all-in-one.

* `**` `sensor.launch` <br>
	* Pairing with the used sensor on board.Checking IP setting before running the launch. robosense device IP should be 192.168.1.200,and also require set computer IP to 192.168.1.102.

#### Map
Each map has a `.png` 2D grid map and a `.yaml` support it.<br>
In this folder also contain `.txt` stored Node Graph which supported by `_info.txt` file.

---

### Base <a name="h2"></a>
Include basic control node working on three different controllers. ( planning, XBOX controller, Android phone ) <br>
Using mission control as planner give rough command, then optimize it with self developed method, based on virtual force field.

---

### Navigation <a name="h3"></a>
This folder include multiple important packages for Routing, Planning and Control.
* Map manager read Node Graph Map and make plan from vehicle location to target.
* Map server is modified based on Navigation Stack. Now it allow map shift via ROS topic message. 
* Mission control package generate trajectory and send command based on global path from map manager. And provide a collision free path to avoiding obstacle in real-time using sensor data and obstacle information. 


---

### Sensor <a name="h4"></a>
Include Perception Package and sensor Drive for this project. 
* Obstacle manager subscribe obstacle information from Power-X. Process then send to mission control package for navigation purpose. 
* One for converting pointcloud message to laser scan message type from raw lidar data. 
* `ros_rslidar` is official package provided by robosense, include driver, ROS interface and calibration file.

---



