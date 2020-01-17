<!-- 
What doesn't kill you makes you stronger
Ronghou Ha (Timothy)
 -->
Package Description
---

### Genius Pros - UGV Project

Welcome to Genius Pros - UGV Project GitHub page



### Update Logs

##### v2.0 - Cubic Spline Curve Version of New Navigation System <br>
* Refactoring navigation stack, lightweight plus new cubic spline curve algorithm.
* Adding configure folder for user to changing configuration without complier workspace again.
* Debug for previous version.
* Modify version numbers and edited instruction documents. 

##### v1.5 - Adapted to New Base Structure <br>
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
2. Launch bringup_navi.launch.
3. Using RViz or Xbox controller to send command or task. 

Note: Current version of project only contains HD map,prediction,planning and vehicle RS232 communication parts. Localization or Customize communication protocol are not provide at this point. 

---
### Installation
##### System Requirement: Ubuntu 16.04
##### ROS Environment   : ROS Kinetic
##### Other Libraries   : OpenCV 2/3 + PCL 1.7 + Boost 1.7

1. Change directory to folder of your workspace.
2. catkin_make

---

### Documents
>TODO

##### Configure <a name="h1"></a>

##### Base <a name="h2"></a>

##### Navigation <a name="h3"></a>

##### Perception <a name="h4"></a>

---



