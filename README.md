Package Description
--- 

### Genius Pros - UGV Project
### Update
##### v0.1 - Initial Version <br>
* Include base control node allows both planner, XBOX and Android.
* Simple log system, record and print log in 4 level (Initialization, State, Warning, Error) with changeable color and period
* Basic obstacle avoidance system, based on Virtual Force Field (VFF) combine with path prediction to optimize 
efficiency
* Navigation planner is totally base on ROS Navigation Stack, only slightly change for report planner state at each stage

##### v0.2 - Way-point Visualization Version <br>
* Analyze global path extract way point by chosen lookahead distances.
* Display way-point estimate pass-through speed on each way-point based on curvature.
* Adding "super control" mode to XBOX in case facing narrow path algorithm would allow robot to pass.
* Android App is able to sending preset goal to robot. 



---
### Install Guide
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

* `**` `base.launch` <br>

* `**` `sensor.launch` <br>
	* Pairing with the used sensor on board.Checking IP setting before running the launch. robosense device IP should be 192.168.1.200,and also require set computer IP to 192.168.1.102.

* `*` `navi.launch` <br>
	* Navigation stack loading launch, all setup parameters are under /param/config.

---

### Base <a name="h2"></a>
Include basic control node working on three different controllers ( planning, XBOX controller, Android phone )

* Using navigation stack as planner give rough command, then optimize it with self developed method, based on virtual force field.

---

### Navigation <a name="h3"></a>
Modified based on ROS Navigation stack.

* Remove original recovery behaviors from `move_base`, send out state signal all time for taking over control when needed. 

More detail check following Wiki page http://wiki.ros.org/navigation

---

### Sensor <a name="h4"></a>
Include two parts to this project. 

* One for converting pointcloud message to laser scan message type from raw lidar data. 
* `ros_rslidar` is official package provided by robosense, include driver, ROS interface and calibration file.

---



