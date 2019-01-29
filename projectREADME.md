# Ishmael Rogers
# Robotics Software Engineer
# Infinitely Deep Robotics Group
# 2019


[images1]: ./images/costmaps.png
[images2]: ./images/rviz-robot-initial-posearray-small.png
[images3]: ./images/rviz-nav-goal-small.png 
[images4]: ./images/transform-tolerance-warning.png
[image5]: ./images/map-update-warning.png


The following instructions builds robot packages in ROS, from scratch. This is the methodology that I follow to successful build robots in ROS.


# Gazebo: 

1. Navigate to src directory and create a new empty package 

``` bash
$ cd /home/workspace/catkin_ws/src/
$ catkin_create_pkg hexapod0
```

2. Create folders, ``launch`` and ```worlds`` so that we can further define the structure of the package

``` bash
$ cd hexapod0
$ mkdir launch
$ mkdir worlds
```

Gazebo worlds
--

Each individual Gazebo world is saved in the ```worlds``` folder. The following commands create a simple world with no onjects or models


$ cd worlds
$ nano hexapod0.world

Add the following to hexapod0.world

``` xml
<?xml version="1.0" ?>

<sdf version="1.4">

  <world name="default">

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- World camera -->
    <gui fullscreen='0'>
      <camera name='world_camera'>
        <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>

```


The ```.world ``` file uses the XML file format to describe all the elements that are being defined with respect to the Gazebo environment. The simple world that you are creating above, has the following elements


* ```<sdf>```: The base element which encapsulates the entire file structure and content.
  
* `` <world>``: The world element defines the world description and several properties pertaining to that world. In this example, you are adding a ground plane, a light source, and a camera to your world. Each model or property can have further elements that describe it better. For example, the ```camera``` has a ``pose`` element which defines its position and orientation.
  
* ```<include>```: The include element, along with the ``<uri>`` element, provide a path to a particular model. In Gazebo there are several models that are included by default, and you can include them in creating your environment.


Launch files 
--
Launch files in ROS allow us to execute more than one node simultaneously. This helps avoid a tedious task of defininig and launching several nodes in separate shells or terminals.

``` bash
$ cd ..
$ cd launch
$ nano hexapod0_world.launch

```
Add the following to the launch file:

``` xml

<?xml version="1.0" encoding="UTF-8"?>

<launch>

  <arg name="world" default="empty"/> 
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find udacity_bot)/worlds/udacity.world"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>

</launch>

```

The structure of the file aboce is divided into two parts

* Define arguments using the ``<arg>`` element. Each element will have a ``name`` attribute and a ``default`` value
* include the ``empty_world.launch`` file from the gazebo_ros package. 

[The empty_world](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros/launch/empty_world.launch) file includes a set of important definitions that are inherited by the world that we create. Using the world_name argument and the path to your .world file passed as the value to that argument, you will be able to launch your world in Gazebo.


Launch the Gazebo environment 
--

``` bash
$ cd /home/workspace/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch hexapod0 hexapod0_world.launch
```

Be sure to explore these links for more insight 

[The SDF format for Gazebo from OSRF](http://osrf-distributions.s3.amazonaws.com/sdformat/api/dev.html)
[A general guide to the SDF format](http://sdformat.org/spec?ver=1.6&elem=sdf)

# Robot Modeling 

Robot URDF
---

Now we will create [The universal description file](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/304b7bc0-6fe8-4614-bd09-4a545665adad) from scratch.

1. Create a new folder in the package directory and an empty xacro file for the robot's URDF description.

``` bash
$ cd /home/workspace/catkin_ws/src/hexapod0/
$ mkdir urdf
$ cd urdf
$ nano hexapod0.xacro
```

In this basic model we will create a cubidal base with two caser wheels. We now copy the following code into the ``hexapod0.xacro `` file.

``` xacro
<?xml version='1.0'?>

<robot name="hexapod0" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <link name="robot_footprint"></link>

  <joint name="robot_footprint_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="robot_footprint"/>
    <child link="chassis" />
  </joint>

  <link name='chassis'>
    <pose>0 0 0.1 0 0 0</pose>

    <inertial>
      <mass value="15.0"/>
      <origin xyz="0.0 0 0" rpy=" 0 0 0"/>
      <inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/> 
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </collision>

    <visual name='chassis_visual'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/>
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </visual>


    <collision name='back_caster_collision'>
      <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.0499"/>
      </geometry>
    </collision>

    <visual name='back_caster_visual'>
      <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

    <collision name='front_caster_collision'>
      <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.0499"/>
      </geometry>
    </collision>

    <visual name='front_caster_visual'>
      <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

  </link>

</robot>

```
In the code above we create a single link the ```name``` "chassis" that contains the base and (caster) wheels of the robot. Each link has a specific elements such as ``inertial`` or ``collision`` elements. We'll revisit [previous lessons](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/304b7bc0-6fe8-4614-bd09-4a545665adad) to get a more in depth overview of these elements. 

The chassis in this example is cuboidal. The wheels are spherical. These geometric properties are denoted by the ``<geometry>`` tags. Each link, or link has an origin, or pose that must be defined as well. Every element of that link or joint has its own origin which is defined relative to the link's frame of reference. 

The caster wheels in this example are attached directly to the base. Therefore there is no need for any additional links to define the casters. As such, there are no joints to connect them. The caster have ```friction``` coefficients associated with them, but are set to 0 in this example to allow for free motion while moving. 


Launch the Model
---

In order to launch our newly created robot we will need to create a new launch file that will help load the URDF file 

``` $ cd /home/catkin_ws/src/hexapod0/launch
    $ nano robot_decription.launch
```

The following code needs to be copied into the ```robot_desrcription.launch``` file in order to define a parameter called ``robot description`` which is used to set a single command use the [xacro package](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File) to generate the URDF from the xacro file.

``` bash
<?xml version="1.0"?>
<launch>

  <!-- send urdf to param server -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find udacity_bot)/urdf/udacity_bot.xacro'" />

</launch>
``` 

Inside the ``launch`` folder we need to update ```hexapod0_world.launch``` so that Gazebo can load the UDRF

``` bash

$ nano hexapod0_world.launch 

```

Add the following to the launch file after ```<launch>```

``` bash

<include file = "$(find hexapod0)/launch/robot_description.launch"/>

````

Add the following to the launch file before ``</launch>``

``` bash
<!--spawn a robot in gazebo world-->
<node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" 
output="screen" args="-urdf -param robot_description -model udacity_bot"/>
```

The [gazebo_ros package](http://wiki.ros.org/gazebo_ros) spawns the model from the URDF that ```robot_description``` helps generate. Next, we launch everything and check if the robot loads up properly.

```
$ cd /home/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch hexapod0 hexapod0_world.launch
```
Robot Actuation
---

Now we add wheels to the robot. In this example, we only include two wheels. Each wheel is represent as a link and is connected to the base link, previously referred to as chassis with a joint.

First we create the links for each wheel using the specifications given below and add that to xacro file. For each whell we have a ``` collision```, ```intertial```, and ```visual``` element along with the following properties/definitions 

* ```link name``` - "SIDE_wheel", where the SIDE is either left or right
* ```geometry``` - "cylinder" with radius 0.1 and length 0.05 
* ```origin``` for each element element - [0, 0, 0, 0, 1.5707, 1.5707]
* ```mass``` of each wheel - "5"
* ```inertia``` values are the same for the chassis 

``` 
ixx="0.1" ixy="0" ixz="0"
iyy="0.1" iyz="0"
izz="0.1"
```
Once the links are defined, we need to define the corresponding joints. The following code will create a joint between the left wheel (the child link) and the robot chassis (the parent link)

```
<joint type="continuous" name="left_wheel_hinge">
    <origin xyz="0 0.15 0" rpy="0 0 0"/>
    <child link="left_wheel"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
    <limit effort="10000" velocity="1000"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>
  ```
  
  The ``joint type`` is set to "continuous" and is similar to a revolute joint but has no limits on its rotation. It can rotate continuously about an axis.  The joint will have its own `axis` of rotation i.e some specific joint ``dynamics`` that correspond to the physcial properties of the joint like "friction", and certain ```limits```to enforce the maximum "effort" and "velocity" for that joint. The limits are useful constraints in regards to physical robot and can help create a more robust robot model in simulation. [click here](http://wiki.ros.org/pr2_controller_manager/safety_limits) to check out more resources to help understand these limits better. 
  
  We'll use the above as template to create the joint between the right wheel and the chassis by modifying the ``child link`` and ``position`` elements
  
 Use ``roslaunch`` as describe earlier
  

# Robot Sensors 

Adding a camera
---
For this robot project we will add two sensors: a camera and a laser rangefinder. Let's start off by adding the camera with the following specifications in the ```hexapod0.xacro``` file:

* ``link namne`` - camera
* ``link origin`` - "[0,0,0,0,0,0]
* ``joint name`` - "camera_joint"
* ``joint origin`` - [0.2,0,0,0,0,0]
* ``geometry`` - box with size "0.05"
* ``mass`` - "0.1"
* ``inertia`` - ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"
* joint ``parent link`` - "chassis", and joint ``child link`` - "camera" 

Each link has its on ``visual``, ``collision``, and ``inertial`` elements



``` bash

$ cd /home/workspace/catkin_ws/hexapod0/urdf
$ nano hexapod0.xacro

```

Laser Rangefinder
---

ROS offer support for many different types of [sensors](http://wiki.ros.org/Sensors#A2D_range_finders). The one for this lab is the Hokuyo rangefinder. Adding this sensors to the robot is done in a similar fashion.

* ``link name``    - "hokuyo"
* ``link origin``  - "[0, 0, 0, 0, 0, 0]" 
* ``joint name``   - "hokuyo_joint"
* ``joint origin`` - "[.15, 0, .1, 0, 0, 0]"
* ``geometry``     - box with size "0.1" for <collision>, and a [mesh file](https://github.com/udacity/RoboND-Localization-Project/tree/master/meshes) file for <visual>
*  ``mass``         - "0.1"
* ``inertia``      - ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"


 Also define the joint type, and the parent and child links!


Mesh files deine the shape of the object or model that we are working with. There are basic shapes to work, but for more advanced designs we will utilize mesh files.

Add the mesh file for the hokuyo sensor using the following tag -

``
<mesh filename= "package://hexapod0/meshes/hokuyo.dae"/>

``
The file above should be located in a folder called ``meshes`` that we can create in your package folder ``hexapod0`` the [mesh file](https://github.com/udacity/RoboND-Localization-Project/tree/master/meshes) is located here.

Gazebo Plugins
---

Now that we have added the sensors to robot, we need to define how the sensors actually capture data. Additionally, we define how the robot moves in a simulated environment. Gazebo, provides tools that allow us to create or use plugins that help utilize all available gazebo functionality in order to implmwent specific use-cases for specific models

The xacro file, ``udacity_bot.gazebo`` can be found in the [github repo](https://github.com/udacity/RoboND-Localization-Project/tree/master/urdf). It needs to be renamed to hexapod0.gazebo and placed in the ``udrf`` folder of the package.

The robot in this project has two wheel. Therefore we will use, a Differential Drive Controller. Below is an overview of how this plugin is defined in the ``.gazebo``

`` xml
<gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <legacyMode>false</legacyMode>
      <alwaysOn>true</alwaysOn>
      <updateRate>10</updateRate>
      <leftJoint>left_wheel_hinge</leftJoint>
      <rightJoint>right_wheel_hinge</rightJoint>
      <wheelSeparation>0.4</wheelSeparation>
      <wheelDiameter>0.2</wheelDiameter>
      <torque>10</torque>
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>robot_footprint</robotBaseFrame>
    </plugin>
  </gazebo>
  
``

``libgazebo_ros_diff_drive.so`` is the shared object file created from compiling some C++ code. THe plugin takes in infomration specific tp the tpbot's model such as wheel sepatation, joint names and more, then calculates and publishes the robot's odometery information to the topics that we are specifying above (the odom topic). The controller allows us to send velocity commands to the robot so that it can move in a specific direction. Refer to this link for the [source code](https://bitbucket.org/osrf/gazebo/src/afe08834571835008fa7419f1feba5b7f89b9d62/plugins/DiffDrivePlugin.cc?at=gazebo7&fileviewer=file-view-default) 


Gazebo has many plugins availble. Therefore we use preexisting plugins for the [camera sensor](http://gazebosim.org/tutorials?tut=ros_gzplugins#Camera) and hokuyo sensor. 

We need to define topics for each sensor and where they publish information or data. For the camera, ``image_raw`` topic 

```
<imageTopicName>image_raw</imageTopicName>

```

For the laser, its ```udacity_bot/laser/scan</topicName> 

```
<topicName>/udacity_bot/laser/scan</topicName>
```

Test the update model in Gazebo after the plugins are imported by the UDRF.

```
$ cd /home/workspace/catkin_ws/src/udacity_bot/urdf
$ nano udacity_bot.xacro
```

Add the following to the top of the file right before the definition of the ```robot_footprint``` link 

``` xacro
<xacro:include filename="$(find udacity_bot)/urdf/udacity_bot.gazebo" />
```

Run the launch file to test the model

```
$ roslaunch hexapod0 hexapod.launch
```

# RViz Integration

Gazebo is a very useful physic simulator that gives us access to models and objects that can be likely found in the real world. [RViz](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/809ef993-dbbb-49d4-8520-0bae34921a60) is a useful tool that can visualize any type of sensor data being published over a ROS topic:

1. Camera images
2. Point clouds 
3. Lidar Data

The data can be a live stream that is measured directly from the sensor or some pre-recorded data stored as a bag file. In general, we will use RViz as a one-stop tool for visualizing all three core aspects of robotics: 

1. Perception
2. Decision making
3. Actuation 

Let's start by intergrating the model into RViz and visualize data from the camera and laser sensors

Modify the robot description file
---

```
$ cd /home/catkin_ws/src/hexapod0/launch/
$ nano robot_description.launch
```

Now we add two `nodes`  to launch file after the first "param" definition: 

```
<!-- Send fake joint values-->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="false"/>
  </node>

<!-- Send robot states to tf -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>
```
One of these nodes uses `joint_state_publisher` that publishes joint state messages for the robot such as the angles for the non-fixed joints. The other node uses the `robot_state_publisher` package that publishes the robot's state to `tf` (transformation tree). The robot model has several frames corresponding to each link/joint. The `robot_state_publisher` publishes the 3D poses of these links. 


Modify the hexapod0_world launch file
---

We'll launch Rviz along with Gazebo

``
$ nano hexapod0_world.launch 
``

Next, we add a node at the end of the file after the `urdf_spawner` node definition. This node launches the `rviz` package.

``
<!--launch rviz-->
<node name="rviz" pkg="rviz" type="rviz" respawn="false"/>

``

Launching it
---

The following command will launch both Gazebo and RViz.  


```
$ cd /home/catkin_ws/
$ roslaunch hexapod0 hexapod0_world.launch
```

We'll now navigate to the RViz Window and on the left side, under `Displays`:

Select “odom” for fixed frame
Click the “Add” button and
add “RobotModel”
add “Camera” and select the Image topic that was defined in the camera gazebo plugin
add “LaserScan” and select the topic that was defined in the hokuyo gazebo plugin. 

The robot model should appear in RViz. 

Now we navigate to to Gazebo and click on "insert" and add any item in the world in front of the robot. The item should be visible in the RViz in the "Camera" viewer and the Laser scan of that object.

Lets publish messages to cmd_vel, a topic which was defined in the drive controller plugin. We'll set the values for linear.x and angular.z to enable the robot to start moving in a circle. 


```
rostopic pub /cmd_vel geometry_msgs/Twist  "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1" 
  ```
 NOTE: For the above command if we enter `geometry_msg/Twist`, pressing Tab twice will complete the message definition so that the required values can be changed. 
 
 Challenge: Incorporate the [teleop package](https://classroom.udacity.com/nanodegrees/nd209/parts/dad7b7cc-9cce-4be4-876e-30935216c8fa/modules/f5048868-4bd8-4e8d-8c6b-69bd559ed9db/lessons/72cae9de-deae-4579-93da-7e56bd5021fc/concepts/3ad78b60-f8b6-494c-afd5-f03de3dcfa3c) to drive the the robot around.
 
 
 # Localization map 
 
 Adding a map
 ---
 
 We will now launch the robot in a new environment created by Clearpath Robotics. Let's first create a new folder in the package directory 
 
 ``
$ cd /home/catkin_ws/src/hexapod0/
$ mkdir maps
$ cd maps

``
We'll `jackal_race.pgm` and `jackal_race.yaml` from the project [repo](https://github.com/udacity/RoboND-Localization-Project/tree/master/maps) into the maps folder.

NOTE: do not open the `.pgm` file because it is large and may make the system freeze

We now revist the empty Gazebo world, called `hexapod0.world`. The map that we will be working with is generated based on its own world.

``
$ cd ..
$ cd worlds

``

Let's modify the ``hexapod0_world.launch`` file and update the path to this new map/world

`` 
$ cd .. 
$ cd launch/
$ nano hexapod0_world.launch
``

Modify the ``world_name`` argument so that it points to `jackal_race.world`. Now we launch the robot in the new map.

``
$ cd /home/catkin_ws/
$ roslaunch hexapod0 hexapod0_world.launch 
``

Note: As of this step, you won’t see the map in RViz.

Localization: Adaptive Monte Carlo Localization Package 

This modification to the [Monte Carlo Localization](https://github.com/IshmaelRogers/Localization/tree/master/MonteCarlo) dynamically adjusts the number of particles over a period of time as the robot navigates around in a map. This adaptive method offers a significant computational advantage over MCL. 

The ROS [amclpackage](http://wiki.ros.org/amcl) implements this variant. We will integrate this package with our robot to localize it inside the provided map.

Create a new launch file

``
$ cd /home/workspace/catkin_ws/src/hexapod0/launch/
$ nano amcl.launch
``

The launch file has three nodes, one of which is for the amcl package. Copy the following into the launch file.


```

<?xml version="1.0"?>
<launch>

  <!-- Map server -->
  <arg name="map_file" default="$(find udacity_bot)/maps/jackal_race.yaml"/>
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

  <!-- Localization-->
  <node pkg="amcl" type="amcl" name="amcl" output="screen">
    <remap from="scan" to="udacity_bot/laser/scan"/>
    <param name="odom_frame_id" value="odom"/>
    <param name="odom_model_type" value="diff-corrected"/>
    <param name="base_frame_id" value="robot_footprint"/>
    <param name="global_frame_id" value="map"/>
  </node>
  
  ```
  
  First we need to load the provided map using a new node for the map_server package. Previousl, the ``robot_state_publisher`` helped build out the entire tf tree of the robot based on the URDF file. However, it did not extend that tree by linking in the 'map' frame. The amcl package does this automatically by linking the 'map' and 'odom' frames. 
  
  Next, we add a node that will launch the amcl package. The package has its own set of parameters that define its behavior in RViz and how everything relates to the robot and the provided map so that the robot can effectively localize itself. The amcl package reles entirely on the robot's odometry and laser scan data. 
  1. `remap` the `scan` topic to the `hexapod0/laser/scan` topic on which the [hokuyo sensor](https://www.hokuyo-aut.jp/search/index.php?cate01=1) publishes the sensors data. We defined this topic previously when we added the gazebo plugin for the sensor. Then we add parameters and define values for the different reference frames such as odom or the map frames. The amcl package will now be able to take in laser and odom data and loclaize the robot.

Navigation Stack 
---

To implement the [Navigation Stack](http://wiki.ros.org/navigation) we will work with the [move_base](http://wiki.ros.org/move_base) package that we can use to define a goal osition for the robot in the map and allow the robot to navigate to that goal position. 

The move_base package is a powerful tool that utilizes a costmap where each part of the map is divided into which area is occupied, like walls or obstacles and which area is unoccupied. As the robot moves around, a local costmap, in relation to the global costmap, keeps getting updated allowing the package to define a contunout path for the robot to move along. 

The package has some built-in corrective behaviors or maneuvers. Based on specific conditions like detecting a particular obstacle or if the robot is stuck, it will navigate the robot around the obstacle or rotate the robot until it finds a clear path ahead. 


Below are images of the local and global costmaps. The gloabl costmap on the left, displays all the occupied areas based on the provided map. The local costmap on the right is only displaying what the laser sensor captures during that time period. 

![alt text][image1]
  
Copy the following code into the above launch file


```
<!-- Move base -->
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <rosparam file="$(find udacity_bot)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find udacity_bot)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find udacity_bot)/config/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find udacity_bot)/config/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find udacity_bot)/config/base_local_planner_params.yaml" command="load" />

    <remap from="cmd_vel" to="cmd_vel"/>
    <remap from="odom" to="odom"/>
    <remap from="scan" to="udacity_bot/laser/scan"/>

    <param name="base_global_planner" type="string" value="navfn/NavfnROS" />
    <param name="base_local_planner" value="base_local_planner/TrajectoryPlannerROS"/>

  </node>


</launch>
```

The move_base package has its on set of required parameters that help it perform efficiently. We `remap` specific topics to allow them to take in input from odometry or laser data and also define some configuration files for parameters of definitions pertaining to the costmapsas well as the local plann that creates a path and navigates the robot along that path. 

Add the configuration file 

```

$ cd ..
$ mkdir config
$ cd config

```
Add the following files from this [repo](https://github.com/udacity/RoboND-Localization-Project/tree/master/config) in the `config` folder:

* `local_costmap_params.yaml`
* `global_costmap_params.yaml`
* `costmap_common_params.yaml`
* `base_local_planner_params.yaml`

The values in these files have some parameters and values defined for simplicity 

Launch Setup
---

Now that we have robot, your map, your localization and navigation nodes

```
$ cd /home/catkin_ws/
$ roslaunch hexapod0 hexapod0_world.launch
```

In a new terminal execute the following command:

``
$ roslaunch hexapod0 amc.launch 

``

Navigate to RViz and and select the required topics to visualize the robot and map. 

* Select “odom” for fixed frame
* Click the “Add” button and
    * add “RobotModel”
    * add “Map” and select first topic/map
      * The second and third topics in the list will show the global costmap, and the local costmap. Both can be helpful to tune your parameters.
    * add “PoseArray” and select topic /particlecloud
      * This will display a set of arrows around the robo
      
      
Note: Similar to the [EKF lab], we can save the above RViz setup in a configuration file and launch RViz with the same configuration every time. This will make the process more efficient for later!


Each arrow is essentially a particle defining the pose of the robot that the localization package created. Our goal is to add/tune the parameters that will help localize the robot better and thereby improve the pose array.

![alt text][image2]
  

In the RViz toolbar, 

Select “2D Nav Goal” click anywhere else on the map and drag from there to define the goal position along with the orientation of the robot at the goal position.


![alt text][image3] 

In order to improve the robot's behavior we tune certain parameters. 

# Paramater Tuning 

The main focus of our project is to explore, add and tune parameters for the amcl and move_base packages. 

Expected results
---

The `move_base` package will help navigate the robot to the goal position by creating or calculating a path from the initial position to the goal, and the `amcl` package will localize the robot. However we must define a method to evaluate the certainity of the algorithm about the robot's pose.

The `PoseArray` in RVIz depicts a certain number of particles, respresented as arrows, around the robot.

![alt text][image3] 


The position and the direction the arrows point in, represent an uncertainty in the robot’s pose. This is a very convenient, and slightly subjective, method to understand how well the algorithm and tuned parameters are performing. Based on the parameters and what values are selected for them, as the robot moves forward in the map, the number of arrows should ideally reduce in number. The algorithm rules out some poses. This is the case for when the robot is closer to walls - it is more certain of its pose because of the laser data, as opposed to when it is roaming in an open area for too long.


Robot Motion
---

In the previous section we used the "2D Nav Goal" button in RViz to make the robot navigate to the goal position. That did not work as intended and resulted in some warnings or errors. We'll look at some of these warnings and how to handle them. 

Transform timeout
-----

There are three different types of maps that are being created/generated in RViz when we run `amcl.launch`.

*  **The world map** - The visualization of the world or environment from Gazebo.
* **The global costmap** - Created by the navigation stack/package. A costmap, which you will learn more about in future lessons, essentially divides the map into a grid where a cell could represent free space or an obstacle. The global costmap is used to generate a long-term path for the robot, such as the path to the goal position from the robot's starting position.
*  **The local costmap** - The local costmap is used to generate a short-term path for the robot. For example, a path that attempts to align itself and the robot with the global path.

The `tf` package, helps keep track of multiple coordinate frames, such as the transforms from these maps, along with any transforms corresponding to the robot and its sensors. Both the `amcl` and `move_base` packages or nodes require that this information be up-to-date and that it has as little a delay as possible between these transforms.

![alt text][image4]

The warning message in the above image is indicative of the maximum allowed delay to either be not defined or to be too low for the system to compensate for. This maximum amount of delay or latency allowed between transforms is defined by the `transform_tolerance` parameter.

The first task is to add and tune this parameter for both the `amcl` node in `amcl.launch` file and for the `move_base` node in the `costmap_common_params.yaml` file. Tuning the value for this parameter is usually dependent on your system.

Once the `transform_tolerance` variable defined and tuned properly, we should be able to visualize all the three maps in RViz without any issues, and the warning should disappear. Only to be replaced by a new warning.

Map Update Loop
---

![alt text][image5]

The warning seems to indicate that the map or costmaps are not getting updated fast enough. The update loop is taking longer than the desired frequency rate of 50 Hz or 0.02 seconds.

How can we fix this? One way might be to get more processing power! But that's not always feasible. Since it's an issue related to the map or the costmaps, why don't we check out the configuration files (yaml) that we created for the move_base node and identify which parameters we can add or update to solve this problem.

Here is a detailed list of parameters corresponding to costmaps in ROS - http://wiki.ros.org/costmap_2d that can help identify the necessary parameters.

Necessaary Parameters:
----

* update freqency
* publish_frequency 
</ To be edited >
As you might notice, a lot of the warnings you seem to be getting are because of system limitations. A larger and more detailed map will result in a large global costmap, and would use more resources. Apart from tuning the frequency with which your map is getting updated and published, you can also modify the dimension and resolution of your global and local costmaps.

Modifying these parameters can help free up some resources, however, decreasing the resolution of your map by too much can lead to loss of valuable information too. For example, in case of small passages, low resolution might cause the obstacle regions to overlap in the local costmap, and the robot might not be able to find a path through the passage.

Note: Modifying the above two parameters might help with the overall response, but it could potentially also help with ensuring that your robot is able to follow the defined local path. This might come in handy to experiment with later as well.
Try running your project again, and define a goal position using the "2D Nav Goal" button in RViz, a short distance from your robot.

Your robot should start moving! That's brilliant!

But, it doesn't seem to be following the path and might be hitting the walls.

In the next section, we will go through some more parameters that can help you tackle this problem and then move on to some of the parameters that could help improve your localization results as well. 
<To be edited />

Parameter Tuning - 2
---

