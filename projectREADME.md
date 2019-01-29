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






  

