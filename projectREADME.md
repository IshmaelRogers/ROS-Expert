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
  
*`` <world>``: The world element defines the world description and several properties pertaining to that world. In this example, you are adding a ground plane, a light source, and a camera to your world. Each model or property can have further elements that describe it better. For example, the ```camera``` has a ``pose`` element which defines its position and orientation.
  
* ```<include>```: The include element, along with the ``<uri>`` element, provide a path to a particular model. In Gazebo there are several models that are included by default, and you can include them in creating your environment.
