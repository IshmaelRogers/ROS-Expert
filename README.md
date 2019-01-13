# Ishmael Rogers
# Robotics Software Engineer
# Building Robots With ROS
# Infinitely Deep Robotics Group, LLC
# 2019




ROS Architecture 

The ROS architecture has been designed and divided into three sections or levels of concepts:

# The Filesystem level - How ROS is internally formed

1. Folder structure
2. Minimum number of files that it needs to work

a. Packages - Contain the minimum structure and content to create a program within ROS. (nodes, configuration files, etc.)
b. Package manifests - Provide information about a package, licenses, dependencies, compilation flags. It is managed with the *package.xml* file
c. Metapackages - Is used to aggregate several packages in a group. (navigation stack)
d. Metapackage manifests - Similat to normal packages except thaey contain an export tag in XML.
e. Message types - a message is the information that a process sends to other process. Message descriptions are stored in *my_package/msg/MyMessageType.msg*
f. Service types - define the request and response data structures for services provided by each process in ROS. Service types are store in *my_package/srv/MyServiceType.srv*

![alt text][image1]

The workspace
---

A folder that contains packages that contain the source files and the environment provides us with a way to compile those packages. Allows us to centralize all of the developments. 

- The source space (src folder) contains the packages, projects, clone packages, etc. The CMakeLists.txt file resides here because it is invoked by cmake when packages are configured in the workspace. 

NOTE: This file is created with the catkin_init_workspace command

- The build folder contains the cmake and catkin to keep the cache information, configuration and other intermediate files for packages and projects.

- Development (devel) folder is used to keep the compiled programs. Use this to test the programs without the installation step. 

![alt text][image2]


2 ways to build packages with catkin
--
1. Use standard CMake workflow to compile one package at a time

```
$ cmake packageToBuild/
$ make

```
2. Use *catkin_make* command to compile all the packages 

```
$ cd workspace
$ catkin_make

```

NOTE: Both commands build the executable in the build apace directory configured in ROS

Packages
--
Typical packages structure ROS include the following

* include/package_name/: This directory includes the headers of the libraries that you would need.
* msg/: If you develop nonstandard messages, put them here.
* scripts/: These are executable scripts that can be in Bash, Python, or any other scripting language.
* src/: This is where the source files of your programs are present. You can create a folder for nodes and nodelets or organize it as you want.
* srv/: This represents the service (srv) types.
* CMakeLists.txt: This is the CMake build file. 
* package.xml: This is the package manifest.

To create, modify or work with packages, ROS uses the following tools:

* rospack: This command is used to get information or find packages in the system.
* catkin_create_pkg: This command is used when you want to create a new package.
* catkin_make: This command is used to compile a workspace
* rosdep: This command installs the system dependencies of a package.
* rqt_dep: This command is used to see the package dependencies as a graph. If you want to see the package dependencies as a graph, you will find a plugin called package graph in rqt. Select a package and see the dependencies.

To move between packages and their folders and files, ROS comes with the *rosbash* package. It provides commands that are similar to Linux commands:

* roscd: This command helps us change the directory. This is similar to the cd command in Linux.
* rosed: This command is used to edit a file.
* roscp: This command is used to copy a file from a package.
* rosd: This command lists the directories of a package.
* rosls: This command lists the files from a package. This is similar to the ls command in Linux.

NOTE: Every package must contain a package.xml file that is used to specify information about the package. 

Two typical tags that are used in the package.xml file are 

1. *<build_depend>* 
2. *<run_depend>*

*<build_depend>* 

shows which package must be installed before installing the current package.

*<run_depend>*

shows what packages are necessary for running the code of the package

Metapackages
---

Special packages with only one file inside, *package.xml*. It is used to refer to other packages that are normally grouped following a feature-like functionality.

NOTE: To locate the *ros_tutorials* metapackage use the following command:

```
$ rosstack find ros_tutorials

```

Messages 
---

ROS uses a simplified message description language to describe the data values that ROS nodes publish.

NOTE: With this description, ROS can generate the right source code for these types of messages in several programming languages.

Messages are stored in the msg / folder of the package. Certain files with the .msg extension define the messages.

Messages must have two main parts

1. *fields* - defines the type of data to be transmitted in the message i.e *int32*, *float32*, *strings*, *type1*, *type2*, etc. 
2. *constants* - define the name of the field 

Example of a message 

*int32 id*
*float32 vel*
*string name*

Figure 4. shows the standard message types in ROS

![alt text][image3]

The ROS header type
---
A special type used to add the time, frame, and sequence number. This allows us to have the messages numbered to see who is sending the message and to have more functions that are transparent for the user and that ROS is handling. It is possible to record the timestamp and frame of what is happening with the robot.


The header type contains the following fields:

*unit32 seq*
*time stamp*
*string frame_id*

View the structure using the following command
```
$ rosmsg show std_msgs/Header

```
Services
---

ROS uses a simplified service description language to describe ROS service types. This builds directly upon the ROS msg format to enable request/response communication between nodes. Service descriptions are stored in .srv files in the srv/ subdirectory of a package.

To call services, the package name along with the service name is needed.

To refer to the *sample_package1/srv/sample1.srv* file, use *sample_package1/sample1*

To create a service, use the service generator. These tools generate code from an intitial specification of the service. The 
*gensrv()* line is added to the CMakeLists.txt file



# The Computation Graph level - Where the communication process and systems happen. How ROS:

1. Sets up systems 
2. Handles process 
3. Communicate with more than a single computer


ROS creates a network where all the processes are connected. Any node in the system can access this network, interact with other nodes, see the information that they are sending, and transmit data to the network:


![alt text][image4]

* Nodes: Are processes where computation is done. To develop a process that can interact with other nodes, a node with is process to connect it to the ROS network is created. Having many nodes to control single functions is better than having a large node that makes everything happen in the system. Nodes are written using either the roscpp or rospy client library.

* Master: The master provides the registration of names and the lookup service to the rest of the nodes. It also establishes connections between the nodes. 

NOTE: For a distributed system, the master is in one computer and nodes can be executed in this or other computers.

* Parameter Server: Allows the possibility of using keys to store data in a central location. Nodes can be configured while running or to change the working parameter of a node.

* Messages: Nodes communicate with each other through messages. A message contains data the provides information to other nodes. 

* Topics: Each message must have a name to be routed by the ROS network. 
  1. Publishing a topic - a node is sending data
  2. Subscribing to a topic - a node is receiving topics from other nodes.
  
NOTE: Topic names must be unique to avoid problems and confusion between topics with the same name.

* Services: Used to retrieve answers from a node.

* Bags: A format to save and playback the ROS message data. Bags are essential to such as sensor data. Bags are used frequently with complex robots.

RESUME at page 51 WITH ROS BOOK

# The Community level - Tools and concepts to share knowledge, algorithms and code between developers.

