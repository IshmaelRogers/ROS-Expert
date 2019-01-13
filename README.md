ROS Architecture 

The ROS architecture has been designed and divided into three sections or levels of concepts:

The Filesystem level - How ROS is internally formed
---
1. Folder structure
2. Minimum number of files that it needs to work

a. Packages - Contain the minimum structure and content to create a program within ROS. (nodes, configuration files, etc.)
b. Package manifests - Provide information about a package, licenses, dependencies, compilation flags. It is managed with the *package.xml* file
c. Metapackages - Is used to aggregate several packages in a group. (navigation stack)
d. Metapackage manifests - Similat to normal packages except thaey contain an export tag in XML.
e. Message types - a message is the information that a process sends to other process. Message descriptions are stored in * my_package/msg/MyMessageType.msg *
f. Service types - define the request and response data structures for services provided by each process in ROS. Service types are store in * my_package/srv/MyServiceType.srv *

The workspace
---

A folder that contains packages that contain the source files and the environment provides us with a way to compile those packages. Allows us to centralize all of the developments. 

- The source space (src folder) contains the packages, projects, clone packages, etc. The CMakeLists.txt file resides here because it is invoked by cmake when packages are configured in the workspace. 

NOTE: This file is created with the catkin_init_workspace command

- The build folder contains the cmake and catkin to keep the cache information, configuration and other intermediate files for packages and projects.

- Development (devel) folder is used to keep the compiled programs. Use this to test the programs without the installation step. 

2 ways to build packages with catkin
--
1. Use standard CMake workflow to compile one package at a time

```
$ cmake packageToBuild/
$ make

```



The Computation Graph level - Where the communication process and systems happen. How ROS:
---
1. Set up systems 
2. Handles process 
3. Communicate with more than a single computer


The Community level - Tools and concepts to share knowledge, algorithms and code between developers.

