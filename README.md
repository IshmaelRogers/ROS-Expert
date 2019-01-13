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
e. Message types - a message is the information that a process sends to other process. Message descriptions are stored in *my_package/msg/MyMessageType.msg*
f. Service types - define the request and response data structures for services provided by each process in ROS. Service types are store in *my_package/srv/MyServiceType.srv*

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

The Computation Graph level - Where the communication process and systems happen. How ROS:
---
1. Set up systems 
2. Handles process 
3. Communicate with more than a single computer


The Community level - Tools and concepts to share knowledge, algorithms and code between developers.

