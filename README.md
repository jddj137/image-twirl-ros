# ROS 101
## Prerequisites
1. Get a working Ubuntu operating system. If you ask around the lab, our preferred methods are to dual-boot or install Ubuntu on an old laptop, but to get started a virtual machine also works.
 - Check this tutorial to [Install Ubuntu Desktop](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop#0) on a real machine. If dual-booting, you may need to read about partitioning your system.
 - This guide explains how to [Run Ubuntu Within Windows Using VirtualBox](https://www.lifewire.com/run-ubuntu-within-windows-virtualbox-2202098). If using macOS, the process is very similar.  


 **Note:** Ubuntu 16 is OK for ROS 101 training, but Marine Robotics code only runs on Ubuntu 18!  

2. Install ROS on your new Linux machine.  
[ROS Installation Instructions](http://wiki.ros.org/ROS/Installation)
  - Follow the installation directions, steps 1.1 through 1.7.
 - Choose the installation best for your current Ubuntu version.
	- Ubuntu 18 > [Install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
	- Ubuntu 16 > [Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

3. Setup the catkin workspace.   
[Create a ROS Workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)

4. Configuring the ROS environment.   

 - Adding the following two `source` commands to the `.bashrc` file, which is run every time the terminal is opened, ensures that the ROS environment is properly initialized. The `.bashrc` file is located in the home directory.

    source /opt/ros/<distro>/setup.bash
    source [path to catkin_ws]/catkin_ws/devel/setup.bash

 **Note:** You **MUST** source the ROS environment into the terminal shell, for ROS to function properly. Sometimes it may appear ROS is "broken", but usually this is caused by files changing slightly and the new environment not being imported.

5. Log in to your [Georgia Tech GitHub Enterprise](https://support.cc.gatech.edu/support-tools/faq/what-gt-github-enterprise) account and set-up a profile.

# ROS 101: Part A
## What is (R)obot (O)perating (S)ystem?
_def._ ROS is an open-source operating system for robots. It provides services, like hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management.

- nodes
- publisher/subscriber (pub/sub)
    - messages; topics; asynchronous; publishers and subscribers are agnostic
- services (request/response)
- callback functions

### What the hey is a catkin_ws?
**catkin_ws**  
- src     -- ROS packages go here.  
- devel -- `source devel/setup.bash` to overlay this workspace onto ROS environment.  
- build  -- Built binaries go here. May need to "blow this folder away" when rebuilding.  

*TODO: Verify that the `.bashrc` is configured correctly.*  
*Hint: The `.bashrc` file is a hidden file stored in the user home directory: `~/.bashrc`.*

## ROS Software Packages
_def._ [ROS packages](http://wiki.ros.org/Packages) are the smallest organization unit of ROS code. Each package may contain ROS nodes, libraries, executables, scripts, or other third-party modules.

### Downloading pre-built ROS packages
Many ROS packages are pre-built and bundled for software release as debian packages that may be installed using the `apt` package manager.

    sudo apt install ros-[distro]-[package_name]

*TODO: Install the ROS [usb cam package](http://wiki.ros.org/usb_cam) from the command line by modifying the above command.*

### Building ROS packages from source
ROS packages can also be "built from source" meaning the source code is downloaded (usually by cloning a repo), then compiled and built locally. This method also allows for development of individual ROS packages.

Navigate to `catkin_ws/src`.
   
    git clone [repo_name]
    cd ../..
    catkin_make

*TODO: Build the adept tutorials ROS package from source.*  
*Hint: Start by cloning the [adept tutorials](https://github.gatech.edu/Adept/adept_tutorials) GitHub repo.*

## Launching ROS Nodes
### roslaunch
_def._ Launch files are [roslaunch XML](http://wiki.ros.org/roslaunch/XML#Tag_Reference) files with tags specifying node names, package names, executable names, and other parameters.

Generally, ROS packages are comprised of multiple nodes running in unison.
To consecutively launch the required nodes, ROS packages contain "launch files".

**ros_tutorials package**  
- launch    -- Launch files for ROS package nodes.  
- src       -- Code goes here.  
- srv       -- ROS service description files.

From the catkin workspace, use the command `roslaunch` to start the nodes specified in a launch file.

    roslaunch [package] [filename].launch

*TODO: From the command line, launch the usb cam node using the launch file in the ros tutorials package.*  

### rosrun
_def._ `rosrun` allows you to use the package name to directly run a node within a package.

This method of starting nodes is useful for:

- Debugging a single node in a larger package, because it prints the Python error messages to the stdout.  
- Launching a specific node from a package of tools (such as rqt-common-plugins explained below).

    rosrun [package_name] [node_name]

*TODO: Run the rqt image view tool to view the usb camera output.*

### roscore
_def._ roscore is a collection of nodes and programs that are pre-requisites of a ROS-based system.

**Note:** There **MUST** be a roscore running for ROS nodes to communicate.

To launch,  use the `roscore` command.
If you use `roslaunch`, it automatically starts `roscore` if it is not already running.

## Creating a Custom ROS Launch File
*TODO: Complete the launch file `img_pipeline.launch` and launch the image pipeline node.*  

**Once the image pipeline node is running:**  
*TODO: Send a `rosservice call` via the command line to trigger image processing.*  
*Hint: Review the [rosservice](http://wiki.ros.org/rosservice) documentation.*

## Useful Tools 
### rostopic
rostopic contains the [rostopic command-line tool](http://wiki.ros.org/rostopic#rostopic_command-line_tool) for displaying debug information about ROS Topics, including publishers, subscribers, publishing rate, and ROS Messages.

Some of the commonly used commands are:

`rostopic list`    -- print information about active topics  
`rostopic echo`    -- print messages to screen  
`rostopic type`    -- print topic type  
`rostopic hz`    -- display publishing rate of topic  

### RQT Common Plugins
The [rqt common plugins](http://wiki.ros.org/rqt_common_plugins) library provides a suite of useful backend GUI tools for visualizing and interacting with ROS nodes in real time.

Some of the commonly used commands are:

`rosrun rqt_image_view rqt_image_view`     -- GUI plugin for displaying images  
`rosrun rqt_reconfigure rqt_reconfigure`    -- view and edit parameters accessible via dynamic_reconfigure    
`rosrun rqt_topic rqt_topic`    -- GUI plugin for displaying debug information about ROS topics    
`rosrun rqt_logger_level rqt_logger_level`    -- GUI plugin for configuring the logger level of ROS nodes    
`rosrun rqt_graph rqt_graph`    -- GUI plugin for visualizing the ROS computation graph

To run any rqt plugin, type in a single command `rqt`, then select any plugins you want from the GUI that launches afterwards.

# ROS 101: Part B
## Building a Custom ROS Node
ROS nodes may be written in Python or C++ and, depending the package architecture, the package could function with nodes in both Python and C++. This tutorial explains using the Python library to create a ROS node, but note that there are C++ counterparts to each element explained.

### rospy
_def._ [rospy](http://wiki.ros.org/rospy) is a pure Python client library for ROS. The rospy client API enables Python programmers to quickly interface with ROS Topics, Services, and Parameters.


### Node elements
- rospy.init()
- messages
- publisher
    - topics    
- subscriber
    - callback function
- services (request/response)
- logging

**Note:** To enable a custom ROS node to be launched, you **MUST** ensure the .py file is an executable by using the `chmod` command to change the access permissions of the file.

    chmod +x [node_name].py

Also recall that a node may be debugged by launching the node with `rosrun`, because this allows the Python error messages to print to stdout.

    rosrun [package_name] [node_name]

*TODO: Modify the permissions of `img_pipeline_B.py` so it runs as an executable node.*

*TODO: Complete the `img_pipeline_B.py` node to process input from the usb_cam node, and output the post-processed image. The file is commented with required and optional to-do's.*  
*Hint: After reading through the [rospy documentation](http://wiki.ros.org/rospy/Overview), take a look at the [VRX fork of ADePT-ROS](https://github.gatech.edu/vrx/ADePT-ROS).*

**Once the img_pipeline node is complete:**  
*TODO: Modify the launch file `img_pipeline.launch` to launch the image_pipeline_B node.*  
*TODO: Send a `rosservice call` via the command line to trigger your custom image processing pipeline.*  
*Hint: Review the [rosservice](http://wiki.ros.org/rosservice) documentation.*

# Additional Resources
[aws: What is Pub/Sub Messaging?](https://aws.amazon.com/pub-sub-messaging/)  
[ROS Service Client](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)  

