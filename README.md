# ROS 101
The purpose of this tutorial is achieve basic understanding of ROS nodes and their inner workings such as publishers, subscribers, console logging, and service calls. Part A explains how to download and "launch" existing ROS packages. In Part B you are challenged to write a ROS node by completing provided skeleton code.

This tutorial is intended to be interactive (and is often presented in a workshop format). Complete the *TODO*s after a section before continuing to the next section. Finally, solutions are contained in this tutorial - **AVOID OPENING** `img_pipeline-soln.py` and `img_pipeline-soln.launch` until the tutorial is complete.

## Prerequisites
Ensure the below prerequisites are satisfied before starting the ROS 101 tutorial.
1. A basic USB camera. If using a laptop, the built-in webcam should suffice.

2. Get a working Ubuntu operating system. If you ask around the lab, our preferred methods are to dual-boot or install Ubuntu on an old laptop, but to get started a virtual machine also works.
 - Check this tutorial to [Install Ubuntu Desktop](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop#0) on a real machine. If dual-booting, you may need to read about partitioning your system.
 - This guide explains how to [Run Ubuntu Within Windows Using VirtualBox](https://www.lifewire.com/run-ubuntu-within-windows-virtualbox-2202098). If using macOS, the process is very similar.

    **Note:** Ubuntu 16 is OK for ROS 101 training, but if installing for the first time, choose Ubuntu 18!

3. Install ROS on your new Linux machine: [ROS Installation Instructions](http://wiki.ros.org/ROS/Installation)
  - Follow the installation directions, steps 1.1 through 1.7.
 - Choose the installation best for your current Ubuntu version.
    - Ubuntu 18 > [Install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
    - Ubuntu 16 > [Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

4. Setup the catkin workspace: [Create a ROS Workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)

5. Configure the ROS environment by adding the following `source` command to the `.bashrc` file which ensures the ROS environment is properly initialized each time a terminal is opened.
    `source /opt/ros/<distro>/setup.bash`
- This file is located in the home directory: `~/.bashrc`

    **Note:** You **MUST** source the ROS environment in the terminal shell for ROS to function properly. Sometimes it may appear ROS is "broken", but usually this is caused by files changing slightly and the new environment not being imported.

# ROS 101: Part A
Part A outlines basic ROS structure and explains how to download and "launch" existing ROS packages. The section *TODO*s can be completed using the terminal shell, i.e. command line, and a text editor. By the end you will be able to write your own launch file to launch pre-built or custom ROS packages.

## What is (R)obot (O)perating (S)ystem?
_def._ ROS is an open-source operating system for robots. It provides services, like hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management.

Basic elements:
- nodes
- publisher/subscriber (pub/sub)
    - messages; topics; asynchronous; publishers and subscribers are agnostic
- services (request/response)
- callback functions

### What the hey is a catkin_ws?
**catkin_ws**
- src    -- ROS packages go here.
- devel  -- `source devel/setup.bash` to overlay the workspace onto ROS environment.
- build  -- Built binaries go here. May need to "blow this folder away" when rebuilding.

*TODO: Verify that the `.bashrc` file is configured correctly.*  
*Hint: Step 5 in prerequisites.*  
*Hint: The `.bashrc` file is a hidden file stored in the user home directory: `~/.bashrc`.*

## ROS Software Packages
_def._ [ROS packages](http://wiki.ros.org/Packages) are the smallest organization unit of ROS code. Each package may contain ROS nodes, libraries, executables, scripts, or other third-party modules.

### Downloading pre-built ROS packages
Many ROS packages are pre-built and bundled for software release as debian packages that may be installed using the `apt` package manager. To download pre-built ROS packages, substitute the distro and package name into the following command.

    sudo apt install ros-[distro]-[package_name]

*TODO: Install the ROS [usb cam package](http://wiki.ros.org/usb_cam) from the command line by modifying the above command.*

### Building ROS packages from source
ROS packages can also be "built from source" meaning the source code is downloaded (usually by cloning a repo), then compiled and built locally. This method allows for development of new ROS packages and installing ROS packages without pre-built debian packages. To build ROS packages from source, navigate to `catkin_ws/src` and use the following commands.

    git clone [repo_address]
    cd ..
    catkin_make

ROS packages generally follow the structure of this ros tutorial package.

**ros_img_pipe_tutorial package**
- launch    -- Launch files for ROS package nodes.
- src       -- Code goes here.
- srv       -- ROS service description files.

*TODO: Build the `ros_img_pipe_tutorial` ROS package from source.*  
*Hint: Start by cloning the [ros_img_pipe_tutorial](https://github.com/djoshuadulle/ros_img_pipe_tutorial.git) GitHub repo.*

## Launching ROS Nodes
_def._ [ROS nodes](http://wiki.ros.org/Nodes) are the package executables performing computations and running other functions. Nodes may subscribe to inputs and publish outputs to communicate with other nodes.

### roslaunch
_def._ Launch files are [roslaunch XML](http://wiki.ros.org/roslaunch/XML) files with tags specifying node names, package names, executable names, and other parameters.

"Launching a node" means calling the ROS package launch file from the command line using the `roslaunch` command.

To start the nodes specified in a launch file, navigate to the catkin workspace and substitute the package name containing the file and filename into the following command.

    roslaunch [package_name] [filename].launch

Finally, ROS packages may launch a single node or multiple nodes running in unison. For example, launching a camera node along with a node processing the camera images for other use.

*TODO: From the command line, launch the usb cam node using the launch file in the ros tutorial package.*

### rosrun
_def._ `rosrun` allows you to use the package name to directly run a single node within a package.

This method of starting nodes is useful for:

- Debugging a single node in a larger package, because it prints the Python/C++ error messages to the stdout.
- Launching a specific node from a package of tools (such as rqt-common-plugins explained below).

To run a single node, navigate to the catkin workspace and substitute the package name and node name into the following command.

    rosrun [package_name] [node_name]

*TODO: With the usb cam node launched, run the rqt image view tool to view the usb camera output.*  
*Hint: Check out the "Useful Tools" section below to learn more about the rqt library.*

### roscore
_def._ `roscore` is a collection of nodes and programs that are pre-requisites of a ROS-based system.

There **MUST** be a roscore running for ROS nodes to communicate. If you use `roslaunch`, it automatically starts `roscore` if it is not already running, but sometimes it's useful to keep a roscore running in one terminal while launching other nodes in separate terminals.

To start the roscore, open a terminal and use the command `roscore`.

## Custom ROS Launch Files
It's possible to create launch files to launch nodes you've written, nodes from other packages, or both. For example, the completed `ros_img_pipe_tutorial` package launches the image pipeline node and the usb cam node from a single launch file.

Two essential tags are the <node> tag and the <param> tag.

The required [node tag](https://wiki.ros.org/roslaunch/XML/node) specifies the node to be launched. Similar to launching a node from the command line, the node tag requires the package name and node name as arguments along with the name of the executable that will be run (.py or compiled cpp binary).

The optional [param tag](https://wiki.ros.org/roslaunch/XML/param) defines parameters to be set on the parameter server. These parameters can be retrieved in the code and are useful for passing in required arguments or data.

For full tag descriptions and their usage, see the [roslaunch tag reference](http://wiki.ros.org/roslaunch/XML#Tag_Reference).

## Create Your Own ROS Launch File
The final exercise of ROS 101: Part A is launching the Python executable `img_pipeline-soln.py`. This node subscribes to a raw camera image topic, passes the raw image through an OpenCV image processing pipeline, and publishes the modified image to the processed image topic. By default the node simply passes the raw image through without modification, but different OpenCV image processing functions may be triggered using a rosservice call (see the bonus).

*TODO: In the ros_img_pipe_tutorial package, open the file `usb_cam.launch` stored in the launch folder and look at the structure of the node tag.*

*TODO: Open the launch file `img_pipeline-skeleton.launch` and add a node tag to launch `img_pipeline-soln.py` as a node. **Avoid opening** `img_pipeline-soln.py`!*

*TODO: Launch both the image pipeline node and the usb cam node at the same time (using separate launch files or a single custom launch file), then use the rqt image view tool to view the processed image.*

**BONUS: Process images using the image pipeline node**  
**Once the image pipeline node is running...**  
*TODO: Send a `rosservice call` via the command line to trigger image processing. Possible arguments are: "raw" [default], "flip", "color", "blur", "edge".*  
*Hint: Review the documentation [rosservice](http://wiki.ros.org/rosservice) and [ROS Service Client](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29).*

## Useful Tools
### rostopic
_def._ `rostopic` is a command-line tool for displaying information about ROS topics such as published topics and publishing rate, and printing topics to the screen.

Some of the commonly used commands are:

`rostopic list`    -- print information about active topics  
`rostopic echo`    -- print messages to screen  
`rostopic type`    -- print topic type  
`rostopic hz`    -- display publishing rate of topic  

For full list of commands, see [rostopic command-line tool](http://wiki.ros.org/rostopic#rostopic_command-line_tool).

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
Part B challenges you to write a Python ROS node by completing the provided image pipeline skeleton code. The section *TODO*s will be completed using a text editor and the command line. By the end you will be able to write your own ROS node and trigger node behaviors using a rosservice call.

## ROS APIs
ROS nodes may be written in Python or C++ using either the rospy or roscpp client APIs. Generally packages are written in a single language, but due to the pub/sub architecture of ROS, a package could _technically_ function with nodes in both Python and C++. While this tutorial uses the rospy Python library to create a ROS node, note that the roscpp C++ library has counterparts to each element demonstrated. A major difference between the Python and C++ APIs is the use of node handles in roscpp.

### rospy
_def._ [rospy](http://wiki.ros.org/rospy) is a pure Python client library for ROS. The rospy client API enables Python programmers to quickly interface with ROS Topics, Services, and Parameters.

### Common node elements
- ros init()    -- Initializes a ROS node.
- messages      -- Containers for data; may be custom.
- topics        -- Streams/channels carrying data as messages.
- publisher     -- "Outputs" data to a topic in the form of a message.
- subscriber    -- Requests data from a topic; data receipt triggers a callback.
- callback function  -- Contains code to be executed on receipt of message data; asynchronous execution.
- services (request/response) -- One-to-one, two-way communication between nodes using messages.
- logging       -- The ROS version of standard output, may print to screen or write to log files.

To learn more about these common elements, review the [rospy overview](https://wiki.ros.org/rospy/Overview).

**Note:** To enable a custom ROS node to be launched, you **MUST** ensure the .py file is an executable by using the `chmod` command to change the access permissions of the file.

    chmod +x [node_name].py

Also recall that a node may be debugged by launching the node with `rosrun`, because this allows the Python error messages to print to stdout.

    rosrun [package_name] [node_name]

## Creating a Custom ROS Node
At this point the tutorial moves from this README to the file `img_pipeline-skeleton.py`. This file contains the framework for the image processing node launched at the end of ROS 101: Part A and will function similarly when completed. Read through the code comments from top to bottom and complete the *TODO*s embedded in the code (details below), then return to this README for the final two *TODO*s.

**Complete the image pipeline node:**  
*TODO: Complete the `img_pipeline-skeleton.py` node to process input from the usb cam node, and output the post-processed image.*
- The *TODO*s related to publishing, subscribing, and service calls at lines 31, 36, 82, & 129 are required.
- The *TODO*s related to logging at lines 55, 74, 86, 120, & 132 are optional.

*Hint: If you read through the [rospy documentation](http://wiki.ros.org/rospy/Overview) and are still stuck, look through the [Core ROS Tutorials](https://wiki.ros.org/ROS/Tutorials#Core_ROS_Tutorials).*

**Once the image pipeline node is complete:**  
*TODO: Modify the launch file `img_pipeline-skeleton.launch` to launch the img_pipeline-skeleton.py node (instead of the provided solution file).*

**BONUS: Create your own image processing pipeline**  
Beginning on line 139 are the utility functions for processing and displaying images using OpenCV. The true image processing occurs in the `process_image()` function beginning on line 176.

**Once you're familiar with the image processing options...**  
*TODO: Make your own image processing pipeline by completing the processing template on lines 234-239. Be sure to add your option to the PROCESSING_OPTIONS on line 143.*

*TODO: Send a `rosservice call` via the command line to trigger your custom image processing pipeline.*  
*Hint: Review the [rosservice](http://wiki.ros.org/rosservice) documentation.*

# Additional Resources
[AWS: What is Pub/Sub Messaging?](https://aws.amazon.com/pub-sub-messaging/)
