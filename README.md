# Image Twirl ROS
Image Twirl ROS is a basic ROS package for pre/post image processing. The user specifies a
series of image processing operations, i.e. an 'image twirl', and the parameters for each
operation. The input image passes through these operations and the altered image is output.

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
