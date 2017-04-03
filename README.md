# beginner_tutorials

## Overview
This repository is an implementation of various ROS beginner tutorials (found [here](http://wiki.ros.org/ROS/Tutorials)). This ROS package is a simple publisher/subscriber where the publisher sends out a simple string message on the "chatter" topic. The publisher prints out the string message being sent for validation between publisher and subscriber. If the subscriber defined in this package is running, it will listen on the "chatter" topic for the published messages. As it sees messages, it will print the messages to the console prefaced with and "I heard" string. 

The source code for the talker and listener nodes is commented to walk through what the code is doing. In summary, the process for the talker node is as follows: 
1. Initialize ROS 
2. Advertise that we will be publishing messages on the "chatter" topic
3. Publish messages on the "chatter" topic at 10 Hz

Similarly, the process for the listener node is as follows:
1. Initialize ROS
2. Subscribe to the "chatter" topic 
3. Wait for messages
4. When a message is received, call a callback function that prints the message to the console

## Prerequisites / Dependencies
This package requires that [ROS](http://wiki.ros.org/indigo/Installation) is installed as well as [catkin](http://wiki.ros.org/catkin?distro=indigo#Installing_catkin). This was tested using ROS Indigo, however any subsequent versions of ROS should still work. 

## Build Steps
To use this package, a catkin workspace must be setup first. Assuming catkin has been installed, run the following steps in the directory of your choice (a common one is ~/catkin_ws)
```
$ cd <PATH_TO_YOUR_DIRECTORY>
$ mkdir -p catkin_ws/src
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
```
Your workspace should now be setup and you should be able to use ros commands like `roscd` and `rosls`. Note that if you cannot use these commands or can't find ROS packages, try running `source devel/setup.bash` again in your catkin workspace.

To build the beginner_tutorials ROS package in this repository, first clone the repository into the catkin `src` directory:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws/src
$ git clone https://github.com/patnolan33/beginner_tutorials
```
Now simply run catkin_make to build the ROS package.
```
$ cd ..
$ catkin_make
```
You should now see a beginner_tutorials directory in `catkin_ws/build`. 

## Run Subscriber using rosrun
To run this package, first ensure that a roscore is running. Open a new terminal and run `roscore`. Once roscore is up and running, open another terminal and change directories into the catkin_ws from above.
```
$ roscd beginner_tutorials
```
Ensure that your directory has changed to `<PATH_TO_YOUR_DIRECTORY>/catkin_ws/src/beginner_tutorials`. If roscd cannot find the package, you can also simply change directories manually to your catkin_ws. If this is needed, make sure to source the directory again so that you can find the beginner_tutorials package in the next step.

Now, run the listener node:
```
$ rosrun beginner_tutorials listener
```
You shouldn't see anything because the publisher has not been started yet.

## Run Publisher using rosrun
Open a new terminal and change directories into the catkin_ws from above.
```
$ roscd beginner_tutorials
```
Ensure that your directory has changed to `<PATH_TO_YOUR_DIRECTORY>/catkin_ws/src/beginner_tutorials`. If roscd cannot find the package, you can also simply change directories manually to your catkin_ws. If this is needed, make sure to source the directory again so that you can find the beginner_tutorials package in the next step.

Now, run the talker node:
```
$ rosrun beginner_tutorials talker
```
You should see printouts of the message being published in the talker window. You should also see printouts of what the listener is hearing on the chatter topic. You can also pass in an argument to define the publish frequency of the talker node by adding an integer after the previous command (as below).
```
$ rosrun beginner_tutorials talker 10
```
The above command runs the talker node and sets the publish frequency to 10 Hz.

## Run Publisher/Subscriber using roslaunch
Open a new terminal and make sure roscore is running.
```
$ roscore
```
Open a separate terminal and change directories into your catkin workspace. Source the directory and run the launch file as follows:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials service.launch
```
Two terminals will open--one running the talker node and the other running the listener node as before. As a default, you should see `"Initial Text"` being printed in both terminals with message counters. Note that you can pass in an argument to the launch file to change the frequency of the messages being published. To change the frequency to 10 Hz so, use the following command:
```
$ roslaunch beginner_tutorials service.launch freq:=10
```

## Calling the textService using rosservice call
With the publisher and subscriber running (either via launch file or rosrun), we can change the text being published between nodes by calling a rosservice defined in the talker node. Open a new terminal and change directories into your catkin workspace. Source the directory and call the service as follows:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ rosservice call /textService "Other text"
```
What this does is call the `textService` service with the argument "Other text". That argument is used to change the text that is sent via the talker publisher over the `chatter` topic. If the service executes properly, you should now see "Other text" (or whatever string you pass as an argument) being published and heard in the two terminals running the publisher and subscriber, respectively. 
