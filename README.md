# beginner_tutorials

## Table of Contents
- [Overview](#overview)
- [Prerequisites / Dependencies](#prerequisites-dependencies)
- [Build Steps](#build-steps)
- [Run Subscriber using rosrun](#run-subscriber-rosrun)
- [Run Publisher using rosrun](#run-publisher-rosrun)
- [Run Publisher/Subscriber using roslaunch](#run-roslaunch)
- [Calling the textService using rosservice call](#call-service)
- [ROS TF Transform](#tf-transform)
- [Testing](#testing)
  - [rostest](#testing-rostest)
  - [catkin](#testing-catkin)
- [Playback using rosbag](#playback-rosbag)

## <a name="overview"></a> Overview
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

## <a name="prerequisites-dependencies"></a> Prerequisites / Dependencies
This package requires that [ROS](http://wiki.ros.org/indigo/Installation) is installed as well as [catkin](http://wiki.ros.org/catkin?distro=indigo#Installing_catkin). This was tested using ROS Indigo, however any subsequent versions of ROS should still work. 

## <a name="build-steps"></a> Build Steps
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

## <a name="run-subscriber-rosrun"></a> Run Subscriber using rosrun
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

## <a name="run-publisher-rosrun"></a> Run Publisher using rosrun
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

## <a name="run-roslaunch"></a> Run Publisher/Subscriber using roslaunch
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

## <a name="call-service"></a> Calling the textService using rosservice call
With the publisher and subscriber running (either via launch file or rosrun), we can change the text being published between nodes by calling a rosservice defined in the talker node. Open a new terminal and change directories into your catkin workspace. Source the directory and call the service as follows:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ rosservice call /textService "Other text"
```
What this does is call the `textService` service with the argument "Other text". That argument is used to change the text that is sent via the talker publisher over the `chatter` topic. If the service executes properly, you should now see "Other text" (or whatever string you pass as an argument) being published and heard in the two terminals running the publisher and subscriber, respectively. 

## <a name="tf-transform"></a> ROS TF Transform
The talker node also broadcasts a ROS TF transform relative to the world frame (`/talk` and `/world`, respectively). We can inspect this transform by first running the talker node and then using `tf_echo` to print the transformation as it is broadcast. Assuming a roscore is already running and the talker node is already running (see [Run Publisher using rosrun](#run-publisher-rosrun)), open a new terminal and run the following:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ rosrun tf tf_echo /world /talk
```
You should see an output similar to the following:

```
At time 1491868139.860
- Translation: [2.000, 2.000, 0.000]
- Rotation: in Quaternion [1.000, 0.000, 0.000, 0.000]
            in RPY [3.142, -0.000, 0.000]
At time 1491868140.859
- Translation: [2.000, 2.000, 0.000]
- Rotation: in Quaternion [1.000, 0.000, 0.000, 0.000]
            in RPY [3.142, -0.000, 0.000]
At time 1491868141.859
- Translation: [2.000, 2.000, 0.000]
- Rotation: in Quaternion [1.000, 0.000, 0.000, 0.000]
            in RPY [3.142, -0.000, 0.000]
```
If you want to see the transform tree (i.e. how the `/world` and `/talk` frames are related graphically), simply run `rosrun rqt_tf_tree rqt_tf_tree` when the talker node is running. Alternatively, you can also save a PDF of the transform tree using `rosrun tf view_frames`. This will save a PDF file in the current directory named `frames.pdf`.

## <a name="testing"></a> Testing
An integration test using the rostest/gtest framework can be run to ensure integration of any new components are not breaking legacy code. To run the tests, you can use either `rostest` or `catkin`:

### <a name="testing-rostest"></a> rostest
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ rostest beginner_tutorials talkerTest.launch
```
### <a name="testing-catkin"></a> catkin
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ catkin_make run_tests_beginner_tutorials
```

## <a name="playback-rosbag"></a> Playback using rosbag
The package `rosbag` is a common ROS tool that is used to record and playback ROS topic messages. The launch file for this project accepts a boolean flag called `record_talker` that toggles rosbag recording if included (true for record, false for do not record). To run both the publisher and subscriber node and record the published topics, run:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials service.launch record_talker:=true
```
rosbag will save a file named `talker.bag` in the `~/.ros/` directory. To inspect it, simply change into the directory with `cd ~/.ros/` and run `rosbag info talker.bag`. This will output various information about the file, such as how long the file recorded, the start and end times, file size, the number of messages, and the topics recorded. 

We can playback this recorded data to recreate a recorded scenario. Assuming a rosbag recording has taken place according to the above process, run the listener node in a new terminal using:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials listener
```
In another terminal, playback the rosbag file by executing:
```
$ cd ~/.ros/
$ rosbag play talker.bag
```
In the rosbag terminal, you will see an indication that the rosbag file is running. In the listener terminal, you should see output messages similar to what was output in normal operation with another publisher node running. 

*NOTE: rosbag does not record services, so they will not be played back when in playback mode.*


