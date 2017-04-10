/**
 *  @file      talker_node.cpp
 *  @brief     Talker (publisher) node implementation
 *  @details   Implementation of the ROS Talker node to publish a string on chatter.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <sstream>
#include "talker.hpp"
#include "beginner_tutorials/textService.h"

int main(int argc, char **argv) {
  // Initialize ROS and name our node "talker"
  ros::init(argc, argv, "talker");

  // set default publish frequency to 1 Hz
  int frequency = 1;

  // Check if we have an argument being passed in
  if (argc > 1) {
    // If we have an argument passed in, set the frequency to the value
    ROS_DEBUG_STREAM("argv[1] is " << argv[1]);
    frequency = atoi(argv[1]);
  }

  // Handle for the process node. Will handle initialization and
  //   cleanup of the node
  ros::NodeHandle n;

  // Talker object
  Talker talker;

  // Register service with the master
  ros::ServiceServer server = n.advertiseService("textService",
                                                 &Talker::changeText, &talker);

  // Alert the user to the frequency in which we will publish messages
  ROS_INFO_STREAM("Publishing messages at " << frequency << "Hz");

  // Publish the "chatter" topic
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);

  // Set up the publisher rate to the desired frequency
  ros::Rate loop_rate(frequency);

  // TF Broadcaster and TF containers:
  tf::TransformBroadcaster tfBroadcaster;
  tf::Transform transform;

  // Initialize a counter for the number of messages we've sent
  //   so we can create a unique string
  int count = 0;
  while (ros::ok()) {
    // Container for the string we will publish
    std_msgs::String msg;

    // Populate the string with data
    std::stringstream ss;
    ss << "[" << talker.getText() << " ; Message count: " << count << "]";
    msg.data = ss.str();

    // Print the message data before publishing
    ROS_INFO_STREAM(msg.data.c_str());

    // Publish the string to anyone listening
    chatter_pub.publish(msg);

    // Set our transform:
    transform.setOrigin(tf::Vector3(2.0, 2.0, 0.0));
    transform.setRotation(tf::Quaternion(1, 0, 0, 0));

    // Broadcast our transform
    //  (arg1 = transform, arg2 = timestamp, arg3 = parent TF, arg4 = TF name)
    tfBroadcaster.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    // "Spin" a callback in case we set up any callbacks
    ros::spinOnce();

    // Sleep for the remaining time until we hit our 10 Hz rate
    loop_rate.sleep();

    // Increment the counter
    ++count;
  }

  return 0;
}
