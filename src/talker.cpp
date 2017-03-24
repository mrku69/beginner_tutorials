/**
 *  @file      talker.cpp
 *  @brief     Talker (publisher) node implementation
 *  @details   Implementation of the ROS Talker node to publish a string on chatter.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    // Initialize ROS and name our node "listener"
    ros::init(argc, argv, "talker");

    // Handle for the process node. Will handle initialization and
    //   cleanup of the node
    ros::NodeHandle n;

    // Set up a publisher to the "chatter" topic to publish messages
    //   on that topic.
    // Set the buffer to 1000 messages
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // Set up the publisher rate to 10 Hz
    ros::Rate loop_rate(10);

    // Initialize a counter for the number of messages we've sent
    //   so we can create a unique string
    int count = 0;
    while (ros::ok()) {
        // Container for the string we will publish
        std_msgs::String msg;

        // Populate the string with data
        std::stringstream ss;
        ss << "Patrick Nolan's custom string message " << count;
        msg.data = ss.str();

        // Print the message data before publishing
        ROS_INFO("%s", msg.data.c_str());

        // Publish the message on the "chatter" topic
        chatter_pub.publish(msg);

        // "Spin" a callback in case we set up any callbacks
        //   (we haven't in this example)
        ros::spinOnce();

        // Sleep for the remaining time until we hit our 10 Hz rate
        loop_rate.sleep();

        // Increment the counter
        ++count;
    }

    return 0;
}
