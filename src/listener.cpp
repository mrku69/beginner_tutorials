/**
 *  @file      listener.cpp
 *  @brief     Listener (subscriber) node implementation
 *  @details   Implementation of the ROS Listener node to listen for any messages published on chatter.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief callback for any messages published on chatter. 
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    // Initialize ROS and name our node "listener"
    ros::init(argc, argv, "listener");

    // Handle for the process node. Will handle initialization
    //   and cleanup of the node
    ros::NodeHandle n;

    // Subscribe to the "chatter" topic to listen for any
    //   messages published on that topic.
    // Set the buffer to 1000 messages
    // Set the callback to the chatterCallback method
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    // "Spin" up a loop to call any callbacks should a message
    //   be received on the "chatter" topic
    ros::spin();

    return 0;
}
