/**
 *  @file      listener_node.cpp
 *  @brief     Listener (subscriber) node implementation
 *  @details   Implementation of the ROS Listener node to listen for any messages published on chatter.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include "listener.hpp"
#include "beginner_tutorials/textService.h"

int main(int argc, char **argv) {
  // Initialize ROS and name our node "listener"
  ros::init(argc, argv, "listener");

  // Handle for the process node. Will handle initialization
  //   and cleanup of the node
  ros::NodeHandle n;

  // Listener object container
  Listener listener;

  // Register client to "textService" service
  ros::ServiceClient client = n.serviceClient < beginner_tutorials::textService
      > ("textService");

  if (client.exists()) {
    ROS_DEBUG_STREAM("textService is advertised and available");
  } else {
    ROS_ERROR_STREAM("textService is either not advertised or not available.");
  }

  // Wait until the talker node is started and service becomes active
  // -If it doesn't come active within 30 seconds, issue a warning (non-fatal)
  bool rosRet;
  rosRet = ros::service::waitForService("textService", 30000);

  // If we see the service, let the user know the service is available
  if (rosRet == true) {
    ROS_INFO_STREAM("textService is available");
  } else {
    // If we don't see a service for 30 seconds, warn the user that no service was seen
    ROS_WARN_STREAM("Timeout waiting for textService");
  }

  // Subscribe to the "chatter" topic to listen for any
  //   messages published on that topic.
  // Set the buffer to 1000 messages
  // Set the callback to the chatterCallback method
  ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::chatterCallback,
                                    &listener);

  // Alert user of the success or failure of the topic subscription
  if (sub) {
    ROS_DEBUG_STREAM("Subscribed to chatter topic");
  } else {
    ROS_FATAL_STREAM(
        "Failed to subscribe to the chatter topic. Closing listener...");
    return -1;
  }

  // "Spin" up a loop to call any callbacks should a message
  //   be received on the "chatter" topic
  ros::spin();

  return 0;
}
