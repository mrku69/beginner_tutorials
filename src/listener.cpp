/**
 *  @file      listener.cpp
 *  @brief     Listener (subscriber) node implementation
 *  @details   Implementation of the ROS Listener callback.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "listener.hpp"

/**
 * @brief callback for any messages published on chatter.
 */
void Listener::chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM("Listener heard: " << msg->data.c_str());
}
