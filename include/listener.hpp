/**
 *  @file      listener.hpp
 *  @brief     Listener (subscriber) support methods definition
 *  @details   Listener class support methods definitions
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * @brief Listener class handles any callbacks that need to be defined for any topics we subscribe to
 */
class Listener {
 public:
  /**
   * @brief callback for any messages published on chatter.
   */
  void chatterCallback(const std_msgs::String::ConstPtr&);
};

