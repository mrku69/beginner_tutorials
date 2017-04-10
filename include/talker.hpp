/**
 *  @file      talker.hpp
 *  @brief     Talker (publisher) support methods definition
 *  @details   Definition of the ROS Talker support methods.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <string>
#include "beginner_tutorials/talkerService.h"

/**
 * @brief Talker class handles any support methods for the talker node
 */
class Talker {
 public:
  /**
   * @brief Talker constructor
   */
  Talker();

  /**
   * @brief method to handle changing the text to be sent on the chatter topic
   */
  bool changeText(beginner_tutorials::talkerService::Request &,
                  beginner_tutorials::talkerService::Response &);

  /**
   * @brief Get the current text that is being sent over the chatter topic
   */
  std::string getText(void) {
    return text;
  }

 private:
  /**
   * @brief Container for the text to send over the chatter topic
   */
  std::string text;
};
