/**
 *  @file      talker.cpp
 *  @brief     Talker (publisher) node implementation
 *  @details   Implementation of the ROS Talker support methods.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */

#include "talker.hpp"

/**
 * @brief Talker constructor
 */
Talker::Talker()
    : text("Initial text") {
}

/**
 * @brief method to handle changing the text to be sent on the chatter topic
 */
bool Talker::changeText(beginner_tutorials::talkerService::Request &req,
                        beginner_tutorials::talkerService::Response &resp) {

  text = req.text;
  resp.resp = "GOOD";

  ROS_INFO_STREAM("Change talker text to [" << text << "]");
  ROS_INFO_STREAM("Response for client: " << resp.resp);

  return true;
}
