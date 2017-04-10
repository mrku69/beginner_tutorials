/**
 * @file talkerTest.cpp
 * @brief Unit tests
 * @details This file is used to run all unit tests for the Talker ROS node
 * @author Patrick Nolan (patnolan33)
 * @copyright MIT License.
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include <gtest/gtest.h>
#include <string>
#include "talker.hpp"
#include "beginner_tutorials/textService.h"

/**
 * @brief Test that the transform is being broadcast
 */
TEST(TestSuite, testTransform) {
  ros::NodeHandle n;

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.waitForTransform("/world", "/talk",
        ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/world", "/talk",
        ros::Time(0), transform);

    // Check the origin is correct:
    EXPECT_EQ(2, transform.getOrigin().x());
    EXPECT_EQ(2, transform.getOrigin().y());
    EXPECT_EQ(0, transform.getOrigin().z());

    // Check the rotation is correct:
    EXPECT_EQ(1, transform.getRotation().x());
    EXPECT_EQ(0, transform.getRotation().y());
    EXPECT_EQ(0, transform.getRotation().z());
    EXPECT_EQ(0, transform.getRotation().w());
  } catch (tf::TransformException ex) {
    ADD_FAILURE();
  }
}

/**
 * @brief Test that the textService service from the talker node exists
 */
TEST(TestSuite, testTextServiceExists) {
  ros::NodeHandle n;

  // Register client to "textService" service
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::textService>("textService");

  // Test that the service exists after 1 second
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

/**
 * @brief Test that calling the service returns with a successful response
 */
TEST(TestSuite, testTextServiceSuccess) {
  ros::NodeHandle n;

  // Register client to "textService" service
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::textService>("textService");

  // Call service and test response:
  beginner_tutorials::textService srv;
  srv.request.text = "Other text";
  client.call(srv);

  EXPECT_STREQ("GOOD", srv.response.resp.c_str());
}

