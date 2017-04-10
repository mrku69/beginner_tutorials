/**
 * @file main.cpp
 * @brief Unit tests
 * @details This file is used to run all unit tests for the ROS Talker Node
 * @author Patrick Nolan (patnolan33)
 * @copyright BSD
 */

#include <ros/ros.h>
#include <gtest/gtest.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "talkerTest");

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
