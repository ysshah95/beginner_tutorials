/**
 * MIT License
 * 
 * Copyright (c) 2018 Yash Shah
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file talker_test.cpp
 * @Auther Yash Shah
 * @version 1.0
 * @brief test file for ROS TEST
 * 
 * This program is to test the talker node. 
 * 
 * @copyright MIT License (c) 2018  
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/change_talker_string.h"
 /**
 * @brief      Tests whether the service exists
 *
 * @param[in]  TESTSuite          gtest framework
 * @param[in]  testServiceExists  Name of the test
 */
TEST(TESTSuite, testexistance) {
  // Create node handle
  ros::NodeHandle n;
  // Register the client to the service
  ros::ServiceClient client =
        n.serviceClient<beginner_tutorials::change_talker_string>
                ("change_talker_string");
  // Check if the service exist
  bool ok(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(ok);
}

 /**
 * @brief      Tests whether the service changes the text
 *
 * @param[in]  gtest framework
 * @param[in]  Name of the test
 */
TEST(TESTSuite, test_Service) {
  // Create a node handle
  ros::NodeHandle n;
  // Register the client to the service
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::change_talker_string>
                ("change_talker_string");
  beginner_tutorials::change_talker_string srv;
  // Set the input text
  srv.request.request_service = "example_test";
  // Call the service
  client.call(srv);
  // Check the response
  EXPECT_STREQ("example_test", srv.response.response_service.c_str());
}
