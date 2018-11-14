/**
 * MIT License

 * Copyright (c) 2018 Nithish Sanjeev Kumar

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**@file testTalker.cpp
 *
 * @brief To test the talker node
 *
 * @author Nithish Sanjeev Kumar
 * @copyright 2018 , Nithish Sanjeev Kumar All rights reserved

 */


#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/changeString.h"
#include "std_msgs/String.h"


/** @brief Test for checking the availability of service
 *  @param none
 *  @return none
 */

TEST(testTalker, checkServiceAvailability) {
ros::NodeHandle nh;
auto client = nh.serviceClient<beginner_tutorials::changeString>("changeString");
EXPECT_TRUE(client.exists());
}


/** @brief Test for checking the working of the changeString service
 *  @param none
 *  @return none
 */
TEST(testTalker, checkStringModification) {

ros::NodeHandle nh;
auto client = nh.serviceClient<beginner_tutorials::changeString>("changeString");
beginner_tutorials::changeString::Request req;
beginner_tutorials::changeString::Response resp;
req.newString = "Nithish116316958";
client.call(req,resp);
EXPECT_STREQ("String has been changed toNithish116316958",resp.modifiedString.c_str());

}
