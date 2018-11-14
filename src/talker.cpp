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

/**@file talker.cpp
 *
 * @brief To publish message in a topic
 *
 * @author Nithish Sanjeev Kumar
 * @copyright 2018 , Nithish Sanjeev Kumar All rights reserved

 */

#include <tf/transform_broadcaster.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/changeString.h"

extern std::string temp = "Initialize";


/** @brief Function for the callback function
 *  @param string pointer for Request
 *  @param string pointer for Response
 *  @return bool
 */
bool baseString(beginner_tutorials::changeString::Request &req ,
                beginner_tutorials::changeString::Response &resp) {
  temp = req.newString;
  resp.modifiedString = "String has been changed to" + temp;
  ROS_WARN_STREAM("Modified the base string");
  return true;
}
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * Creating the tf object for further processing
   */
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(2.0, 3.0, 5.0));
  tf::Quaternion q;
  q.setRPY(0, 0, 1.57);
  transform.setRotation(q);


  /**
     * Creation of a service server
     * The service available will fully initialize this parameters and the name
     * Object destructed will close down the service.
     */
  auto server = n.advertiseService("changeString", &baseString);
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  int chatterFrequency = 10;
  if (argc == 2) {
     chatterFrequency = atoi(argv[1]);}
  ROS_INFO("Frequency input [%d]", chatterFrequency);

  ROS_DEBUG_STREAM("User Input Frequency is: " << chatterFrequency);
  if (chatterFrequency < 0) {
    ROS_ERROR_STREAM("Frequency cannot be negative");
    ROS_WARN_STREAM("Resetting frequency to 10");
    chatterFrequency = 10; } else if (chatterFrequency == 0) {
                          ROS_FATAL_STREAM("Frequency cannot be 0");
                          ROS_WARN_STREAM("Resetting frequency to 10");
                          chatterFrequency = 10;
    }

  ros::Rate loop_rate(chatterFrequency);
  ROS_INFO("Frequency set [%d]", chatterFrequency);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << temp << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    /**
     * sendTransform() function publishes the tf frames to the topic
     */
    br.sendTransform(tf::StampedTransform(transform,
                                          ros::Time::now(), "world", "talk"));

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  temp = "Reset";

  return 0;
}
