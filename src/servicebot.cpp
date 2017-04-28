/********************************************************************
 *   MIT License
 *  
 *   Copyright (c) 2017 Huei-Tzu Tsai, Steven Gambino
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 ********************************************************************/

/** @file servicebot.cpp
 *  @brief Implementation of class ServiceBot methods
 *
 *  This file contains implemenation of callback function in ServiceBot
 *  class.
 *
 *  @author Huei Tzu Tsai
 *          Steven Gambino
 *  @date   04/27/2017
*/


#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "servicebot/commandService.h"
#include "servicebot.hpp"
#include "action.hpp"

void ServiceBot::initialize(ros::NodeHandle &n) {
    ROS_INFO_STREAM("ServiceBot::initialize");

    nodeHandle = n;

    // Subscribe topic serviceCommand from master to receive messages published on this topic
    commandSub = nodeHandle.subscribe("/servicebot/command", 1000,
                                      &ServiceBot::commandCallback, this);

    // Register to publish topic 
    commandPub = nodeHandle.advertise<std_msgs::String>("/servicebot/command", 1000);

    // Register service with the master
    commandServer =
        nodeHandle.advertiseService("commandService", &ServiceBot::commandService, this);

    return;
}


bool ServiceBot::commandService(
        servicebot::commandService::Request &req,
        servicebot::commandService::Response &resp) {
    std_msgs::String msg;

    ROS_INFO_STREAM("ServiceBot::commandService: receive req = " << req.command << "," << req.action);

    std::stringstream ss;
    ss << req.command << " " << req.action;
    msg.data = ss.str();

    // send messages
    commandPub.publish(msg);

    return true;
}


void ServiceBot::commandCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO_STREAM("ServiceRobot::commandCallback: receive " << msg->data.c_str());

    if (strcmp(msg->data.c_str(), "name") == 0) {
        action.execute(nodeHandle, Action::ACT_NAME);
    } else if (strcmp(msg->data.c_str(), "time") == 0) {
        action.execute(nodeHandle, Action::ACT_TIME);
    } else if (strcmp(msg->data.c_str(), "play music") == 0) {
        action.execute(nodeHandle, Action::ACT_PLAYMUSIC);
    } else if (strcmp(msg->data.c_str(), "stop music") == 0) {
        action.execute(nodeHandle, Action::ACT_STOPMUSIC);
    } else
        ;

    return;
}

