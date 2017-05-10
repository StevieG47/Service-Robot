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
 *  This file contains implemenation of methods in ServiceBot class which
 *  receives voice commands or commands from console and executes the commands 
 *  accordingly.
 *
 *  @author Huei Tzu Tsai \n
 *          Steven Gambino
 *  @date   04/27/2017
*/


#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>
#include <algorithm>
#include "servicebot/commandService.h"
#include "servicebot.hpp"
#include "action.hpp"

using std::string;

void ServiceBot::initialize(ros::NodeHandle &n) {
    ROS_INFO_STREAM("ServiceBot::initialize");

    nodeHandle = n;

    // subscribe topic /servicebot/command from master to receive voice
    // or console commands
    commandSub = nodeHandle.subscribe("/servicebot/command", 1000,
                                      &ServiceBot::commandCallback, this);

    // register to publish commands received from console to
    // /servicebot/commands
    commandPub =
        nodeHandle.advertise<std_msgs::String>("/servicebot/command", 1000);

    // register service with the master to receive commands from console
    commandServer =
        nodeHandle.advertiseService("commandService",
                                    &ServiceBot::commandService, this);

    // initialize action
    action.initialize(nodeHandle);

    return;
}


bool ServiceBot::commandService(
        servicebot::commandService::Request &req,
        servicebot::commandService::Response &resp) {
    std_msgs::String msg;

    ROS_INFO_STREAM("ServiceBot::commandService: receive req = "
                    << req.command << " " << req.args);

    std::stringstream ss;
    ss << req.command << "," << req.args;
    msg.data = ss.str();

    // send messages
    commandPub.publish(msg);

    resp.resp = "OK";

    return true;
}


void ServiceBot::commandCallback(const std_msgs::String::ConstPtr& msg) {
    string cmd;
    string args;
    std::istringstream lineStream(msg->data.c_str());

    // parse string message into command and arguments
    getline(lineStream, cmd, ',');
    getline(lineStream, args, ',');

    // convert command and args to lower case
    std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
    std::transform(args.begin(), args.end(), args.begin(), ::tolower);

    ROS_INFO_STREAM("ServiceBot::commandCallback cmd="
                    << cmd << " args=" << args);

    if (cmd.find("your name") != string::npos) {
        action.execute(Action::ACT_NAME);
    } else if (cmd.find("what time") != string::npos) {
        action.execute(Action::ACT_TIME);
    } else if (cmd.find("play music") != string::npos) {
        action.execute(Action::ACT_PLAYMUSIC, args);
    } else if (cmd.find("stop music") != string::npos) {
        action.execute(Action::ACT_STOPMUSIC);
    } else if ((cmd.find("move to") != string::npos) ||
               (cmd.find("go to") != string::npos) ||
               (cmd.find("room a") != string::npos) ||
               (cmd.find("room b") != string::npos) ||
               (cmd.find("room c") != string::npos)) {
        if (args.empty()) {
            args = cmd;
        }
        action.execute(Action::ACT_MOVETO, args);
    } else if ((cmd.find("stop moving") != string::npos) ||
               (cmd.find("abort") != string::npos) ||
               (cmd.find("cancel") != string::npos)) {
        action.execute(Action::ACT_STOPMOVETO);
    } else if (cmd.find("come back") != string::npos) {
        action.execute(Action::ACT_COMEBACK);
    } else if (cmd.find("forward") != string::npos) {
        action.execute(Action::ACT_FORWARD);
    } else if (cmd.find("backward") != string::npos) {
        action.execute(Action::ACT_BACKWARD);
    } else if (cmd.find("turn left") != string::npos) {
        action.execute(Action::ACT_TURNLEFT);
    } else if (cmd.find("turn right") != string::npos) {
        action.execute(Action::ACT_TURNRIGHT);
    } else if (cmd.find("stop") != string::npos) {
        action.execute(Action::ACT_STOPMOVE);
    } else {
        ROS_INFO_STREAM("ServiceBot:: unknown action");
    }

    return;
}

