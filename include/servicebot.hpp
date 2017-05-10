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

/** @file servicebot.hpp
 *  @brief Definition of class ServiceBot
 *
 *  This file contains definitions of class ServiceBot
 *
 *  @author Huei-Tzu Tsai
 *          Steven Gambino
 *  @date   04/27/2017
*/

#ifndef INCLUDE_SERVICEBOT_HPP_
#define INCLUDE_SERVICEBOT_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <servicebot/commandService.h>
#include "action.hpp"

/**
 *  @brief Class definition of ServiceBot class
*/
class ServiceBot {
 public:
     void initialize(ros::NodeHandle &);

 private:
     ros::NodeHandle nodeHandle;
     ros::Subscriber commandSub;
     ros::Publisher commandPub;
     ros::ServiceServer commandServer;
     Action action;
     void commandCallback(const std_msgs::String::ConstPtr&);
     bool commandService(servicebot::commandService::Request &,
                         servicebot::commandService::Response &);
};

#endif  // INCLUDE_SERVICEBOT_HPP_
