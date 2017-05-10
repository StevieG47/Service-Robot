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
 *  This file contains definitions of class ServiceBot which receives voice 
 *  commands or commands from console and executes the commands accordingly.
 *
 *  @author Huei-Tzu Tsai \n
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
     /**
      *   @brief  Constructor of ServiceBot class
      *
      *   @param  none
      *   @return none
     */
     ServiceBot() {}


     /**
      *   @brief  Deconstructor of ServiceBot class
      *
      *   @param  none
      *   @return none
     */
     ~ServiceBot() {}

     /**
      *   @brief  Initialize ServiceBot to subscribe/publish topics for command
      *           processing and setup a service to receive commands from console
      *
      *   @param  ros node handle
      *   @return none
     */
     void initialize(ros::NodeHandle &);

 private:
     ros::NodeHandle nodeHandle;         ///< ros node handle
     ros::Subscriber commandSub;         ///< subscriber to /servicebot/command
     ros::Publisher commandPub;          ///< publisher to /servicebot/command
     ros::ServiceServer commandServer;   ///< server of /commandService
     Action action;                      ///< action object to execute commands


     /**
      *   @brief  Command callback to process commands received from voice
      *           recognition output or console input
      *
      *   @param  commands in string
      *   @return none
     */
     void commandCallback(const std_msgs::String::ConstPtr&);


     /**
      *   @brief  Command service callback to receive commands from console
      *           and send the command to /servicebot/command for execution
      *
      *   @param  client request command in string
      *   @param  response to client in string
      *   @return none
     */
     bool commandService(servicebot::commandService::Request &,
                         servicebot::commandService::Response &);
};

#endif  // INCLUDE_SERVICEBOT_HPP_
