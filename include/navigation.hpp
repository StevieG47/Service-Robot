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

/** @file navigation.hpp
 *  @brief Definition of class navigation
 *
 *  This file contains definitions of class Navigation
 *
 *  @author Huei Tzu Tsai
 *          Steven Gambino
 *  @date   04/28/2017
*/

#ifndef INCLUDE_NAVIGATION_HPP_
#define INCLUDE_NAVIGATION_HPP_

#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 *  @brief Class definition of Navigation class
*/
class Navigation {
 public:
     enum dir {
         DIR_FORWARD = 0,
         DIR_BACKWARD,
         DIR_TURNRIGHT,
         DIR_TURNLEFT,
     };

     Navigation() : mbClient("move_base", true) {}

     void initialize(ros::NodeHandle &n);
     void moveTo(geometry_msgs::Pose &);
     void cancelMove(void);
     void forward(void);
     void backward(void);
     void turnLeft(void);
     void turnRight(void);
     void stop(void);
     void movebaseCallback(
        const actionlib::SimpleClientGoalState&,
        const move_base_msgs::MoveBaseResult::ConstPtr&);

 private:
     ros::Publisher movebaseCmdVelPub;
     ros::Timer timer;
     MoveBaseClient mbClient;
     int direction;

    void timerCallback(const ros::TimerEvent&);
};

#endif  // INCLUDE_NAVIGATION_HPP_
