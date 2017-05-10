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
 *  @brief Definition of class Navigation
 *
 *  This file contains definitions of class Navigation which navigates robot to 
 *  pre-defined locations and controls robot's movements to go forward, backward,
 *  turn left or right, or stop
 *
 *  @author Huei Tzu Tsai \n
 *          Steven Gambino
 *  @date   04/28/2017
*/

#ifndef INCLUDE_NAVIGATION_HPP_
#define INCLUDE_NAVIGATION_HPP_

#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

/**
 *  @typedef MoveBaseClient
 *
 *  @brief Actionlib client of Mobilebase
*/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 *  @brief Class definition of Navigation class
*/
class Navigation {
 public:
    /**
     *  @enum Dir
     *
     *  @brief Enumeration of directions
    */
     enum dir {
         DIR_IDLE,              ///< idle
         DIR_FORWARD,           ///< move foward
         DIR_BACKWARD,          ///< move backward
         DIR_TURNLEFT,          ///< turn left
         DIR_TURNRIGHT,         ///< turn right
     };


     /**
      *   @brief  Constructor of Navigation class
      *
      *   @param  none
      *   @return none
     */
     Navigation() : mbClient("move_base", true) {}


     /**
      *   @brief  Deconstructor of Navigation class
      *
      *   @param  none
      *   @return none
     */
     ~Navigation() {}


     /**
      *   @brief  Initialize Navigation class
      *
      *   @param  ros node handle
      *   @return none
     */
     void initialize(ros::NodeHandle &n);


     /**
      *   @brief  Send goal to movebase to navigate robot to goal location
      *
      *   @param  goal location in geometry_msgs::Pose
      *   @return none
     */
     void moveTo(geometry_msgs::Pose &);


     /**
      *   @brief  Cancel movebase goal to abort navigation
      *
      *   @param  none
      *   @return none
     */
     void cancelMove(void);


     /**
      *   @brief  Start timer to publish velocity commands to move robot
      *           forward
      *
      *   @param  none
      *   @return none
     */
     void forward(void);


     /**
      *   @brief  Start timer to publish velocity commands to move robot
      *           backward
      *
      *   @param  none
      *   @return none
     */
     void backward(void);


     /**
      *   @brief  Start timer to publish velocity commands to make robot turn
      *           left by 90 degrees
      *
      *   @param  none
      *   @return none
     */
     void turnLeft(void);


     /**
      *   @brief  Start timer to publish velocity commands to make robot turn
      *           right by 90 degrees
      *
      *   @param  none
      *   @return none
     */
     void turnRight(void);


     /**
      *   @brief  Stop timer to stop sending velocity command
      *
      *   @param  none
      *   @return none
     */
     void stop(void);


 private:
     ros::Publisher movebaseCmdVelPub;   ///< publisher to publish on
                                         ///< command velocity topic
     ros::Subscriber odomSub;            ///< subscriber to get odom info
     ros::Timer timer;                   ///< timer to send velocity commands
     MoveBaseClient mbClient;            ///< movebase client to send 
                                         ///< navigation goal
     geometry_msgs::Pose curPose;        ///< current pose from odom
     int direction;                      ///< forward or backward direction
                                         ///< to move to
     int angle;                          ///< left or right angle to move to 
     double startAngle;                  ///< starting angle of robot in degree


     /**
      *   @brief  Callback function to receive odom info
      *
      *   @param  odometry info in nav_msgs::Odometry
      *   @return none
     */
     void odomCallback(const nav_msgs::Odometry::ConstPtr&);


     /**
      *   @brief  Callback function to receive navigation result
      *           from movebase
      *
      *   @param  goal state in actionlib::SimpleClientGoalState
      *   @param  result in move_base_msgs::MoveBaseResult::ConstPtr
      *   @return none
     */
     void movebaseCallback(
        const actionlib::SimpleClientGoalState&,
        const move_base_msgs::MoveBaseResult::ConstPtr&);


     /**
      *   @brief  Timer callback function to send velocity commands
      *
      *   @param  timer event passed by ros::Timer
      *   @return none
     */
     void timerCallback(const ros::TimerEvent&);


     /**
      *   @brief  Helper function to convert radians to degree
      *
      *   @param  yaw radian in double
      *   @return none
     */
     double convert2degree(double);
};

#endif  // INCLUDE_NAVIGATION_HPP_
