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

/** @file navigation.cpp
 *  @brief Implementation of class Navigation methods
 *
 *  This file contains implemenation of class Navigation
 *
 *  @author Huei Tzu Tsai
 *          Steven Gambino
 *  @date   04/28/2017
*/

#include <stdlib.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "navigation.hpp"


void Navigation::initialize(ros::NodeHandle &n) {
    ROS_INFO_STREAM("Navigation::initialize");

    // wait for the action server to come up
    ROS_INFO_STREAM("+++ wait for move_base action server +++");
    //mbClient.waitForServer();
    ROS_INFO_STREAM("--- wait for move_base action server ---");

    // Register to publish topic on /servicerobot/command to send voice commands to 
    // service robot
    movebaseCmdVelPub = n.advertise<geometry_msgs::Twist>
                         ("/mobile_base/commands/velocity", 1000);

    // Register for timer callback, set it to stop initially
    timer = n.createTimer(ros::Duration(1), &Navigation::timerCallback, this);
    timer.stop();
}


void Navigation::moveTo(geometry_msgs::Pose &goal) {
    move_base_msgs::MoveBaseGoal mbGoal;

    ROS_INFO_STREAM("moving to goal:");
    ROS_INFO_STREAM("position");
    ROS_INFO_STREAM("x" << goal.position.x);
    ROS_INFO_STREAM("y" << goal.position.y);
    ROS_INFO_STREAM("z" << goal.position.z);
    ROS_INFO_STREAM("orientation");
    ROS_INFO_STREAM("x" << goal.orientation.x);
    ROS_INFO_STREAM("y" << goal.orientation.y);
    ROS_INFO_STREAM("z" << goal.orientation.z);
    ROS_INFO_STREAM("w" << goal.orientation.w);


    mbGoal.target_pose.header.frame_id = "map";
    mbGoal.target_pose.header.stamp = ros::Time::now();

    mbGoal.target_pose.pose = goal;

    mbClient.sendGoal(mbGoal);

    // send a goal to the robot
    mbClient.sendGoal(mbGoal,
                boost::bind(&Navigation::movebaseCallback, this, _1, _2),
                MoveBaseClient::SimpleActiveCallback(),
                MoveBaseClient::SimpleFeedbackCallback());

    return;
}


void Navigation::cancelMove(void) {
    ROS_INFO_STREAM("cancel moving to goal");

    // send a goal to the robot
    mbClient.cancelGoal();

    return;
}


void Navigation::forward(void) {
    ROS_INFO_STREAM("forward");
    return;
}


void Navigation::backward(void) {
    ROS_INFO_STREAM("backward");
    return;
}


void Navigation::turnLeft(void) {
    ROS_INFO_STREAM("turn left");
    return;
}


void Navigation::turnRight(void) {
    ROS_INFO_STREAM("turn right");
    return;
}


void Navigation::stop(void) {
    ROS_INFO_STREAM("stop move");
    timer.stop();
    return;
}


void Navigation::movebaseCallback(
    const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResult::ConstPtr& result) {

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO_STREAM("Navigation:: Base succeeded to moved to goal");
    else
        ROS_INFO_STREAM("Navigation:: Base failed to move to goal");

}


void Navigation::timerCallback(const ros::TimerEvent& event) {
    geometry_msgs::Twist msg;

    msg.linear.x = 0.1;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    movebaseCmdVelPub.publish(msg);
}
