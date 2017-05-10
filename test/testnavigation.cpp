/********************************************************************
 *   MIT License
 *  
 *   Copyright (c) 2017  Huei-Tzu Tsai, Steven Gambino
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

/** @file testnavigation.cpp
 *  @brief Implementation of unit test for Navigation class
 *
 *  This file contains implementation of unit test for Navigation class
 *
 *  @author Huei Tzu Tsai \n
 *          Steven Gambino
 *  @date   05/08/2017
*/

#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <gtest/gtest.h>
#include <boost/thread.hpp>
#include "testhelper.hpp"
#include "navigation.hpp"


/**
 *   @brief  spin thread to process callbacks
 *
 *   @param  none
 *   @return none
*/
void processThread(void) {
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}


/**
 *   @brief  Verify initialization in Navigation class
 *
 *   @param  none
 *   @return none
*/
TEST(TestNavigation, testInit) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Navigation navi;

    ros::Rate loop_rate(2);

    navi.initialize(n);

    // register to check number of publishers to /mobile_base/commands/velocity
    ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                    &TestHelper::testMBCmdVelocityCallback, &testItem);

    // register to check number of subscribers to /odom topic
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);

    loop_rate.sleep();

    EXPECT_EQ(1, sub.getNumPublishers());
    EXPECT_EQ(1, pub.getNumSubscribers());
}


/**
 *   @brief  Verify moveTo function in Navigation class
 *
 *   @param  none
 *   @return none
*/
TEST(TestNavigation, testMoveToFunc) {
    ros::NodeHandle n;
    TestHelper testItem;
    Navigation navi;
    std_msgs::String msg;

    ros::Rate loop_rate(2);

    ros::Subscriber subMBGoal = n.subscribe("/move_base/goal", 1000,
                    &TestHelper::testMoveBaseGoalCallback, &testItem);

    ros::Subscriber subMBCancel = n.subscribe("/move_base/cancel", 1000,
                    &TestHelper::testMoveBaseCancelCallback, &testItem);

    ROS_DEBUG_STREAM("subMBGoal: number of publisher = "
                     << subMBGoal.getNumPublishers());
    ROS_DEBUG_STREAM("subMBCancel: number of publisher = "
                     << subMBCancel.getNumPublishers());

    geometry_msgs::Pose expGoal;
    expGoal.position.x = -5;
    expGoal.position.y = -11;
    expGoal.position.z = 0.0;
    expGoal.orientation.x = 0.0;
    expGoal.orientation.y = 0.0;
    expGoal.orientation.z = 0.950;
    expGoal.orientation.w = 0.312;

    // test move to command
    navi.moveTo(expGoal);

    // allow callback to process
    loop_rate.sleep();

    geometry_msgs::Pose actGoal = testItem.pos;

    // confirm goal pose is expected
    EXPECT_EQ(true, std::memcmp(&expGoal, &actGoal, sizeof(expGoal)));
}


/**
 *   @brief  Verify cancelMove function in Navigation class
 *
 *   @param  none
 *   @return none
*/
TEST(TestNavigation, testCancelMoveFunc) {
    ros::NodeHandle n;
    TestHelper testItem;
    Navigation navi;
    std_msgs::String msg;

    ros::Rate loop_rate(2);

    ros::Subscriber subMBGoal = n.subscribe("/move_base/goal", 1000,
                        &TestHelper::testMoveBaseGoalCallback, &testItem);

    ros::Subscriber subMBCancel = n.subscribe("/move_base/cancel", 1000,
                        &TestHelper::testMoveBaseCancelCallback, &testItem);

    ROS_DEBUG_STREAM("subMBGoal: number of publisher = "
                     << subMBGoal.getNumPublishers());
    ROS_DEBUG_STREAM("subMBCancel: number of publisher = "
                     << subMBCancel.getNumPublishers());

    geometry_msgs::Pose expGoal;
    expGoal.position.x = -5;
    expGoal.position.y = -11;
    expGoal.position.z = 0.0;
    expGoal.orientation.x = 0.0;
    expGoal.orientation.y = 0.0;
    expGoal.orientation.z = 0.950;
    expGoal.orientation.w = 0.312;

    navi.moveTo(expGoal);

    // allow mpveTo to process
    loop_rate.sleep();

    // test cancel move function
    navi.cancelMove();

    // allow callback to process
    loop_rate.sleep();

    // confirm goal ID is cancelled
    EXPECT_STREQ(testItem.goalID.c_str(), testItem.cancelID.c_str());
}


/**
 *   @brief  Verify forward function in Navigation class
 *
 *   @param  none
 *   @return none
*/
TEST(TestNavigation, testForwardFunc) {
    ros::NodeHandle n;
    TestHelper testItem;
    Navigation navi;
    std_msgs::String msg;

    ros::Rate loop_rate(2);

    navi.initialize(n);

    ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                        &TestHelper::testMBCmdVelocityCallback, &testItem);

    ROS_DEBUG_STREAM("sub: number of publisher = " << sub.getNumPublishers());

    // test move forward
    navi.forward();

    // allow callback to process
    loop_rate.sleep();

    geometry_msgs::Twist expTwist;
    expTwist.linear.x = 0.1;
    expTwist.linear.y = 0.0;
    expTwist.linear.z = 0.0;
    expTwist.angular.x = 0.0;
    expTwist.angular.y = 0.0;
    expTwist.angular.z = 0.0;

    geometry_msgs::Twist actTwist = testItem.twist;

    // confirm velocity is expected
    EXPECT_EQ(true, std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));

    // test stop
    navi.stop();

    // allow callback to process
    loop_rate.sleep();

    expTwist.linear.x = 0.0;
    actTwist = testItem.twist;

    // confirm velocity is 0
    EXPECT_EQ(true, std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));
}



/**
 *   @brief  Verify backward function in Navigation class
 *
 *   @param  none
 *   @return none
*/
TEST(TestNavigation, testBackwardFunc) {
    ros::NodeHandle n;
    TestHelper testItem;
    Navigation navi;
    std_msgs::String msg;

    ros::Rate loop_rate(2);

    navi.initialize(n);

    ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                        &TestHelper::testMBCmdVelocityCallback, &testItem);

    ROS_DEBUG_STREAM("sub: number of publisher = " << sub.getNumPublishers());

    // test move backward
    navi.backward();

    // allow callback to process
    loop_rate.sleep();

    geometry_msgs::Twist expTwist;
    expTwist.linear.x = -0.1;
    expTwist.linear.y = 0.0;
    expTwist.linear.z = 0.0;
    expTwist.angular.x = 0.0;
    expTwist.angular.y = 0.0;
    expTwist.angular.z = 0.0;

    geometry_msgs::Twist actTwist = testItem.twist;

    // confirm velocity is expected
    EXPECT_EQ(true, std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));

    // test stop
    navi.stop();

    // allow callback to process
    loop_rate.sleep();

    expTwist.linear.x = 0.0;
    actTwist = testItem.twist;

    // confirm velocity is 0
    EXPECT_EQ(true, std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));
}


/**
 *   @brief  Verify turn left function in Navigation class
 *
 *   @param  none
 *   @return none
*/
TEST(TestNavigation, testTurnLeftFunc) {
    ros::NodeHandle n;
    TestHelper testItem;
    Navigation navi;
    std_msgs::String msg;

    ros::Rate loop_rate(1);

    navi.initialize(n);

    ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                        &TestHelper::testMBCmdVelocityCallback, &testItem);

    ROS_DEBUG_STREAM("sub: number of publisher = " << sub.getNumPublishers());

    // test turn left
    navi.turnLeft();

    // allow callback to process
    loop_rate.sleep();

    geometry_msgs::Twist expTwist;
    expTwist.linear.x = 0.0;
    expTwist.linear.y = 0.0;
    expTwist.linear.z = 0.0;
    expTwist.angular.x = 0.0;
    expTwist.angular.y = 0.0;
    expTwist.angular.z = 0.1;

    geometry_msgs::Twist actTwist = testItem.twist;

    // confirm velocity is expected
    EXPECT_EQ(true, std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));

    // test stop
    navi.stop();

    // allow callback to process
    loop_rate.sleep();

    expTwist.angular.z = 0.0;
    actTwist = testItem.twist;

    // confirm velocity is 0
    EXPECT_EQ(true, std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));
}

/**
 *   @brief  Verify turn right function in Navigation class
 *
 *   @param  none
 *   @return none
*/
TEST(TestNavigation, testTurnRightFunc) {
    ros::NodeHandle n;
    TestHelper testItem;
    Navigation navi;
    std_msgs::String msg;

    ros::Rate loop_rate(1);

    navi.initialize(n);

    ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                        &TestHelper::testMBCmdVelocityCallback, &testItem);

    ROS_DEBUG_STREAM("sub: number of publisher = " << sub.getNumPublishers());

    // test turn right
    navi.turnRight();

    // allow callback to process
    loop_rate.sleep();

    geometry_msgs::Twist expTwist;
    expTwist.linear.x = 0.0;
    expTwist.linear.y = 0.0;
    expTwist.linear.z = 0.0;
    expTwist.angular.x = 0.0;
    expTwist.angular.y = 0.0;
    expTwist.angular.z = -0.1;

    geometry_msgs::Twist actTwist = testItem.twist;

    // confirm velocity is expected
    EXPECT_EQ(true, std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));

    // test stop
    navi.stop();

    // allow callback to process
    loop_rate.sleep();

    expTwist.angular.z = 0.0;
    actTwist = testItem.twist;

    // confirm velocity is 0
    EXPECT_EQ(true, std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));
}


/*
 *   @brief  unit test entrypoint
 *  
 *   @param  number of arguments
 *   @param  argument character array
 *   @return integer 0 upon exit success
*/
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "testnavigation");
    ros::NodeHandle nh;

    // spawn another thread
    boost::thread th(processThread);

    int ret = RUN_ALL_TESTS();

    ros::shutdown();

    // wait the second thread to finish
    th.join();

    return ret;
}
