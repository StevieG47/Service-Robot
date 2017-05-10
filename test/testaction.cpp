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

/** @file testaction.cpp
 *  @brief Implementation of unit test for Action class
 *
 *  This file contains implementation of unit test for Action class
 *
 *  @author Huei Tzu Tsai \n
 *          Steven Gambino
 *  @date   05/08/2017
*/

#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <gtest/gtest.h>
#include <sound_play/SoundRequest.h>
#include <boost/thread.hpp>
#include <sstream>
#include "testhelper.hpp"
#include "action.hpp"


/**
 *   @brief  spin thread to process callbacks
 *
 *   @param  continue flag in boolean \n
 *           true to continue, false to terminate thread
 *   @return none
*/
void processThread(bool *cont) {
    ros::Rate loop_rate(10);

    while (ros::ok() && *cont) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}


/**
 *   @brief  Verify initialization in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testInit) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    act.initialize(n);

    // register to check number of publishers to /servicebot/command topic
    ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                                      &TestHelper::testMBCmdVelocityCallback, &testItem);

    // register to check number of subscribers to /recognizer/output topic
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);

    loop_rate.sleep();

    EXPECT_EQ(1, sub.getNumPublishers());
    EXPECT_EQ(1, pub.getNumSubscribers());
}


/**
 *   @brief  Verify name action in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testNameAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    // Subscribe topic /robotsound to verify commands sent from Action class
    ros::Subscriber sub = n.subscribe("/robotsound", 1000,
                                      &TestHelper::testRobotSoundCallback, &testItem);

    // test name action
    act.execute(Action::ACT_NAME);

    // allow callback to process
    loop_rate.sleep();

    EXPECT_EQ(sound_play::SoundRequest::SAY, testItem.snd);
    EXPECT_EQ(sound_play::SoundRequest::PLAY_ONCE, testItem.sndCmd);
    EXPECT_STREQ("my name is servicebot", testItem.cmd.c_str());
}


/**
 *   @brief  Verify time action in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testTimeAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    // Subscribe topic /robotsound to verify commands sent from Action class
    ros::Subscriber sub = n.subscribe("/robotsound", 1000,
                                      &TestHelper::testRobotSoundCallback, &testItem);

    // test name action
    act.execute(Action::ACT_TIME);

    // allow callback to process
    loop_rate.sleep();

    EXPECT_EQ(sound_play::SoundRequest::SAY, testItem.snd);
    EXPECT_EQ(sound_play::SoundRequest::PLAY_ONCE, testItem.sndCmd);
    EXPECT_TRUE(testItem.cmd.find("time now is") != std::string::npos);
}


/**
 *   @brief  Verify play music action in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testPlayMusicAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    // Subscribe topic /robotsound to verify commands sent from Action class
    ros::Subscriber sub = n.subscribe("/robotsound", 1000,
                                      &TestHelper::testRobotSoundCallback, &testItem);

    // test play music
    act.execute(Action::ACT_PLAYMUSIC);

    // allow callback to process
    loop_rate.sleep();

    EXPECT_EQ(sound_play::SoundRequest::PLAY_FILE, testItem.snd);
    EXPECT_EQ(sound_play::SoundRequest::PLAY_ONCE, testItem.sndCmd);
    EXPECT_STREQ("demo/01.mp3", testItem.cmd.c_str());
}


/**
 *   @brief  Verify play music action with filepath arg in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testPlayMusicFileAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    // Subscribe topic /robotsound to verify commands sent from Action class
    ros::Subscriber sub = n.subscribe("/robotsound", 1000,
                                      &TestHelper::testRobotSoundCallback, &testItem);

    std::string path = ros::package::getPath("servicebot");

    std::stringstream musicPath;
    musicPath << path << "/demo/01.mp3";

    // test play music with music file path
    act.execute(Action::ACT_PLAYMUSIC, musicPath.str());

    // allow callback to process
    loop_rate.sleep();

    EXPECT_EQ(sound_play::SoundRequest::PLAY_FILE, testItem.snd);
    EXPECT_EQ(sound_play::SoundRequest::PLAY_ONCE, testItem.sndCmd);
    EXPECT_STREQ(musicPath.str().c_str(), testItem.cmd.c_str());
}


/**
 *   @brief  Verify stop music in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testStopPlayMusicAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    // Subscribe topic /robotsound to verify commands sent from Action class
    ros::Subscriber sub = n.subscribe("/robotsound", 1000,
                                      &TestHelper::testRobotSoundCallback, &testItem);

    std::string path = ros::package::getPath("servicebot");

    std::stringstream musicPath;
    musicPath << path << "/demo/01.mp3";

    // play music
    act.execute(Action::ACT_PLAYMUSIC, musicPath.str());

    // allow callback to process
    loop_rate.sleep();

    // test stop play music
    act.execute(Action::ACT_STOPMUSIC);

    // allow callback to process
    loop_rate.sleep();

    EXPECT_EQ(sound_play::SoundRequest::PLAY_STOP, testItem.sndCmd);

}


/**
 *   @brief  Verify move to in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testMoveToAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    ros::Subscriber subMBGoal = n.subscribe("/move_base/goal", 1000,
                                      &TestHelper::testMoveBaseGoalCallback, &testItem);

    ros::Subscriber subMBCancel = n.subscribe("/move_base/cancel", 1000,
                                      &TestHelper::testMoveBaseCancelCallback, &testItem);

    act.execute(Action::ACT_MOVETO, "room a");

    loop_rate.sleep();

    geometry_msgs::Pose expGoal;
    expGoal.position.x = -5;
    expGoal.position.y = -11;
    expGoal.position.z = 0.0;
    expGoal.orientation.x = 0.0;
    expGoal.orientation.y = 0.0;
    expGoal.orientation.z = 0.950;
    expGoal.orientation.w = 0.312;

    geometry_msgs::Pose actGoal = testItem.pos;

    EXPECT_TRUE(0 == std::memcmp(&expGoal, &actGoal, sizeof(expGoal)));

    act.execute(Action::ACT_MOVETO, "room b");

    loop_rate.sleep();

    expGoal.position.x = 7.1;
    expGoal.position.y = -10.7;
    expGoal.position.z = 0.0;
    expGoal.orientation.x = 0.0;
    expGoal.orientation.y = 0.0;
    expGoal.orientation.z = -0.713;
    expGoal.orientation.w = 0.702;

    actGoal = testItem.pos;

    EXPECT_TRUE(0 == std::memcmp(&expGoal, &actGoal, sizeof(expGoal)));

    act.execute(Action::ACT_MOVETO, "room c");

    loop_rate.sleep();

    expGoal.position.x = 0;
    expGoal.position.y = 0;
    expGoal.position.z = 0;
    expGoal.orientation.x = 0.0;
    expGoal.orientation.y = 0.0;
    expGoal.orientation.z = -0.713;
    expGoal.orientation.w = 0.702;

    actGoal = testItem.pos;

    EXPECT_TRUE(0 == std::memcmp(&expGoal, &actGoal, sizeof(expGoal)));
}


/**
 *   @brief  Verify stop moving to in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testStopMovingToAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    ros::Subscriber subMBGoal = n.subscribe("/move_base/goal", 1000,
                                      &TestHelper::testMoveBaseGoalCallback, &testItem);

    ros::Subscriber subMBCancel = n.subscribe("/move_base/cancel", 1000,
                                      &TestHelper::testMoveBaseCancelCallback, &testItem);

    // move to room a
    act.execute(Action::ACT_MOVETO, "room a");

    loop_rate.sleep();

    // test stop moving to 
    act.execute(Action::ACT_STOPMOVETO);

    loop_rate.sleep();

    EXPECT_STREQ(testItem.goalID.c_str(), testItem.cancelID.c_str());
}


/**
 *   @brief  Verify come back in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testComeBackAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    ros::Subscriber subMBGoal = n.subscribe("/move_base/goal", 1000,
                                      &TestHelper::testMoveBaseGoalCallback, &testItem);

    ros::Subscriber subMBCancel = n.subscribe("/move_base/cancel", 1000,
                                      &TestHelper::testMoveBaseCancelCallback, &testItem);

    act.execute(Action::ACT_COMEBACK);

    loop_rate.sleep();

    geometry_msgs::Pose expGoal;
    expGoal.position.x = 0;
    expGoal.position.y = 0;
    expGoal.position.z = 0.0;
    expGoal.orientation.x = 0.0;
    expGoal.orientation.y = 0.0;
    expGoal.orientation.z = 0.0;
    expGoal.orientation.w = 1.0;

    geometry_msgs::Pose actGoal = testItem.pos;

    EXPECT_TRUE(0 == std::memcmp(&expGoal, &actGoal, sizeof(expGoal)));
}


/**
 *   @brief  Verify forward in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testForwardAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    act.initialize(n);

    ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                                      &TestHelper::testMBCmdVelocityCallback, &testItem);

    act.execute(Action::ACT_FORWARD);

    loop_rate.sleep();

    geometry_msgs::Twist expTwist;
    expTwist.linear.x = 0.1;
    expTwist.linear.y = 0.0;
    expTwist.linear.z = 0.0;
    expTwist.angular.x = 0.0;
    expTwist.angular.y = 0.0;
    expTwist.angular.z = 0.0;

    geometry_msgs::Twist actTwist = testItem.twist;

    EXPECT_TRUE(0 == std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));
}


/**
 *   @brief  Verify backward in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testBackwardAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    act.initialize(n);

    ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                                      &TestHelper::testMBCmdVelocityCallback, &testItem);

    act.execute(Action::ACT_BACKWARD);

    loop_rate.sleep();

    geometry_msgs::Twist expTwist;
    expTwist.linear.x = -0.1;
    expTwist.linear.y = 0.0;
    expTwist.linear.z = 0.0;
    expTwist.angular.x = 0.0;
    expTwist.angular.y = 0.0;
    expTwist.angular.z = 0.0;

    geometry_msgs::Twist actTwist = testItem.twist;

    EXPECT_TRUE(0 == std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));
}


/**
 *   @brief  Verify turn left in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testTurnLeftAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    act.initialize(n);

    ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                                      &TestHelper::testMBCmdVelocityCallback, &testItem);

    act.execute(Action::ACT_TURNLEFT);

    loop_rate.sleep();

    geometry_msgs::Twist expTwist;
    expTwist.linear.x = 0.0;
    expTwist.linear.y = 0.0;
    expTwist.linear.z = 0.0;
    expTwist.angular.x = 0.0;
    expTwist.angular.y = 0.0;
    expTwist.angular.z = 0.1;

    geometry_msgs::Twist actTwist = testItem.twist;

    EXPECT_TRUE(0 == std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));
}


/**
 *   @brief  Verify turn right in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testTurnRightAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    act.initialize(n);

    ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                                      &TestHelper::testMBCmdVelocityCallback, &testItem);

    act.execute(Action::ACT_TURNRIGHT);

    loop_rate.sleep();

    geometry_msgs::Twist expTwist;
    expTwist.linear.x = 0.0;
    expTwist.linear.y = 0.0;
    expTwist.linear.z = 0.0;
    expTwist.angular.x = 0.0;
    expTwist.angular.y = 0.0;
    expTwist.angular.z = -0.1;

    geometry_msgs::Twist actTwist = testItem.twist;

    EXPECT_TRUE(0 == std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));
}


/**
 *   @brief  Verify stop move in Action class
 *
 *   @param  none
 *   @return none
*/
TEST(TestAction, testStopMoveAction) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    Action act;

    ros::Rate loop_rate(2);

    act.initialize(n);

    ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                                      &TestHelper::testMBCmdVelocityCallback, &testItem);

    act.execute(Action::ACT_TURNRIGHT);

    loop_rate.sleep();

    act.execute(Action::ACT_STOPMOVE);

    loop_rate.sleep();

    geometry_msgs::Twist expTwist;
    expTwist.linear.x = 0.0;
    expTwist.linear.y = 0.0;
    expTwist.linear.z = 0.0;
    expTwist.angular.x = 0.0;
    expTwist.angular.y = 0.0;
    expTwist.angular.z = 0.0;

    geometry_msgs::Twist actTwist = testItem.twist;

    EXPECT_TRUE(0 == std::memcmp(&expTwist, &actTwist, sizeof(expTwist)));
}


int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "testaction");
    ros::NodeHandle nh;

    bool cont = true;

    // spawn another thread
    boost::thread th(processThread, &cont);

    int ret = RUN_ALL_TESTS();

    cont = false;

    // wait the second thread to finish
    th.join();

    return ret;
}
