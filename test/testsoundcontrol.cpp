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

/** @file testsoundcontrol.cpp
 *  @brief Implementation of unit test for SoundControl class
 *
 *  This file contains implementation of unit test for SoundControl class
 *
 *  @author Huei Tzu Tsai \n
 *          Steven Gambino
 *  @date   05/05/2017
*/


#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sound_play/SoundRequest.h>
#include <gtest/gtest.h>
#include <sstream>
#include "testhelper.hpp"
#include "soundcontrol.hpp"


/**
 *   @brief  spin thread to process callbacks
 *
 *   @param  continue flag in boolean \n
 *           true to continue, false to terminate thread
 *   @return none
*/
void spinThread(bool *cont) {
    ros::Rate loop_rate(10);

    while (ros::ok() && *cont) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}


/**
 *   @brief  Verify initialize in SoundControl class
 *
 *   @param  none
 *   @return none
*/
TEST(TestSoundControl, testInit) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    SoundControl soundCtl;

    ros::Rate loop_rate(2);

    soundCtl.initialize(n);

    // register to check number of publishers to /servicebot/command topic
    ros::Subscriber sub = n.subscribe("/servicebot/command", 1000,
                                      &TestHelper::testCommandCallback, &testItem);

    // register to check number of subscribers to /recognizer/output topic
    ros::Publisher pub = n.advertise<std_msgs::String>("/recognizer/output", 1000);

    loop_rate.sleep();

    EXPECT_EQ(1, sub.getNumPublishers());
    EXPECT_EQ(1, pub.getNumSubscribers());
}


/**
 *   @brief  Verify speechCallback in SoundControl class
 *
 *   @param  none
 *   @return none
*/
TEST(TestSoundControl, testSpeechCallback) {
    ros::NodeHandle n;
    TestHelper testItem;
    std_msgs::String msg;
    SoundControl soundCtl;

    ros::Rate loop_rate(2);

    soundCtl.initialize(n);

    ros::Subscriber sub = n.subscribe("/servicebot/command", 1000,
                                      &TestHelper::testCommandCallback, &testItem);


    // register to publish topic on /recognizer/output to test speechCallback in 
    // SoundControl class
    ros::Publisher pub = n.advertise<std_msgs::String>("/recognizer/output", 1000);

    msg.data = "test";

    pub.publish(msg);

    loop_rate.sleep();

    // Expect response is test
    EXPECT_STREQ(msg.data.c_str(), testItem.cmd.c_str());
}


/**
 *   @brief  Verify say function in SoundControl class
 *
 *   @param  none
 *   @return none
*/
TEST(TestSoundControl, testSayFunc) {
    ros::NodeHandle n;
    TestHelper testItem;
    SoundControl soundCtl;
    std::string testPhrase = "test";

    ros::Rate loop_rate(2);

    // register to check say command published on /robotsound topic
    ros::Subscriber sub = n.subscribe("/robotsound", 1000,
                                      &TestHelper::testRobotSoundCallback, &testItem);

    soundCtl.say(testPhrase);

    loop_rate.sleep();

    EXPECT_EQ(sound_play::SoundRequest::SAY, testItem.snd);
    EXPECT_EQ(sound_play::SoundRequest::PLAY_ONCE, testItem.sndCmd);
    EXPECT_STREQ(testPhrase.c_str(), testItem.cmd.c_str());
}


/**
 *   @brief  Verify stopSaying function in SoundControl class
 *
 *   @param  none
 *   @return none
*/
TEST(TestSoundControl, testStopSayingFunc) {
    ros::NodeHandle n;
    TestHelper testItem;
    SoundControl soundCtl;
    std::string testPhrase = "test";

    ros::Rate loop_rate(2);

    // register to check stop saying command published on /robotsound topic
    ros::Subscriber sub = n.subscribe("/robotsound", 1000,
                                      &TestHelper::testRobotSoundCallback, &testItem);

    soundCtl.say(testPhrase);

    loop_rate.sleep();

    soundCtl.stopSaying(testPhrase);

    loop_rate.sleep();

    EXPECT_EQ(sound_play::SoundRequest::PLAY_STOP, testItem.sndCmd);
}


/**
 *   @brief  Verify play function in SoundControl class
 *
 *   @param  none
 *   @return none
*/
TEST(TestSoundControl, testPlayFunc) {
    ros::NodeHandle n;
    TestHelper testItem;
    SoundControl soundCtl;
    std::string demoMusic = DEMO_MUSIC;

    ros::Rate loop_rate(2);

    // register to check play command published on /robotsound topic
    ros::Subscriber sub = n.subscribe("/robotsound", 1000,
                                      &TestHelper::testRobotSoundCallback, &testItem);

    std::string path = ros::package::getPath("servicebot");

    std::stringstream musicPath;
    musicPath << path << "/demo/01.mp3";

    ROS_INFO_STREAM("demo music path = " << musicPath.str());

    soundCtl.play(musicPath.str());

    loop_rate.sleep();

    EXPECT_EQ(sound_play::SoundRequest::PLAY_FILE, testItem.snd);
    EXPECT_EQ(sound_play::SoundRequest::PLAY_ONCE, testItem.sndCmd);
    EXPECT_STREQ(musicPath.str().c_str(), testItem.cmd.c_str());
}


/**
 *   @brief  Verify playWaveFromPkg function in SoundControl class
 *
 *   @param  none
 *   @return none
*/
TEST(TestSoundControl, testPlayWaveFromPkgFunc) {
    ros::NodeHandle n;
    TestHelper testItem;
    SoundControl soundCtl;
    std::string demoMusic = DEMO_MUSIC;

    ros::Rate loop_rate(2);

    // register to check play from package command published on /robotsound topic
    ros::Subscriber sub = n.subscribe("/robotsound", 1000,
                                      &TestHelper::testRobotSoundCallback, &testItem);

    soundCtl.playWaveFromPkg(demoMusic);

    loop_rate.sleep();

    EXPECT_EQ(sound_play::SoundRequest::PLAY_FILE, testItem.snd);
    EXPECT_EQ(sound_play::SoundRequest::PLAY_ONCE, testItem.sndCmd);
    EXPECT_STREQ(demoMusic.c_str(), testItem.cmd.c_str());
}


/**
 *   @brief  Verify stopAll function in SoundControl class
 *
 *   @param  none
 *   @return none
*/
TEST(TestSoundControl, testStopAllFunc) {
    ros::NodeHandle n;
    TestHelper testItem;
    SoundControl soundCtl;

    ros::Rate loop_rate(2);

    // register to check stop all command published on /robotsound topic
    ros::Subscriber sub = n.subscribe("/robotsound", 1000,
                                      &TestHelper::testRobotSoundCallback, &testItem);

    soundCtl.stopAll();

    loop_rate.sleep();

    // Expect PLAY_STOP for stopping all playback
    EXPECT_EQ(sound_play::SoundRequest::PLAY_STOP, testItem.sndCmd);
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
    ros::init(argc, argv, "testsoundcontrol");
    ros::NodeHandle nh;

    bool cont = true;

    // spawn another thread
    boost::thread th(spinThread, &cont);

    int ret = RUN_ALL_TESTS();

    cont = false;

    // wait the second thread to finish
    th.join();

    return ret;
}