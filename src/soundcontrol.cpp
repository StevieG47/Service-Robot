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

/** @file soundcontrol.cpp
 *  @brief Implementation of class SoundControl methods
 *
 *  This file contains implemenation of SoundControl methods
 *
 *  @author Huei-Tzu Tsai
 *          Steven Gambino
 *  @date   04/27/2017
*/


#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sound_play/sound_play.h>
#include "soundcontrol.hpp"


void SoundControl::initialize(ros::NodeHandle &n) {
    ROS_INFO_STREAM("SoundControl::initialization");

    // Subscribe topic /recognizer/output to receive voice commands from user
    recognitionSub = n.subscribe<std_msgs::String>("/recognizer/output", 1000,
                                      &SoundControl::speechCallback, this);

    // Register to publish topic on /servicerobot/command to send voice commands to 
    // service robot
    commandPub = n.advertise<std_msgs::String>("/servicebot/command", 1000);
    return;
}


void SoundControl::speechCallback(const std_msgs::String::ConstPtr& msg) {

    // parse speech commands and publish on topic serviceCommand

    ROS_INFO_STREAM("SoundControl::speechCallback: publish " << msg->data.c_str());

    // send messages
    commandPub.publish(msg);

    return;
}


void SoundControl::say(std::string msg) {

    ROS_INFO_STREAM("SoundControl::say: " << msg);
    soundClient.say(msg);
    return;
}


void SoundControl::stopSaying(std::string msg) {

    ROS_INFO_STREAM("SoundControl::stopSaying: " << msg.c_str());
    soundClient.stopSaying(msg.c_str());
    return;
}


void SoundControl::play(std::string filename) {

    ROS_INFO_STREAM("SoundControl::play: " << filename.c_str());
    soundClient.playWave(filename);
    return;
}


void SoundControl::playWaveFromPkg(std::string filename) {

    ROS_INFO_STREAM("SoundControl::playWaveFromPkg: " << filename.c_str());
    soundClient.playWaveFromPkg("servicebot", filename);
    return;
}


void SoundControl::stopPlaying(std::string filename) {

    ROS_INFO_STREAM("SoundControl::stopPlaying: " << filename.c_str());
    soundClient.stopWave(filename);
    return;
}


void SoundControl::stopAll(void) {

    ROS_INFO_STREAM("SoundControl::stopAll");
    soundClient.stopAll();
    return;
}
