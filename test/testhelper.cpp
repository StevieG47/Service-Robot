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

/** @file testhelper.cpp
 *  @brief Implementation of TestHelper class
 *
 *  This file contains implementation of TestHelper class for unit tests
 *
 *  @author Huei Tzu Tsai \n
 *          Steven Gambino
 *  @date   05/05/2017
*/

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "testhelper.hpp"


void TestHelper::testCommandCallback(const std_msgs::String::ConstPtr& msg) {
    std::istringstream lineStream(msg->data.c_str());

    getline(lineStream, cmd, ',');
    getline(lineStream, args, ',');

    // convert command and args to lower case
    std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
    std::transform(args.begin(), args.end(), args.begin(), ::tolower);

    ROS_DEBUG_STREAM("TestHelper::testCommandCallback cmd=" << cmd << " args=" << args);
    return;

}


void TestHelper::testRobotSoundCallback(const sound_play::SoundRequest::ConstPtr& msg) {
    snd = msg->sound;
    sndCmd = msg->command;
    cmd = msg->arg;

    ROS_DEBUG_STREAM("sound = " <<  (int) snd);
    ROS_DEBUG_STREAM("command = " << (int) sndCmd);
    ROS_DEBUG_STREAM("arg = " << cmd);
    ROS_DEBUG_STREAM("arg2 = " << msg->arg2);

    return;
}



