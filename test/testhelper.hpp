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

/** @file testhelper.hpp
 *  @brief Definition of TestHelper class
 *
 *  This file contains definitions of class TestHelper
 *
 *  @author Huei-Tzu Tsai \n
 *          Steven Gambino
 *  @date   05/06/2017
*/

#ifndef INCLUDE_TESTHELPER_HPP_
#define INCLUDE_TESTHELPER_HPP_

#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sound_play/sound_play.h>


/**
 *  @brief Class definition of TestHelper class
*/
class TestHelper {
 public:
     void testCommandCallback(const std_msgs::String::ConstPtr&);
     void testRobotSoundCallback(const sound_play::SoundRequest::ConstPtr&);

     int8_t snd;
     int8_t sndCmd;

     std::string cmd;
     std::string args;

};

#endif  // INCLUDE_TESTHELPER_HPP_