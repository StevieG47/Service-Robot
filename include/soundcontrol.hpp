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

/** @file soundcontrol.hpp
 *  @brief Definition of class SoundControl
 *
 *  This file contains definitions of class SoundControl which subscribes to
 *  /recognizer/output topic and parses voice command from end user
 *
 *  @author Huei-Tzu Tsai
 *          Steven Gambino
 *  @date   04/25/2017
*/

#ifndef INCLUDE_SOUNDCONTROL_HPP_
#define INCLUDE_SOUNDCONTROL_HPP_

#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sound_play/sound_play.h>

/**
 *  @brief Class definition of SoundControl class
*/
class SoundControl {
 public:
     void initialize(ros::NodeHandle &);
     void speechCallback(const std_msgs::String::ConstPtr&);
     void say(std::string);
     void stopSaying(std::string);
     void play(std::string);
     void stopPlaying(std::string);
     void stopAll(void);

 private:
     ros::Subscriber recognitionSub;
     sound_play::SoundClient soundClient;
};

#endif  // INCLUDE_SOUNDCONTROL_HPP_
