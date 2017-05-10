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
 *  receive voice recognition outputs and publishes voice command to servicebot
 *  to execute the commands.  SoundControl also utilizes sound_play node to
 *  say or play sound.
 *
 *  @author Huei-Tzu Tsai \n
 *          Steven Gambino
 *  @date   04/25/2017
*/

#ifndef INCLUDE_SOUNDCONTROL_HPP_
#define INCLUDE_SOUNDCONTROL_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sound_play/sound_play.h>
#include <string>


#define DEMO_MUSIC     "demo/01.mp3"

/**
 *  @brief Class definition of SoundControl class
*/
class SoundControl {
 public:
     /**
      *   @brief  Constructor of SoundControl class
      *
      *   @param  none
      *   @return none
     */
     SoundControl() {}


     /**
      *   @brief  Deconstructor of SoundControl class
      *
      *   @param  none
      *   @return none
     */
     ~SoundControl() {}

     /**
      *   @brief  Initialize SoundControl class
      *
      *   @param  ros node handle
      *   @return none
     */
     void initialize(ros::NodeHandle &);


     /**
      *   @brief  Send a string to be said by the sound_node
      *
      *   @param  string to be said
      *   @return none
     */
     void say(std::string);


     /**
      *   @brief  Stop saying a string that was previously started by say()
      *
      *   @param  same string as in say()
      *   @return none
     */
     void stopSaying(std::string);


     /**
      *   @brief  Play a wav/mp3 file
      *
      *   @param  absolute file path of music to be played   
      *   @return none
     */
     void play(std::string);


     /**
      *   @brief  Play a wav/mp3 file from servicebot package
      *
      *   @param  relative file path of music in servicebot package
      *   @return none
     */
     void playWaveFromPkg(std::string);


     /**
      *   @brief  Stop all playback on sound_play node
      *
      *   @param  none  
      *   @return none
     */
     void stopAll(void);

 private:
     ros::Publisher commandPub;           ///< publisher to /servicebot/command
     ros::Subscriber recognitionSub;      ///< subscriber to /recognizer/output
     sound_play::SoundClient soundClient;   ///< SoundClient object to publish
                                            ///< sound commands on /robotsound


     /**
      *   @brief  Speech callback to receive voice commands from 
      *           /recognizer/output topic and then publish commands to
      *           /servicebot/command for execution
      *
      *   @param  voice recognition output in std_msgs::String
      *   @return none
     */
     void speechCallback(const std_msgs::String::ConstPtr&);
};

#endif  // INCLUDE_SOUNDCONTROL_HPP_
