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

#ifndef TEST_TESTHELPER_HPP_
#define TEST_TESTHELPER_HPP_


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sound_play/sound_play.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include <string>

/**
 *  @brief Class definition of TestHelper class
*/
class TestHelper {
 public:
     /**
      *   @brief  Callback function to receive string messages
      *
      *   @param  string messages in std_msgs::String
      *   @return none
     */
     void testCommandCallback(const std_msgs::String::ConstPtr&);


     /**
      *   @brief  Callback function to receive SoundRequest messages
      *           to /robotsound
      *
      *   @param  sound request messages in sound_play::SoundRequest
      *   @return none
     */
     void testRobotSoundCallback(const sound_play::SoundRequest::ConstPtr&);


     /**
      *   @brief  Callback function to receive goal messages sent to movebase
      *
      *   @param  goal messages in move_base_msgs::MoveBaseActionGoal
      *   @return none
     */
     void testMoveBaseGoalCallback
         (const move_base_msgs::MoveBaseActionGoal::ConstPtr&);


     /**
      *   @brief  Callback function to receive cancel messages sent to movebase
      *
      *   @param  cancel messages in move_base_msgs::MoveBaseActionGoal
      *   @return none
     */
     void testMoveBaseCancelCallback(const actionlib_msgs::GoalID::ConstPtr&);


     /**
      *   @brief  Callback function to receive command velocity messages sent
      *           to mobilebase_command_velocity to move robot
      *
      *   @param  velocity command messages in geometry_msgs::Twist
      *   @return none
     */
     void testMBCmdVelocityCallback(const geometry_msgs::Twist::ConstPtr&);

     int8_t snd;                  ///< sound to play
     int8_t sndCmd;               ///< indicates what to do with the sound

     std::string cmd;             ///< command string
     std::string args;            ///< arguments string

     geometry_msgs::Pose pos;     ///< position
     std::string goalID;          ///< goal ID
     std::string cancelID;        ///< goal ID to be cancelled

     geometry_msgs::Twist twist;  ///< command velocity
};

#endif  // TEST_TESTHELPER_HPP_
