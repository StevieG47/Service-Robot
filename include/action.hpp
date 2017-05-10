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

/** @file action.hpp
 *  @brief Definition of class Action
 *
 *  This file contains definitions of class Action which performs actions
 *  requested from voice commands or console input
 *
 *  @author Huei Tzu Tsai \n
 *          Steven Gambino
 *  @date   04/27/2017
*/

#ifndef INCLUDE_ACTION_HPP_
#define INCLUDE_ACTION_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include "soundcontrol.hpp"
#include "navigation.hpp"

/**
 *  @brief Class definition of Action class
*/
class Action {
 public:
    /**
     *  @enum Action
     *
     *  @brief Enumeration for actions
    */
     enum act {
         ACT_NAME = 0,       ///< say name
         ACT_TIME,           ///< say time
         ACT_PLAYMUSIC,      ///< play music
         ACT_STOPMUSIC,      ///< stop playing music
         ACT_MOVETO,         ///< move to location
         ACT_STOPMOVETO,     ///< stop moving to location
         ACT_COMEBACK,       ///< come back to initial pose
         ACT_FORWARD,        ///< move forward
         ACT_BACKWARD,       ///< move backward
         ACT_TURNLEFT,       ///< turn left
         ACT_TURNRIGHT,      ///< turn right
         ACT_STOPMOVE        ///< stop move
     };

    /**
     *  @struct
     *
     *  @brief Structure to hold location string its relative position
    */
     struct location {
         std::string loc;         ///< location string
         double pointX;           ///< position x
         double pointY;           ///< position y
         double pointZ;           ///< position z
         double orientationX;     ///< orientation x
         double orientationY;     ///< orientation y
         double orientationZ;     ///< orientation z
         double orientationW;     ///< orientation w

         /**
          *   @brief  Constructor of location structure
          *
          *   @param  location name in string
          *   @param  position x in double
          *   @param  position y in double
          *   @param  position z in double
          *   @param  orientation x in double
          *   @param  orientation y in double
          *   @param  orientation z in double
          *   @param  orientation w in double
          *   @return none
         */
         location(std::string s, double px, double py, double pz,
                  double ox, double oy, double oz, double ow) :
                  loc(s), pointX(px), pointY(py), pointZ(pz),
                  orientationX(ox), orientationY(oy), orientationZ(oz),
                  orientationW(ow) {}
     };

     /**
      *   @brief  Constructor of Action class
      *
      *   @param  none
      *   @return none
     */
     Action() : action(0) {}


     /**
      *   @brief  Deconstructor of Action class
      *
      *   @param  none
      *   @return none
     */
     ~Action() {}

     /**
      *   @brief  Initialization of Action class
      *
      *   @param  ros node handle
      *   @return none
     */
     void initialize(ros::NodeHandle &);


     /**
      *   @brief  Execute actions from voice/console commands
      *
      *   @param  action in int
      *   @param  arguments in string
      *   @return none
     */
     void execute(int, const std::string & args = "");


 private:
     int action;                           ///< action to perform
     ros::NodeHandle nodeHandle;           ///< ros node handle
     SoundControl soundCtl;                ///< soundcontrol object to say or
                                           ///< playback commands
     Navigation naviCtl;                   ///< navigation object to navigate
                                           ///< robot or control robot movement


     /**
      *   @brief  Perform navigation actions to move robot to specific 
      *           location or control robot to move forward, backward,
      *           turn left, turn right, or stop
      *
      *   @param  requested action in int
      *   @param  args to specify location in string for moveto command \n
      *           empty args otherwise
      *   @return none
     */
     void navigate(int, const std::string & args = "");


     /**
      *   @brief  Execute commands to stop or play music from file
      *
      *   @param  requested action in int
      *   @param  args to specify music file path in string for play command \n
      *           empty string otherwise
      *   @return none
     */
     void playMusic(int, const std::string & args = "");
};

#endif  // INCLUDE_ACTION_HPP_
