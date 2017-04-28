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
 *  This file contains definitions of class Action
 *
 *  @author Huei Tzu Tsai
 *          Steven Gambino
 *  @date   04/27/2017
*/

#ifndef INCLUDE_ACTION_HPP_
#define INCLUDE_ACTION_HPP_

#include <ros/ros.h>
#include <string>


/**
 *  @brief Class definition of Action class
*/
class Action {
 public:
    enum act {
        ACT_NAME = 0,
        ACT_TIME,
        ACT_LOCATION,
        ACT_PLAYMUSIC,
        ACT_STOPMUSIC,
        ACT_MUTE,
        ACT_UNMUTE,
        ACT_MOVE = 0x20,
        ACT_STOPMOVE,
        ACT_COMEBACK
    };

    int getStatus(void) { return status; }
    int execute(ros::NodeHandle &, int, const std::string & args="");


 private:
    int action;
    int status;
};

#endif  // INCLUDE_ACTION_HPP_
