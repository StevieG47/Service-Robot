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

/** @file action.cpp
 *  @brief Implementation of class Action methods
 *
 *  This file contains implemenation of class Action
 *
 *  @author Huei Tzu Tsai
 *          Steven Gambino
 *  @date   04/27/2017
*/

#include <ctime>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "action.hpp"
#include "soundcontrol.hpp"


int Action::execute(ros::NodeHandle &n, int act, const std::string &args) {
    SoundControl soundCtl;
    time_t t = time(0);
    struct tm *now = localtime(&t);
    std::stringstream ss;

    action = act;

    ROS_INFO_STREAM("Action:: action = " << action);

    switch(action) {
        case ACT_NAME:
            soundCtl.say("my name is servicebot");
            break;

        case ACT_TIME:
            if (now->tm_hour < 12)
                ss << "time now is " << now->tm_hour << " " << now->tm_min << " AM";
            else           
                ss << "time now is " << (now->tm_hour - 12) << " " << now->tm_min << " PM";

            soundCtl.say(ss.str());
            break;

        case ACT_PLAYMUSIC:
            soundCtl.play("/home/viki/catkin_ws/src/servicebot/demo/01.mp3");
            //vc.play("http://musicmaterial.jpn.org/loop/jingle_logo_001.mp3");
            break;

        case ACT_STOPMUSIC:
            soundCtl.stopPlaying("/home/viki/catkin_ws/src/servicebot/demo/01.mp3");
            //vc.stopPlaying("http://musicmaterial.jpn.org/loop/jingle_logo_001.mp3");
            break;

        default:
            // unknown action
            break;
    }

    return 0;
}



