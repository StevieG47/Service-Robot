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
#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "action.hpp"
#include "soundcontrol.hpp"
#include "navigation.hpp"

using std::string;


void Action::initialize(ros::NodeHandle &n) {
    nodeHandle = n;

    naviCtl.initialize(nodeHandle);
}


int Action::execute(int act, const string &args) {
    time_t t = time(0);
    struct tm *now = localtime(&t);
    std::stringstream ss;

    action = act;

    ROS_INFO_STREAM("Action:: action = " << action << " args=" << args);

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
        case ACT_STOPMUSIC:
            playMusic(action, args);
            break;

        case ACT_MOVETO:
        case ACT_STOPMOVETO:
        case ACT_COMEBACK:
        case ACT_FORWARD:
        case ACT_BACKWARD:
        case ACT_TURNLEFT:
        case ACT_TURNRIGHT:
        case ACT_STOPMOVE:
            navigate(action, args);
            break;

        default:
            // unknown action
            break;
    }

    return 0;
}


int Action::navigate(int act, const string &args) {
    geometry_msgs::Pose goal;

    struct location locationA(string("location A"), -2.856, 0.881, 0.0, 0.0, 0.0, 0.950, 0.312);
    struct location locationB(string("location B"), -0.523, -3.691, 0.0, 0.0, 0.0, -0.713, 0.702);

    std::vector<location> locations;
    locations.push_back(locationA);
    locations.push_back(locationB);

    switch(action) {
        case ACT_MOVETO:
            for (int i = 0; i < locations.size(); ++i) {
                if (args.find(locations[i].loc) != string::npos) {
                    ROS_INFO_STREAM("Action:: navigate move to " << locations[i].loc);
                    goal.position.x = locations[i].pointX;
                    goal.position.y = locations[i].pointY;
                    goal.position.z = locations[i].pointZ;
                    goal.orientation.x = locations[i].orientationX;
                    goal.orientation.y = locations[i].orientationY;
                    goal.orientation.z = locations[i].orientationZ;
                    goal.orientation.w = locations[i].orientationW;
                    naviCtl.moveTo(goal);
                    break;
                }
            }
            break;

        case ACT_STOPMOVETO:
            naviCtl.cancelMove();
            break;

        case ACT_COMEBACK:
            // come back to initial pose (assuming point(0,0,0) quaternion(0,0,1))
            goal.position.x = 0;
            goal.position.y = 0;
            goal.position.z = 0;
            goal.orientation.x = 0;
            goal.orientation.y = 0;
            goal.orientation.z = 0;
            goal.orientation.w = 1.0;
            naviCtl.moveTo(goal);
            break;

        case ACT_FORWARD:
            naviCtl.forward();
            break;

        case ACT_BACKWARD:
            naviCtl.backward();
            break;

        case ACT_TURNLEFT:
            naviCtl.turnLeft();
            break;

        case ACT_TURNRIGHT:
            naviCtl.turnRight();
            break;

        case ACT_STOPMOVE:
            naviCtl.stop();
            break;

    }

    return 0;
}


int Action::playMusic(int act, const string &args) {
    string filename;

    switch(act) {
        case ACT_PLAYMUSIC:
            ROS_INFO_STREAM("Action::playMusic:: args size =" << args.size() << " empty=" << args.empty());
            if (args.empty()) {
                filename = string("/home/viki/catkin_ws/src/Service-Robot/demo/01.mp3");
            } else if ((args.find(".mp3") != string::npos) ||
                       (args.find(".wav") != string::npos) ||
                       (args.find(".ogg") != string::npos)) {
                // only can play music with extensions of mp3, wav, ogg
                filename = args;
            } else {
                // unknown file
                break;
            }

            ROS_INFO_STREAM("Action::playMusic:: filename=" << filename);
            soundCtl.play(filename);
            break;

        case ACT_STOPMUSIC:
            soundCtl.stopAll();
            break;

        default:
            break;
    }

    return 0;
}

