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

/**
 *  @file main.cpp
 *  @brief Initial file of servicebot node
 *
 *  This file contains main entry point of a service robot
 *
 *
 *
 *  @author Huei-Tzu Tsai
 *          Steven Gambino
 *  @date   04/27/2017
*/


#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "soundcontrol.hpp"

/*
 *   @brief  service robot entrypoint
 *  
 *   @param  number of arguments
 *   @param  argument character array
 *   @return integer 0 upon exit success
*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "servicerobot");

    ros::NodeHandle n;
    SoundControl soundCtl;

    soundCtl.initialize(n);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

