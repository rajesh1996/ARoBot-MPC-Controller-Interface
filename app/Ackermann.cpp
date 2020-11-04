/**
 * BSD 3-Clause License
 *
 * @copyright (c) 2020, Rajeshwar N S, Arjun Srinivasan
 * 
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* @file Ackermann.cpp
 * @brief Ackerman vehichle model
 * @authors Rajeshwar N S Arjun Srinivasan
 */
#include<iostream>
#include <cmath>
 #include "../include/Ackermann.hpp"
Ackermann::Ackermann() {
left_steer = 0;/*!< leftwheel steering angle*/
right_steer = 0;/*!< rightwheel steering angle*/
steer = 10;
    wheel_radius = 0;/*!< wheel radius*/
    robot_head = 0;/*!< robot current heading*/
  robot_length = 8;/*!< robot current heading*/
    targethead = 0;/*!< robot target heading*/
robot_width = 4;
     max_steer = 40;
}

double Ackermann::updateSteer(double steer_angle) {
double R = robot_length/ std::tan((M_PI/180)*steer_angle);

int dir = 1;
//  To calculate the angles, follow this link
//  https://www.sciencedirect.com/topics/engineering/ackermann
left_steer = (90 - (180/M_PI)*std::atan((R +
               (robot_length * 0.5))/ robot_width) * dir);
right_steer = (90 - (180/M_PI)*std::atan((R -
               (robot_length * 0.5))/ robot_width) * dir);
                if (std::max(left_steer, right_steer) > 0.78) {
                       steer = max_steer;
                 } else {
            steer = steer_angle;}

return steer;
}

double Ackermann::updateHead(double time, double velocity,
double steer , double currHead) {
    currHead +=(velocity*time*(std::tan((M_PI/180)*steer)/robot_length));


return currHead;
}


Ackermann::~Ackermann() {}



