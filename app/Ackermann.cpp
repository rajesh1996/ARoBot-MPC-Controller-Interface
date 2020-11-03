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
 #include "../include/Ackermann.hpp"

Ackermann::Ackermann() {
    double left_steer = 0;/*!< leftwheel steering angle*/
    double right_steer = 0;/*!< rightwheel steering angle*/
    double steer = 10;
    double wheel_radius = 0;/*!< wheel radius*/
    double robot_head = 0;/*!< robot current heading*/
    double robot_length = 0;/*!< robot current heading*/
    double targethead = 0;/*!< robot target heading*/
    double robot_width = 0;
    double max_steer = 40 ;


}

double Ackermann::updateSteer(double steer_angle) {

	double R = robot_length/ std::tan((M_PI/180)*steer_angle);

        // if ( targetHeading > 0 ) {
        //        dir = 1;
        // } else {
        //        dir = -1;
        // }


int dir = 1;
//  To calculate the angles, follow this link
//  https://www.sciencedirect.com/topics/engineering/ackermann
left_steer_ = (90 - (180/M_PI)*std::atan((R +
               (robot_length * 0.5))/ robot_width) * dir);
right_steer = (90 - (180/M_PI)*std::atan((R -
               (robot_length * 0.5))/ robot_width) * dir);
                 if(std::max(left_steer_, right_steer) > 45) {
                         
                         steer = max_steer;  
                 }
                 else
                 	steer = steer_angle;




return steer;

}

double Ackermann::updateHead(double time, double velocity,
double steer , double currHead) {
return 1;
}

/**
 *  @brief Function to calculate the length of arc in meters
 *  to be traced in order to head in target direction
 *  @param double diffAngle, difference in current and 
 *  target heading
 *  @param double corrRadius, corresponding radius
 *  @return double arclength
 */
// double SteerAlgorithm::arcLength(double diffAngle, double corrRadius) {
//         if ( diffAngle == 0 || diffAngle == 360 ) {
//              diffAngle = 0;
//         } else if (diffAngle == 90 || diffAngle == 180) {
//              diffAngle = diffAngle + 0;
//         } else if (diffAngle < 90) {
//              diffAngle = diffAngle + 270;
//         } else {
//         }
// return ((diffAngle/360)*(2*M_PI*corrRadius));
// }

/**
 *  @brief Function to calculate the angles in degrees for left and
 *  right wheels as per ackermann model, and then feed them
 *  to corresponding servos
 *  @param double corrRadius, corresponding radius
 *  @param double shaftLength, length between wheels
 *  @param double shafDistance, distance between rear and
 *  front shaft
 *  @return double maxWheelAngle
 */
// double SteerAlgorithm::changeWheelAngles(double corrRadius,
//                                          double shaftLength,
//                                          double shaftDistance) {
//         if ( targetHeading > 0 ) {
//                dir = 1;
//         } else {
//                dir = -1;
//         }

// //  To calculate the angles, follow this link
// //  https://www.sciencedirect.com/topics/engineering/ackermann
// lWheelAngle_ = (90 - (180/M_PI)*std::atan((corrRadius +
//                (shaftLength * 0.5))/ shaftDistance) * dir);
// rWheelAngle_ = (90 - (180/M_PI)*std::atan((corrRadius -
//                (shaftLength * 0.5))/ shaftDistance) * dir);
// return std::max(lWheelAngle_, rWheelAngle_);
// }



Ackermann::~Ackermann() {}



