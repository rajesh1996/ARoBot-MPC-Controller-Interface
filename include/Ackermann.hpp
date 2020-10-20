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

/*
 * @file Ackermann.hpp
 * @brief Class declaration for a PID controller
 *
 * @author Rajeshwar N S
 * @author Arjun Srinivasan
 */
#ifndef INCLUDE_ACKERMANN_HPP
#define INCLUDE_ACKERMANN_HPP
#include <iostream>
#include "PIDController.hpp"

class Ackermann {
 private :

    double left_steer;/*!< leftwheel steering angle*/
    double right_steer;/*!< rightwheel steering angle*/
    double wheel_radius;/*!< wheel radius*/
    double robot_head;/*!< robot current heading*/
    double robot_length;/*!< robot current heading*/
    double targethead;/*!< robot target heading*/

 public :

    Ackermann();
    ~Ackermann();
    /**
    *  @brief Function to update induvidual steer angles given vehicle steering angle
    *  @param vehicle steering angle double
    *  @return max of left/right steer angle double
    */
    double updateSteer(double);
    /**
    *  @brief Function to update  heading of vehicle
    *  @param step time double
    *  @param step velocity double
    *  @param step steerangle double
    *  @return max of head angle
    */
    double updateHead(double, double, double);
};

#endif  // INCLUDE_ACKERMANN_HPP
