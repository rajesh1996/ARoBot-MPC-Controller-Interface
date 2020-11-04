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
 * @file PIDController.hpp
 * @brief Class declaration for a PID controller
 *
 * @author Rajeshwar N S
 * @author Arjun Srinivasan
 */

#ifndef INCLUDE_PIDCONTROLLER_HPP_
#define INCLUDE_PIDCONTROLLER_HPP_

#include<iostream>
#include<vector>
#include<utility>

/**
 * @class PIDController
 * @brief Class structure with public and private members
 *
 */

namespace control {
class PIDController {
 private:
  double kp = 0; /*!< Proportional coefficient*/
  double ki = 0; /*!< Integral coefficient*/
  double kd = 0; /*!< Derivative coefficient*/
  double prev_error = 0;/*!< keep track of previous error*/
  double prev_headerror = 0;/*!< keep track of previous error*/
  double sum_error = 0;/*!< keep track of cumulative error*/
  double sum_headerror = 0;/*!< keep track of cumulative error*/
  std::vector<std::pair<double, double>>\
velpoints;/*!< stores velocity wrt time*/

  std::vector<std::pair<double, double>> headpoints;/*!< stores head wrt time*/
  double f = 100.0;/*!< controller frequency*/
  double current_error = 100;/*!< current error*/
  double current_headerror = 100;/*!< current headerror*/
  double currenthead = 0;/*!< current head*/
  double ack_steer = 0;/*!< current steer*/

 public:
  /**
   * @brief Class PIDController
   * The following class PIDController aids in calculation of the newVelocity and steering angle
   * using desired Velocity and heading. It implements functions to tune
   * controller and get the error response
   */

  /**
   * @brief Paramterized Constructor
   * @param kp 
   * @param ki
   * @param kd
   */

  PIDController();
  /**
   * @fn id setKp_(double)
   * @brief Setter for Derivative Gain
   *
   * @param kp
   */

  void setKp_(double kp);
  /**
   * @fn void setKi_(double)
   * @brief Setter for Proportional Gain
   *
   * @param ki
   */

  void setKi_(double ki);

  /**
   * @fn void setKd_(double)
   * @brief Setter for Interal Gain
   *
   * @param kd
   */

  void setKd_(double kd);

  /**
   * @fn double getKp()
   * @brief Get Proportional Gain
   *
   * @return kp
   */

  double getKp();

  /**
   * @fn double getKi()
   * @brief Get Integral Gain
   *
   * @return ki
   */

  double getKi();

  /**
   * @fn double getKd()
   * @brief Get Derivative Gain
   *
   * @return kd
   */

  double getKd();

  /**
   * @fn double calculateError(double, double)
   * @brief
   *
   * @param current
   * @param setpoint
   * @return Computed Error
   */

  double calculateError(double current, double setpoint);

  /**
   *  @brief Function to converge  current value and set point velocity/heading
   *  @param double  Target heading of the robot
   *  @param double currentvel is current velocity of robot
   *  @param double setPoint is the desired velocity
   *  @return double convergedheading
   */
  double convergeParams(double currentvel, double setvel, \
    double currenthead, double sethead);

  /**
   *  @brief Function to use GNUplot for velocity convergence graph
   *  @param bool flag
   *  @return double
   */
  double plotVelocity(bool a);
  /**
   *  @brief Function to use GNUplot for Heading convergence graph
   *  @param bool flag
   *  @return double
   */

  double plotHeading(bool a);


  double calculateheadError(double desired_head, double actual_head);



  /**
   * @brief Destructor of Class PIDController
   */
  ~PIDController();
};
}  // namespace control

#endif  // INCLUDE_PIDCONTROLLER_HPP_
