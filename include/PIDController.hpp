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

#ifndef INCLUDE_PIDCONTROLLER_HPP
#define INCLUDE_PIDCONTROLLER_HPP

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
  double max_velocity;/*!< Max vehicle velocity*/
  double max_accel;/*!< Max vehicle acceleration*/
  double ang_vel;/*!< Max vehicle angular velocity*/
  double ang_accel;/*!< Max vehicle ngular acceleration*/
  double kp; /*!< Proportional coefficient*/
  double ki; /*!< Integral coefficient*/
  double kd; /*!< Derivative coefficient*/
  double error;/*!< keep track of current error*/
  double prev_error;/*!< keep track of previous error*/
  double sum_error = 0;/*!< keep track of cumulative error*/
  double t_update = 0.01;/*!< step time interval*/
  std::vector<std::pair<double, double>>\
 velpoints;/*!< stores velocity wrt time*/
  std::vector<std::pair<double, double>> headpoints;/*!< stores head wrt time*/

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
   *  @brief Function to calculate new velocity in m/s with a PID
   *  Algorithm using kp, ki & kd
   *  @param double targetHeading, Target heading of the robot
   *  @param double currentVelocity, current velocity of robot
   *  @param double setPoint, Target Velocity
   *  @param int flag, to enable while
   *  @return double newVelocity
   */
  double convergeParams(double current, double setpoint);

  /**
   *  @brief Function to use GNUplot for velocity convergence graph
   *  @param std::vector<std::pair<double, double>>velpoints vector
   *  @return none
   */
  double plotVelocity();
  /**
   *  @brief Function to use GNUplot for Heading convergence graph
   *  @param std::vector<std::pair<double, double>> headpoints vector
   *  @return none
   */
  double plotHeading();

  /**
   * @brief Destructor of Class PIDController
   */
  ~PIDController();
};
}

#endif  // INCLUDE_PIDCONTROLLER_HPP
