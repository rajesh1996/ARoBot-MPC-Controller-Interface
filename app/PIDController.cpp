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

/* @file PIDController.cpp
 * @brief PID controller for ackermann model
 * @authors Rajeshwar N S Arjun Srinivasan
 */

#include "../include/PIDController.hpp"

#include "../include/Ackermann.hpp"
#include <cmath>
#include "gnuplot-iostream.h"
#include<chrono>
#include<thread>

using namespace std::chrono_literals;
using std::chrono::steady_clock;
using std::chrono::duration;

control::PIDController::PIDController() {
  kp = 0.5;
  ki = 0.05;
  kd = 0.001;
  prev_error = 0;
  sum_error = 0;
  f = 100.0;
  current_error = 10000;
  max_velocity = 2;
  // currentvel = 0;
  currenthead = 0;
  ack_steer = 0;
  prev_headerror = 0;
  current_headerror = 0;
}


void control::PIDController::setKp_(double kp_) {
  kp = kp_;
}
void control::PIDController::setKi_(double ki_) {
  ki = ki_;
}
void control::PIDController::setKd_(double kd_) {
  kd = kd_;
}

double control::PIDController::getKp() {
  return kp;
}
double control::PIDController::getKi() {
  return ki;
}
double control::PIDController::getKd() {
  return kd;
}

double control::PIDController::calculateError(
  double desired_vel, double actual_vel) {
    current_error = desired_vel - actual_vel;
    double feedback;
    double differror;
    sum_error += current_error*(1/f);
    differror =  current_error - prev_error;
     feedback = kp * current_error + kd * differror*f + ki * sum_error;
    prev_error = current_error;
  return feedback;
}

double control::PIDController::calculateheadError(
  double desired_head, double actual_head) {
    current_headerror = desired_head - actual_head*(M_PI/180);
    double feedback;
    double differror;
    sum_headerror += current_headerror*(1/f);
    differror =  current_headerror - prev_headerror;

     feedback = kp * current_error + kd * differror*f + ki * sum_error;
    prev_headerror = current_headerror;

  return feedback;
}

double control::PIDController::convergeParams(double actvel, double setvel,
  double acthead, double sethead) {
  // initialize timing variables
int i = 1;
  std::chrono::milliseconds duration(static_cast<int>(1000/f));
  auto next_loop_time = steady_clock::now();
Ackermann ack;
double currentvel = actvel;
currenthead = acthead;

      double fbv = kp * (setvel-actvel);
      prev_error = setvel - actvel;


      currentvel +=fbv;
      double fbh = kp*(sethead-acthead*(M_PI/180));
            std::cout << "head error init  is " << fbh <<std::endl;
      prev_headerror = sethead - (acthead*(M_PI/180));

      ack_steer +=fbh;

      ack_steer = ack.updateSteer(ack_steer);
                  std::cout << "steerangle init is " << ack_steer <<std::endl;

         std::cout << "curr vel is " << currentvel <<std::endl;
                  std::cout << "set vel is " << setvel <<std::endl;
     velpoints.push_back(std::make_pair(i/f, currentvel));



                  // points.push_back(std::make_pair(
                  //                 (1/f, heading));

              std::cout << "curr head is " << currenthead <<std::endl;
              std::cout << "set head is " << sethead <<std::endl;
                            std::cout << "sabsis " << fabs(currenthead-sethead) <<std::endl;

while(abs(currenthead-sethead) > 0.1) {
  // while(i<30) {
             // std::cout << "inside lop " <<std::endl;

      next_loop_time += duration;
      currenthead = ack.updateHead(1/f, currentvel, ack_steer , currenthead);

        std::cout << "currenthead is " << currenthead <<std::endl;
        std::cout << "currentvel is " << currentvel <<std::endl;
        std::cout << "step count is " << i <<std::endl;
        i++;
             if (abs(currentvel-setvel) > 0.1) {
                fbv = calculateError(setvel, currentvel);
                currentvel +=fbv;
              }
                      fbh = calculateError(sethead, currenthead);
                     ack_steer+=fbh;
               std::this_thread::sleep_until(next_loop_time);
     velpoints.push_back(std::make_pair(i/f, currentvel));}
                         std::cout << "splos "  <<std::endl;

                  // plotVelocity();

            return currenthead;
}




control::PIDController::~PIDController() {
}


double control::PIDController::plotVelocity() {
//  call object and intialise variables for Graph in GNUPLOT
Gnuplot gnup;
//  set the graph on gnuplot for the respective coordinates
gnup << "set xrange [0:0.03]\nset yrange [0:45]\n";
gnup << "set title \"Velocity Convergence\"\n";
gnup << "set pointsize 1\n";
gnup << "set xlabel \"Time\"\n";
gnup << "set ylabel \"Current Velocity\"\n";
gnup << "set key outside\n";

gnup << "plot" << gnup.file1d(velpoints)
     << "with lp title 'Current Velocity' lc 3, "
     << 1 << " title 'Set Point' lt 1 lc 4" << std::endl;
     return 10.1;
}


double control::PIDController::plotHeading() {
//  call object and intialise variables for Graph in GNUPLOT
Gnuplot gnup;
//  set the graph on gnuplot for the respective coordinates
gnup << "set xrange [0:0.03]\nset yrange [0:45]\n";
gnup << "set title \"Velocity Convergence\"\n";
gnup << "set pointsize 1\n";
gnup << "set xlabel \"Time\"\n";
gnup << "set ylabel \"Current Velocity\"\n";
gnup << "set key outside\n";

gnup << "plot" << gnup.file1d(velpoints)
     << "with lp title 'Current Velocity' lc 3, "
     << 1 << " title 'Set Point' lt 1 lc 4" << std::endl;
     return 10.1;
}


