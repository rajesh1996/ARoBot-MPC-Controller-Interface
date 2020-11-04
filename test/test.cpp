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

/* @file test.cpp
 * @brief PID test
 * @authors Rajeshwar N S Arjun Srinivasan
 */
#include <gtest/gtest.h>

#include <iostream>

#include "../include/PIDController.hpp"

control::PIDController controller;

TEST(PIDTest, TEST_GET_SET_PID) {
  double kp = 0.5;
  double ki = 0.05;
  double kd = 0.001;

  controller.setKp_(kp);
  controller.setKi_(ki);
  controller.setKd_(kd);

  EXPECT_DOUBLE_EQ(kp, controller.getKp());
  EXPECT_DOUBLE_EQ(ki, controller.getKi());
  EXPECT_DOUBLE_EQ(kd, controller.getKd());
}

TEST(PIDCalculateTest, TEST_CALCULATE_ERROR) {
EXPECT_DOUBLE_EQ(0.6005, controller.calculateError(0.0, 1.0));
}

TEST(PIDCalculateheadTest, TEST_CALCULATE_HEADERROR) {
EXPECT_DOUBLE_EQ(-0.6005, controller.calculateheadError(1.0, 2.0));
}

TEST(PIDConvergeTest, TEST_CALCULATE_ERROR) {
EXPECT_NEAR(1, controller.convergeParams(0, 2.0, 0, 2), 2);}


TEST(PIDplotheadTest, TEST_CALCULATE_PLOT) {
ASSERT_EQ(2, controller.plotHeading(false));}


TEST(PIDplotvelTest, TEST_CALCULATE_PLOT) {
ASSERT_EQ(2, controller.plotVelocity(false));
}

