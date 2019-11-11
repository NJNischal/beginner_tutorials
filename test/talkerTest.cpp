/**
*
*Copyright (c) 2019 Nagireddi Jagadesh Nischal
*
*Redistribution and use in source and binary forms, with or without modification, are permitted *provided that the following conditions are met:
*
*1. Redistributions of source code must retain the above copyright notice, this list of conditions and *the following disclaimer.
*
*2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions *and the following disclaimer in the documentation and/or other materials provided with the *distribution.
*
*3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse *or promote products derived from this software without specific prior written permission.
*
*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR *IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND *FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR *CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, *DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER *IN *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT *OF THE *USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/


/*
 * @copyright 2019
 * @copyright BSD 3-Clause
 * @file talkerTest.cpp
 * @author Nagireddi Jagadesh Nischal
 * @date 11/10/2019
 * @brief gtest to test talker node
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/changeText.h"
#include "std_msgs/String.h"

/**
 * @brief  Test to check if the test exists
 * @param  testTalkerNode   gtest framework
 * @param  serviceExistanceTest   test name
 */
TEST(testTalkerNode, testServiceExistance) {
  
// Create the node handle
  
ros::NodeHandle nd;
  
// Registering the client to service

auto client = nd.serviceClient<beginner_tutorials::editString>("editString");

// Checking for service existance

EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));

}
