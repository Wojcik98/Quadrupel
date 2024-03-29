/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Wim Meeussen
*********************************************************************/
#include <tf2_ros_own/buffer_server.h>
#include <tf2_ros_own/transform_listener.h>
#include <ros/ros.h>


class MyClass
{
public:
  MyClass() {}
  MyClass(double d) {}
};

class BlankClass
{
public:
  BlankClass() {}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_buffer");
  ros::NodeHandle nh;  

  double buffer_size;
  nh.param("buffer_size", buffer_size, 120.0);

  // WIM: this works fine:
  tf2_ros_own::Buffer buffer_core(tf2::durationFromSec(buffer_size+0)); // WTF??
  tf2_ros_own::TransformListener listener(buffer_core);
  tf2_ros_own::BufferServer buffer_server(buffer_core, "tf2_buffer_server", false);
  buffer_server.start();
  // But you should probably read this instead:
  // http://www.informit.com/guides/content.aspx?g=cplusplus&seqNum=439

  ros::spin();
}
