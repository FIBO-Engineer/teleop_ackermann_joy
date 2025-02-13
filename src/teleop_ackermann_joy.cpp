/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "teleop_ackermann_joy/teleop_ackermann_joy.h"

#include <map>
#include <string>


namespace teleop_ackermann_joy
{

  /**
   * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
   * parameters later without breaking ABI compatibility, for robots which link TeleopAckermannJoy
   * directly into base nodes.
   */
  struct TeleopAckermannJoy::Impl
  {
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);

    ros::Subscriber joy_sub;
    ros::Publisher ackermann_vel_pub;
  
    bool use_two_enable_button;
    int enable_button;
    int enable_button_complement;
    int enable_turbo_button;

    std::map<std::string, int> axis_speed_map;
    std::map< std::string, std::map<std::string, double> > scale_speed_map;

    std::map<std::string, int> axis_steering_angle_map;
    std::map< std::string, std::map<std::string, double> > scale_steering_angle_map;

    bool sent_disable_msg;
  };

  /**
   * Constructs TeleopAckermannJoy.
   * \param nh NodeHandle to use for setting up the publisher and subscriber.
   * \param nh_param NodeHandle to use for searching for configuration parameters.
   */
TeleopAckermannJoy::TeleopAckermannJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
  {
    pimpl_ = new Impl;

    pimpl_->ackermann_vel_pub = nh->advertise<ackermann_msgs::AckermannDrive>("ackermann_vel", 1, true);
    pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopAckermannJoy::Impl::joyCallback, pimpl_);

    nh_param->param<bool>("use_two_enable_button", pimpl_->use_two_enable_button, false);
    nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
    nh_param->param<int>("enable_button_complement", pimpl_->enable_button_complement, 1);
    nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

    if (nh_param->getParam("axis_speed", pimpl_->axis_speed_map))
    {
      nh_param->getParam("scale_speed", pimpl_->scale_speed_map["normal"]);
      nh_param->getParam("scale_speed_turbo", pimpl_->scale_speed_map["turbo"]);
    }
    else
    {
      nh_param->param<int>("axis_speed", pimpl_->axis_speed_map["speed"], 1);
      nh_param->param<double>("scale_speed", pimpl_->scale_speed_map["normal"]["speed"], 0.5);
      nh_param->param<double>("scale_speed_turbo", pimpl_->scale_speed_map["turbo"]["speed"], 1.0);
    }

    if (nh_param->getParam("axis_steering_angle", pimpl_->axis_steering_angle_map))
    {
      nh_param->getParam("scale_steering_angle", pimpl_->scale_steering_angle_map["normal"]);
      nh_param->getParam("scale_steering_angle_turbo", pimpl_->scale_steering_angle_map["turbo"]);
    }
    else
    {
      nh_param->param<int>("axis_steering_angle", pimpl_->axis_steering_angle_map["steering_angle"], 0);
      nh_param->param<double>("scale_steering_angle", pimpl_->scale_steering_angle_map["normal"]["steering_angle"], M_PI/2);
      nh_param->param<double>("scale_steering_angle_turbo",
                              pimpl_->scale_steering_angle_map["turbo"]["steering_angle"], pimpl_->scale_steering_angle_map["normal"]["steering_angle"]);
    }

    ROS_INFO_NAMED("TeleopAckermannJoy", "Teleop enable button %i.", pimpl_->enable_button);
    ROS_INFO_COND_NAMED(pimpl_->use_two_enable_button, "TeleopAckermannJoy",
                        "Complement enable on button %i.", pimpl_->enable_button_complement);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopAckermannJoy",
                        "Turbo on button %i.", pimpl_->enable_turbo_button);

    for (std::map<std::string, int>::iterator it = pimpl_->axis_speed_map.begin();
         it != pimpl_->axis_speed_map.end(); ++it)
    {
      ROS_INFO_NAMED("TeleopAckermannJoy", "Speed axis %s on %i at scale %f.",
                     it->first.c_str(), it->second, pimpl_->scale_speed_map["normal"][it->first]);
      ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopAckermannJoy",
                          "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_speed_map["turbo"][it->first]);
    }

    for (std::map<std::string, int>::iterator it = pimpl_->axis_steering_angle_map.begin();
         it != pimpl_->axis_steering_angle_map.end(); ++it)
    {
      ROS_INFO_NAMED("TeleopAckermannJoy", "Steering angle axis %s on %i at scale %f.",
                     it->first.c_str(), it->second, pimpl_->scale_steering_angle_map["normal"][it->first]);
      ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopAckermannJoy",
                          "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_steering_angle_map["turbo"][it->first]);
    }

    pimpl_->sent_disable_msg = false;
  }

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
                const std::map<std::string, double>& scale_map, const std::string& fieldname)
  {
    if (axis_map.find(fieldname) == axis_map.end() ||
        scale_map.find(fieldname) == scale_map.end() ||
        joy_msg->axes.size() <= axis_map.at(fieldname))
    {
      return 0.0;
    }

    return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
  }

void TeleopAckermannJoy::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                           const std::string& which_map)
  {
    ackermann_msgs::AckermannDrive ackermann_vel_msg;
    ackermann_vel_msg.speed = getVal(joy_msg, axis_speed_map, scale_speed_map[which_map], "speed");
    ackermann_vel_msg.steering_angle = getVal(joy_msg, axis_steering_angle_map, scale_steering_angle_map[which_map], "steering_angle");

    ackermann_vel_pub.publish(ackermann_vel_msg);

    sent_disable_msg = false;
  }

void TeleopAckermannJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    if (enable_turbo_button >= 0 &&
        joy_msg->buttons.size() > enable_turbo_button &&
        joy_msg->buttons[enable_turbo_button])
    {
      sendCmdVelMsg(joy_msg, "turbo");
    }
    else if (joy_msg->buttons.size() > enable_button &&
             joy_msg->buttons[enable_button])
    {
      sendCmdVelMsg(joy_msg, "normal");
    }
    else
    {
      // When enable button is released, immediately send a single no-motion command
      // in order to stop the robot.
      if (!sent_disable_msg)
      {
        // Initializes with zeros by default.
        ackermann_msgs::AckermannDrive ackermann_vel_msg;
        ackermann_vel_pub.publish(ackermann_vel_msg);
        sent_disable_msg = true;
      }
    }
  }

}  // namespace teleop_ackermann_joy
