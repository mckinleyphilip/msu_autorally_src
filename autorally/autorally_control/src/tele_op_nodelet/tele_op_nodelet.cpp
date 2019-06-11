/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file tele_op_nodelet.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date April 14, 2014
 * @copyright 2014 Georgia Institute of Technology
 * @brief Controller to drive robot at constant speed
 *
 * @details ComstantSpeed Controller class implementation
 ***********************************************/

#include <math.h>
#include <pluginlib/class_list_macros.h>

#include "tele_op_nodelet.h"

#define PI 3.14159265
#define DEGTORAD (PI/180)

PLUGINLIB_DECLARE_CLASS(autorally_control, tele_op_nodelet, autorally_control::tele_op_nodelet, nodelet::Nodelet)

namespace autorally_control
{

tele_op_nodelet::tele_op_nodelet():
  m_controllerState(DISABLED),
  m_constantSpeedPrevThrot(0.0),
  m_controlEnabled(true),
  m_frontWheelsSpeed(0.0),
  m_backWheelsSpeed(0.0),
  m_integralError(0.0)
{}

tele_op_nodelet::~tele_op_nodelet()
{}

void tele_op_nodelet::onInit()
{
  NODELET_INFO("tele_op_nodelet initialization");
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nhPvt = getPrivateNodeHandle();

  loadThrottleCalibration();

  m_mostRecentSpeedCommand.data = 99;
  m_reverse = false;

  m_speedCommandSub = nh.subscribe("tele_op_nodelet/speedCommand", 1,
                          &tele_op_nodelet::speedCallback, this);
  m_wheelSpeedsSub = nh.subscribe("wheelSpeeds", 1,
                          &tele_op_nodelet::wheelSpeedsCallback,
                          this);
  m_chassisCommandPub = nh.advertise<autorally_msgs::chassisCommand>
                        ("tele_op_nodelet/chassisCommand", 1);

  if(!nhPvt.getParam("speedCommander", m_speedCommander) ||
     !nhPvt.getParam("accelerationRate", m_accelerationRate) ||
     !nhPvt.getParam("accelerationRate", m_accelerationRate) ||
     !nhPvt.getParam("KP", m_constantSpeedKP) ||
     !nhPvt.getParam("KD", m_constantSpeedKD) ||
     !nhPvt.getParam("KI", m_constantSpeedKI) ||
     !nhPvt.getParam("IMax", m_constantSpeedIMax))
  {
    NODELET_ERROR("Could not get all tele_op_nodelet params");
  }

  //m_accelerationProfile = generateAccelerationProfile(100);

  m_controlTimer = nh.createTimer(ros::Rate(1),
                      &tele_op_nodelet::controlCallback, this);
  /*m_controlEnableTimer = nh.createTimer(ros::Rate(0.2),
                      &tele_op_nodelet::enableControlCallback, this);*/
  NODELET_INFO("tele_op_nodelet initialization complete");

}

void tele_op_nodelet::speedCallback(const std_msgs::Float64ConstPtr& msg)
{
  if (m_mostRecentSpeedCommand.data != msg->data)
  {
    NODELET_INFO_STREAM("tele_op_nodelet: new speed setpoint:" << msg->data);
  }
  m_mostRecentSpeedCommand = *msg;
  
  // Set reverse flag based on the sign of the speed command
  if (msg->data < 0)
  {
	  m_reverse = true;
  }
  else
  {
	  m_reverse = false;
  }
  
}

void tele_op_nodelet::wheelSpeedsCallback(const autorally_msgs::wheelSpeedsConstPtr& msg)
{
  m_frontWheelsSpeed = 0.5*(msg->lfSpeed + msg->rfSpeed);
  //m_backWheelsSpeed = 0.2*m_backWheelsSpeed + 0.4*(msg->lbSpeed + msg->rbSpeed);

  autorally_msgs::chassisCommandPtr command(new autorally_msgs::chassisCommand);
  command->header.stamp = ros::Time::now();
  command->sender = "tele_op_nodelet";
  command->steering = -5.0;
  command->frontBrake = 0.0;
  command->reverse = m_reverse;
  
  double abs_goal_speed = abs(m_mostRecentSpeedCommand.data);
  double abs_front_wheel_speed = abs(m_frontWheelsSpeed);

  if (abs_goal_speed > 0.1 && abs_goal_speed < 99)
  {
    double p;
    if(m_throttleMappings.interpolateKey(abs_goal_speed, p))
    {
      m_integralError += abs_goal_speed - abs_front_wheel_speed;
      if (m_integralError > (m_constantSpeedIMax / m_constantSpeedKI))
      {
        m_integralError = (m_constantSpeedIMax / m_constantSpeedKI);
      }

      if (m_integralError < -(m_constantSpeedIMax / m_constantSpeedKI))
      {
        m_integralError = -(m_constantSpeedIMax / m_constantSpeedKI);
      }

      command->throttle = p +
                 m_constantSpeedKP*(abs_goal_speed - abs_front_wheel_speed);
      command->throttle += m_constantSpeedKI * m_integralError;
      command->throttle = std::max(0.0, std::min(1.0, command->throttle));

      // NODELET_INFO("interp %f, command %f",p, command->throttle);
      m_constantSpeedPrevThrot = p;
    }
    else
    {
      NODELET_WARN("tele_op_nodelet could not interpolate speed %f", m_mostRecentSpeedCommand.data);
      command->throttle = 0;
    }
  }
  else
  {
    command->throttle = 0;
    
  }
  
  //command->throttle = 1.0;
  //ROS_ERROR("Speed command: %f", command->throttle);
  if (abs_goal_speed != 99)
  {
    m_chassisCommandPub.publish(command);
  }
}

/*void tele_op_nodelet::enableControlCallback(const ros::TimerEvent& time)
{
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  if(!nhPvt.getParam("controlEnabled", m_controlEnabled) )
  {
    NODELET_ERROR("Could not update tele_op_nodelet params");
  }
}*/

void tele_op_nodelet::controlCallback(const ros::TimerEvent& time)
{
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  nhPvt.getParam("KP", m_constantSpeedKP);
  nhPvt.getParam("KD", m_constantSpeedKD);
  nhPvt.getParam("KI", m_constantSpeedKI);
  nhPvt.getParam("IMax", m_constantSpeedIMax);

}

void tele_op_nodelet::loadThrottleCalibration()
{
  NODELET_INFO("Loading calibration");
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  XmlRpc::XmlRpcValue v;
  nhPvt.param("throttleCalibration", v, v);
  std::map<std::string, XmlRpc::XmlRpcValue>::iterator mapIt;
  for(mapIt = v.begin(); mapIt != v.end(); mapIt++)
  {
    if(mapIt->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      std::pair<double, double> toAdd(std::pair<double, double>(
                                      boost::lexical_cast<double>(mapIt->first),
                                      static_cast<double>(mapIt->second)));
      NODELET_INFO_STREAM("tele_op_nodelet added to add mapping " <<
                             toAdd.first << ":" << toAdd.second);
      if(!m_throttleMappings.update(toAdd))
      {
        NODELET_ERROR_STREAM("tele_op_nodelet Failed to add mapping " <<
                             toAdd.first << ":" << toAdd.second);
      }
    } else
    {
      NODELET_ERROR("tele_op_nodelet: XmlRpc throttle calibration formatted incorrectly");
    }
  }
  NODELET_INFO_STREAM("tele_op_nodelet: Loaded " <<
                      m_throttleMappings.size() <<
                      " throttle mappings");
}

}
