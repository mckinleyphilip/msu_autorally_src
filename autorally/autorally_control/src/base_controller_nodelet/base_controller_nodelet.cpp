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
 * @file base_controller_nodelet.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date April 14, 2014
 * @copyright 2014 Georgia Institute of Technology
 * @brief Controller to drive robot at constant speed
 *
 * @details ComstantSpeed Controller class implementation
 ***********************************************/

#include <math.h>
#include <pluginlib/class_list_macros.h>

#include "base_controller_nodelet.h"

#define PI 3.14159265
#define DEGTORAD (PI/180)

PLUGINLIB_DECLARE_CLASS(autorally_control, base_controller_nodelet, autorally_control::base_controller_nodelet, nodelet::Nodelet)

namespace autorally_control
{

base_controller_nodelet::base_controller_nodelet():
  m_controllerState(DISABLED),
  m_constantSpeedPrevThrot(0.0),
  m_controlEnabled(true),
  m_frontWheelsSpeed(0.0),
  m_backWheelsSpeed(0.0),
  m_integralError(0.0)
{}

base_controller_nodelet::~base_controller_nodelet()
{}

void base_controller_nodelet::onInit()
{
  NODELET_INFO("base_controller_nodelet initialization");
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nhPvt = getPrivateNodeHandle();

  loadThrottleCalibration();

  m_mostRecentSpeedCommand.data = -99;

  m_speedCommandSub = nh.subscribe("cmd_vel", 1,
                          &base_controller_nodelet::speedCallback, this);
  m_wheelSpeedsSub = nh.subscribe("wheelSpeeds", 1,
                          &base_controller_nodelet::wheelSpeedsCallback,
                          this);
  m_chassisCommandPub = nh.advertise<autorally_msgs::chassisCommand>
                        ("base_controller_nodelet/chassisCommand", 1);

  if(!nhPvt.getParam("speedCommander", m_speedCommander) ||
     !nhPvt.getParam("accelerationRate", m_accelerationRate) ||
     !nhPvt.getParam("accelerationRate", m_accelerationRate) ||
     !nhPvt.getParam("KP", m_constantSpeedKP) ||
     !nhPvt.getParam("KD", m_constantSpeedKD) ||
     !nhPvt.getParam("KI", m_constantSpeedKI) ||
     !nhPvt.getParam("IMax", m_constantSpeedIMax))
  {
    NODELET_ERROR("Could not get all base_controller_nodelet params");
  }

  //m_accelerationProfile = generateAccelerationProfile(100);

  m_controlTimer = nh.createTimer(ros::Rate(1),
                      &base_controller_nodelet::controlCallback, this);
  /*m_controlEnableTimer = nh.createTimer(ros::Rate(0.2),
                      &base_controller_nodelet::enableControlCallback, this);*/
  NODELET_INFO("base_controller_nodelet initialization complete");

}

void base_controller_nodelet::speedCallback(const geometry_msgs::TwistPtr& msg)
{
	NODELET_INFO("Test");
	/*
  if (m_mostRecentSpeedCommand.data != msg->data)
  {
    NODELET_INFO_STREAM("base_controller_nodelet: new speed setpoint:" << msg->data);
  }
  m_mostRecentSpeedCommand = *msg;
  * */
}

void base_controller_nodelet::wheelSpeedsCallback(const autorally_msgs::wheelSpeedsConstPtr& msg)
{
  m_frontWheelsSpeed = 0.5*(msg->lfSpeed + msg->rfSpeed);
  //m_backWheelsSpeed = 0.2*m_backWheelsSpeed + 0.4*(msg->lbSpeed + msg->rbSpeed);

  autorally_msgs::chassisCommandPtr command(new autorally_msgs::chassisCommand);
  command->header.stamp = ros::Time::now();
  command->sender = "base_controller_nodelet";
  command->steering = -5.0;
  command->frontBrake = 0.0;

  if (m_mostRecentSpeedCommand.data > 0.1)
  {
    double p;
    if(m_throttleMappings.interpolateKey(m_mostRecentSpeedCommand.data, p))
    {
      m_integralError += m_mostRecentSpeedCommand.data - m_frontWheelsSpeed;
      if (m_integralError > (m_constantSpeedIMax / m_constantSpeedKI))
      {
        m_integralError = (m_constantSpeedIMax / m_constantSpeedKI);
      }

      if (m_integralError < -(m_constantSpeedIMax / m_constantSpeedKI))
      {
        m_integralError = -(m_constantSpeedIMax / m_constantSpeedKI);
      }

      command->throttle = p +
                 m_constantSpeedKP*(m_mostRecentSpeedCommand.data - m_frontWheelsSpeed);
      command->throttle += m_constantSpeedKI * m_integralError;
      command->throttle = std::max(0.0, std::min(1.0, command->throttle));

      // NODELET_INFO("interp %f, command %f",p, command->throttle);
      m_constantSpeedPrevThrot = p;
    }
    else
    {
      NODELET_WARN("base_controller_nodelet could not interpolate speed %f", m_mostRecentSpeedCommand.data);
      command->throttle = 0;
    }
  }
  else
  {
    command->throttle = 0;
    
  }
  
  //command->throttle = 1.0;
  //ROS_ERROR("Speed command: %f", command->throttle);
  if (m_mostRecentSpeedCommand.data != -99)
  {
    m_chassisCommandPub.publish(command);
  }
}

/*void base_controller_nodelet::enableControlCallback(const ros::TimerEvent& time)
{
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  if(!nhPvt.getParam("controlEnabled", m_controlEnabled) )
  {
    NODELET_ERROR("Could not update base_controller_nodelet params");
  }
}*/

void base_controller_nodelet::controlCallback(const ros::TimerEvent& time)
{
  ros::NodeHandle nhPvt = getPrivateNodeHandle();
  nhPvt.getParam("KP", m_constantSpeedKP);
  nhPvt.getParam("KD", m_constantSpeedKD);
  nhPvt.getParam("KI", m_constantSpeedKI);
  nhPvt.getParam("IMax", m_constantSpeedIMax);

}

void base_controller_nodelet::loadThrottleCalibration()
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
      NODELET_INFO_STREAM("base_controller_nodelet added to add mapping " <<
                             toAdd.first << ":" << toAdd.second);
      if(!m_throttleMappings.update(toAdd))
      {
        NODELET_ERROR_STREAM("base_controller_nodelet Failed to add mapping " <<
                             toAdd.first << ":" << toAdd.second);
      }
    } else
    {
      NODELET_ERROR("base_controller_nodelet: XmlRpc throttle calibration formatted incorrectly");
    }
  }
  NODELET_INFO_STREAM("base_controller_nodelet: Loaded " <<
                      m_throttleMappings.size() <<
                      " throttle mappings");
}

}