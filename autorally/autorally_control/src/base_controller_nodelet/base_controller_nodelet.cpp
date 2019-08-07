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
        m_lastError(0.0),
        m_derivativeError(0.0),
        m_integralError(0.0),
        m_lastSpeedEst(0.0)
    {}

    base_controller_nodelet::~base_controller_nodelet()
    {}

    void base_controller_nodelet::onInit()
    {
        m_node_name = "base_controller_nodelet";

        NODELET_INFO("%s initialization", m_node_name.c_str());
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle nhPvt = getPrivateNodeHandle();

        //loadThrottleCalibration();

        m_mostRecentSpeedCommand = 99;
        m_mostRecentYawCommand = 99;
        m_reverse = false;

        m_steering_command = -5;

        m_speedCommandSub = nh.subscribe("cmd_vel", 1,
                &base_controller_nodelet::speedCallback, this);
        m_wheelSpeedsSub = nh.subscribe("wheelSpeeds", 1,
                &base_controller_nodelet::wheelSpeedsCallback,
                this);
        m_chassisCommandPub = nh.advertise<autorally_msgs::chassisCommand>
            ("base_controller_nodelet/chassisCommand", 1);
        m_speedEstPub = nh.advertise<std_msgs::Float64>("base_controller_nodelet/speedEst", 1);

        if(!nhPvt.getParam("speedCommander", m_speedCommander) ||
                !nhPvt.getParam("accelerationRate", m_accelerationRate) ||
                !nhPvt.getParam("KP", m_constantSpeedKP) ||
                !nhPvt.getParam("KD", m_constantSpeedKD) ||
                !nhPvt.getParam("KI", m_constantSpeedKI) ||
                !nhPvt.getParam("IMax", m_constantSpeedIMax) ||
                !nhPvt.getParam("alpha", m_alpha))
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
        // NODELET_INFO("%s: New X speed: %f", m_node_name.c_str(), msg->linear.x);


        // Handle Throttle
        //double speed_multiplier = 1.5; 
        m_mostRecentSpeedCommand = msg->linear.x; // removed multiplier -- JF
        m_mostRecentYawCommand = msg->angular.z;

        // Handle Reverse
        // Set reverse flag based on the sign of the speed command
        if (msg->linear.x < 0)
        {
            m_reverse = true;
        }
        else
        {
            m_reverse = false;
        }

    }

    double base_controller_nodelet::Clamp(double num, double min, double max)
    {
        if (num < min)
            num = min;
        if (num > max)
            num = max;
        return num; 
    }

    void base_controller_nodelet::wheelSpeedsCallback(const autorally_msgs::wheelSpeedsConstPtr& msg)
    {
        m_frontWheelsSpeed = 0.5*(msg->lfSpeed + msg->rfSpeed);
        m_lastSpeedEst = (1-m_alpha) * m_lastSpeedEst + m_alpha * m_frontWheelsSpeed;

        std_msgs::Float64Ptr speedEst(new std_msgs::Float64);
        speedEst->data = m_lastSpeedEst;

        m_speedEstPub.publish(speedEst);
        //m_backWheelsSpeed = 0.2*m_backWheelsSpeed + 0.4*(msg->lbSpeed + msg->rbSpeed);

        // Handle Steering
        // angle (rads) = atan(wheelbase * wz / wx)
        // If current velocity is zero, set steering to zero.
        // If current wheel speed is fast, use that when calculating steering
        // Otherwise, use commanded wheel speed
        // Thanks DataSpeed!
        // Assume that we cannot turn more than 45 deg (pi/8 rads)
        // 0.5588 is measured wheelbase)
        /*double omega = 0.0;
          if(std::abs(msg->linear.x) > 1e-2 or std::abs(m_frontWheelsSpeed) > 1e-2)
          {
          omega = std::atan(0.5588*msg->angular.z/(m_frontWheelsSpeed > 0.5 ? m_frontWheelsSpeed : msg->linear.x))/(3.14/8);
          }
          m_steering_command = Clamp(- omega,-1.0,1.0); */

        // -JF-
        // Max angle (from xacro robot description) is 21 deg
        // wheelbase measured to be 0.5588 m -> autorally desc says 0.570 m
        // moved steering calculation here to take advantage of most recent speed measurement
        double angle = 0.0;
        double wheelbase = 0.570; // meters
        double max_angle = 21 * DEGTORAD;
        // to avoid dividing by zero, check to see if the wheelspeed is less than what is 
        // required to achieve requested yaw (given max angle) and set it to max_angle,
        // otherwise do the calculation
        if (std::abs(m_frontWheelsSpeed) <=
                std::tan(max_angle) * std::abs(wheelbase*m_mostRecentYawCommand))
            angle = m_mostRecentYawCommand > 0 ? max_angle : -max_angle;
        else
            angle = std::atan(wheelbase*m_mostRecentYawCommand/std::abs(m_frontWheelsSpeed));
        m_steering_command = Clamp(angle / max_angle, -1.0, 1.0);
        // -JF-

        autorally_msgs::chassisCommandPtr command(new autorally_msgs::chassisCommand);
        command->header.stamp = ros::Time::now();
        command->sender = m_node_name;
        command->steering = m_steering_command;
        command->frontBrake = 0.0;
        command->reverse = m_reverse;

        double abs_goal_speed = std::abs(m_mostRecentSpeedCommand);
        double abs_front_wheel_speed = std::abs(m_frontWheelsSpeed);

        double speed_diff = abs_goal_speed - std::abs(m_lastSpeedEst); //abs_front_wheel_speed;
        // braking -- disabled JF
        //if (speed_diff < 0 || !((0 > m_mostRecentSpeedCommand) == (0 > m_frontWheelsSpeed)) || -0.1 < abs_goal_speed < 0.1)
            //if (!((0 > m_mostRecentSpeedCommand) == (0 > m_frontWheelsSpeed)) )
        //{
            //command->frontBrake = std::max(0.0, std::min(1.0, std::abs(speed_diff)));
        //    command->frontBrake = 0.0;
        //}


        if (abs_goal_speed > 0.1 && abs_goal_speed < 99)
        {
            double p;
            if(true) //m_throttleMappings.interpolateKey(abs_goal_speed, p))
            {
                m_integralError += speed_diff;
                m_integralError = Clamp(m_integralError, -m_constantSpeedIMax,
                        m_constantSpeedIMax);

                m_derivativeError = speed_diff - m_lastError;
                m_lastError = speed_diff;

                command->throttle = m_constantSpeedPrevThrot;
                command->throttle += m_constantSpeedKP * speed_diff;
                command->throttle += m_constantSpeedKI * m_integralError;
                command->throttle += m_constantSpeedKD * m_derivativeError;
                command->throttle = Clamp(command->throttle, 0.0, 1.0);

                // NODELET_INFO("interp %f, command %f",p, command->throttle);
                //m_constantSpeedPrevThrot = p;
                m_constantSpeedPrevThrot = command->throttle;
            }
            else
            {
                NODELET_WARN("base_controller_nodelet could not interpolate speed %f", m_mostRecentSpeedCommand);
                command->throttle = 0;
            }
        }
        else
        {
            command->throttle = 0;

        }

        //command->throttle = 1.0;
        //ROS_ERROR("Speed command: %f", command->throttle);
        if (abs_goal_speed != -99)
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
        nhPvt.getParam("alpha", m_alpha);

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
