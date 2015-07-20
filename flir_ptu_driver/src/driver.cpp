/*
 * flir_ptu_driver ROS package
 * Copyright (C) 2014 Mike Purvis (mpurvis@clearpathrobotics.com)
 *
 * PTU ROS Package
 * Copyright (C) 2009 Erik Karulf (erik@cse.wustl.edu)
 *
 * Author: Toby Collett (University of Auckland)
 * Date: 2003-02-10
 *
 * Player - One Hell of a Robot Server
 * Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                     gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <flir_ptu_driver/driver.h>
#include <serial/serial.h>
#include <ros/console.h>

#include <math.h>
#include <string>
#include <time.h>

using boost::lexical_cast;


namespace flir_ptu_driver
{

/** Templated wrapper function on lexical_cast to assist with extracting
 * values from serial response strings.
 */
template<typename T>
T parseResponse(std::string responseBuffer)
{
  std::string trimmed = responseBuffer.substr(1);
  boost::trim(trimmed);
  T parsed = lexical_cast<T>(trimmed);
  ROS_DEBUG_STREAM("Parsed response value: " << parsed);
  return parsed;
}

bool PTU::initialized()
{
  return !!ser_ && ser_->isOpen() && initialized_;
}

bool PTU::initialize()
{
  std::string rst;

  // clean up the buffer
  rst = ser_->read(20);

  rst = sendCommand("ft ");  // terse feedback
  rst = sendCommand("ed ");  // disable echo
  rst = sendCommand("ci ");  // position mode

  // get pan tilt encoder res
  tr = getRes(PTU_TILT);
  pr = getRes(PTU_PAN);

  PMin = getLimit(PTU_PAN, PTU_MIN);
  PMax = getLimit(PTU_PAN, PTU_MAX);
  TMin = getLimit(PTU_TILT, PTU_MIN);
  TMax = getLimit(PTU_TILT, PTU_MAX);
  PSMin = getLimit(PTU_PAN, PTU_MIN_SPEED);
  PSMax = getLimit(PTU_PAN, PTU_MAX_SPEED);
  TSMin = getLimit(PTU_TILT, PTU_MIN_SPEED);
  TSMax = getLimit(PTU_TILT, PTU_MAX_SPEED);
  PSBase = getLimit(PTU_PAN, PTU_BASE_SPEED);
  TSBase = getLimit(PTU_TILT, PTU_BASE_SPEED);

  if (tr <= 0      ||
      pr <= 0      ||
      PMin == -1   ||
      PMax == -1   ||
      TMin == -1   ||
      TMax == -1   ||
      PSMin == -1  ||
      PSMax == -1  ||
      TSMin == -1  ||
      TSMax == -1  ||
      PSBase == -1 ||
      TSBase == -1) {

    initialized_ = false;
  }
  else
  {
    initialized_ = true;
  }

  return initialized();
}

std::string PTU::sendCommand(std::string command)
{
  ser_->write(command);
  ROS_DEBUG_STREAM("TX: " << command);
  std::string buffer = ser_->readline(PTU_BUFFER_LEN);
  ROS_DEBUG_STREAM("RX: " << buffer);
  return buffer;
}

bool PTU::home()
{
  ROS_INFO("Sending command to reset PTU.");

  // Issue reset command
  ser_->flush();
  ser_->write(" r ");

  std::string actual_response, expected_response("!T!T!P!P*");

  // 30 seconds to receive full confirmation of reset action completed.
  for (int i = 0; i < 300; i++)
  {
    usleep(100000);

    if (ser_->available() >= expected_response.length())
    {
      ROS_INFO("PTU reset command response received.");
      ser_->read(actual_response, expected_response.length());
      return (actual_response == expected_response);
    }
  }

  ROS_WARN("PTU reset command response not received before timeout.");
  return false;
}

bool PTU::setBaudRate(const unsigned int rate)
{
  if (rate > 0) {
    std::string buffer = sendCommand(std::string() + "@(" +  lexical_cast<std::string>(rate) + ",0,f) ");

    if (buffer.length() < 3 || buffer[0] != '*') {
      ROS_ERROR("Error setting serial communication baud rate");
      return false;
    }
    else {
      return true;
    }
  }
  else {
    return false;
  }
}


// get radians/count resolution
double PTU::getRes(char type)
{
  if (!ser_ || !ser_->isOpen()) return -1;

  std::string buffer = sendCommand(std::string() + type + "r ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error getting pan-tilt res");
    return -1;
  }

  double z = parseResponse<double>(buffer);
  z = z / 3600.;  // degrees/count
  return z * M_PI / 180.;  // radians/count
}

// get position limit
int PTU::getLimit(char type, char limType)
{
  if (!ser_ || !ser_->isOpen()) return -1;

  std::string buffer = sendCommand(std::string() + type + limType + " ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error getting pan-tilt limit");
    return -1;
  }

  return parseResponse<int>(buffer);
}


// get position in radians
double PTU::getPosition(char type)
{
  if (!initialized()) return -1;

  std::string buffer = sendCommand(std::string() + type + "p ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error getting pan-tilt pos");
    return -1;
  }

  return parseResponse<double>(buffer) * getResolution(type);
}


// set position in radians
bool PTU::setPosition(char type, double pos, bool block)
{
  if (!initialized()) return false;

  // get raw encoder count to move
  int count = static_cast<int>(pos / getResolution(type));

  // Check limits
  if (count < (type == PTU_TILT ? TMin : PMin) || count > (type == PTU_TILT ? TMax : PMax))
  {
    ROS_ERROR("Pan Tilt Value out of Range: %c %f(%d) (%d-%d)\n",
              type, pos, count, (type == PTU_TILT ? TMin : PMin), (type == PTU_TILT ? TMax : PMax));
    return false;
  }

  std::string buffer = sendCommand(std::string() + type + "p" +
                                   lexical_cast<std::string>(count) + " ");

  if (buffer.empty() || buffer[0] != '*')
  {
    ROS_ERROR("Error setting pan-tilt pos");
    return false;
  }

  if (block)
    while (getPosition(type) != pos) {};

  return true;
}

// get speed in radians/sec
double PTU::getSpeed(char type)
{
  if (!initialized()) return -1;

  std::string buffer = sendCommand(std::string() + type +  PTU_SPEED + " ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error getting pan-tilt speed");
    return -1;
  }

  return parseResponse<double>(buffer) * getResolution(type);
}

// get speed in radians/sec
double PTU::getAcceleration(char type)
{
  if (!initialized()) return -1;

  std::string buffer = sendCommand(std::string() + type +  PTU_ACCELERATION + " ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error getting pan-tilt acceleration");
    return -1;
  }

  return parseResponse<double>(buffer) * getResolution(type);
}



// set speed in radians/sec
bool PTU::setSpeed(char type, double pos)
{
  if (!initialized()) return false;

  // get raw encoder speed to move
  int count = static_cast<int>(pos / getResolution(type));

  // Check limits
  if (abs(count) < (type == PTU_TILT ? TSMin : PSMin) || abs(count) > (type == PTU_TILT ? TSMax : PSMax))
  {
    ROS_ERROR("Pan Tilt Speed Value out of Range: %c %f(%d) (%d-%d)\n",
              type, pos, count, (type == PTU_TILT ? TSMin : PSMin), (type == PTU_TILT ? TSMax : PSMax));
    return false;
  }

  std::string buffer = sendCommand(std::string() + type + "s" +
                                   lexical_cast<std::string>(count) + " ");

  if (buffer.empty() || buffer[0] != '*')
  {
    ROS_ERROR("Error setting pan-tilt speed\n");
    return false;
  }

  return true;
}

bool PTU::setBaseSpeed(char type, double speed)
{
  if (!initialized()) return false;

  int count = static_cast<int>(speed / getResolution(speed));

  // Check limits
  if (abs(count) < (type == PTU_TILT ? TSMin : PSMin) || abs(count) > (type == PTU_TILT ? TSMax : PSMax))
  {
    ROS_ERROR("Pan Tilt Base Speed Value out of Range: %c %f(%d) (%d-%d)\n",
              type, speed, count, (type == PTU_TILT ? TSMin : PSMin), (type == PTU_TILT ? TSMax : PSMax));
    return false;
  }

  std::string buffer = sendCommand(std::string() + type + PTU_BASE_SPEED +
                                   lexical_cast<std::string>(count) + " ");

  if (buffer.empty() || buffer[0] != '*')
  {
    ROS_ERROR("Error setting pan-tilt base speed\n");
    std::cout << buffer << std::endl;
    return false;
  }

  return true;
}

// set acceleration in radians/sec^2
bool PTU::setAcceleration(char type, double accel)
{
  if (!initialized()) return false;

  // get raw encoder accel to move
  int count = static_cast<int>(accel / getResolution(type));

  std::string buffer = sendCommand(std::string() + type + PTU_ACCELERATION +
                                   lexical_cast<std::string>(count) + " ");

  if (buffer.empty() || buffer[0] != '*')
  {
    ROS_ERROR("Error setting axis(%c) acceleration %f rad/sec^2 (count %d). \n %s\n",
              type, accel, count, buffer.c_str());
    return false;
  }

  return true;
}



// set movement mode (position/velocity)
bool PTU::setMode(char type)
{
  if (!initialized()) return false;

  std::string buffer = sendCommand(std::string() + PTU_CONTROL_MODE + type + " ");

  if (buffer.empty() || buffer[0] != '*')
  {
    ROS_ERROR("Error setting pan-tilt move mode");
    return false;
  }

  return true;
}

// get ptu mode
char PTU::getMode()
{
  if (!initialized()) return -1;

  // get pan tilt mode
  std::string buffer = sendCommand("c ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error getting pan-tilt pos");
    return -1;
  }

  if (buffer[2] == PTU_CONTROL_PURE_VELOCITY) {
    return PTU_CONTROL_PURE_VELOCITY;
  }
  else if (buffer[2] == PTU_CONTROL_INDEPENDENT_POSITION) {
    return PTU_CONTROL_INDEPENDENT_POSITION;
  }
  else {
    ROS_ERROR("unknown control mode");
    return -1;
  }
}

bool PTU::setHoldPowerMode(char type, char mode)
{
  return setConfiguration(type, PTU_HOLD_POWER_MODE, std::string(1, mode));
}

char PTU::getHoldPowerMode(char type)
{
  return getConfiguration(type, PTU_HOLD_POWER_MODE);
}

bool PTU::setMovePowerMode(char type, char mode)
{
  return setConfiguration(type, PTU_MOVE_POWER_MODE, std::string(1, mode));
}

char PTU::getMovePowerMode(char type)
{
  return getConfiguration(type, PTU_MOVE_POWER_MODE);
}

bool PTU::setConfiguration(char type, char conf, std::string command)
{
  if (!initialized()) return -1;

  std::string buffer = sendCommand(std::string() + type + conf + command + " ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error setting configuration! Axis: %c, Configuration name: %c, Value %s \n",
              type, conf, command.c_str());
    return -1;
  }
}

char PTU::getConfiguration(char type, char conf)
{
  if (!initialized()) return -1;

  // get
  std::string buffer = sendCommand(std::string() + type + conf + " ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error getting configuration! Axis: %c, Configuration name: %c \n",
              type, conf);
    return -1;
  }
  else {
    return buffer[2];
  }
}


}  // namespace flir_ptu_driver
