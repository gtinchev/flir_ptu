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

#ifndef FLIR_PTU_DRIVER_DRIVER_H
#define FLIR_PTU_DRIVER_DRIVER_H

// serial defines
#define PTU_DEFAULT_BAUD 9600
#define PTU_BUFFER_LEN 255
#define PTU_DEFAULT_PORT "/dev/ttyUSB0"
#define PTU_DEFAULT_HZ 10

// command defines
#define PTU_PAN 'p'
#define PTU_TILT 't'

#define PTU_MIN 'n'
#define PTU_MAX 'x'
#define PTU_MIN_SPEED 'l'
#define PTU_MAX_SPEED 'u'
#define PTU_BASE_SPEED 'b'

#define PTU_POSITION 'p'
#define PTU_SPEED 's'
#define PTU_ACCELERATION 'a'

#define PTU_CONTROL_MODE 'c'
#define PTU_CONTROL_PURE_VELOCITY 'v'
#define PTU_CONTROL_INDEPENDENT_POSITION 'i'

#define PTU_HOLD_POWER_MODE 'h'
#define PTU_MOVE_POWER_MODE 'm'

#define PTU_POWER_MODE_REGULAR 'r'
#define PTU_POWER_MODE_LOW 'l'

#define PTU_HOLD_POWER_OFF 'o'
#define PTU_MOVE_POWER_HIGH 'h'

#include <string>

namespace serial
{
class Serial;
}

namespace flir_ptu_driver
{

class PTU
{
public:
  /** Constructor - opens port
   * \param ser serial::Serial instance ready to communciate with device.
   */
  explicit PTU(serial::Serial* ser) :
    ser_(ser), initialized_(false)
  {
  }

  /** \return true if initialization succeeds. */
  bool initialize();

  /** \return true if the serial port is open and PTU initialized. */
  bool initialized();

  /**
   * \param type 'p' or 't'
   * \return position in radians
   */
  double getPosition(char type);

  /**
   * \param type 'p' or 't'
   * \return speed in radians/second
   */
  double getSpeed(char type);

  /**
   * \param type 'p' or 't'
   * \return speed in radians/second
   */
  double getAcceleration(char type);

  /**
   * \param type 'p' or 't'
   * \return resolution in radians/count
   */
  double getResolution(char type)
  {
    return (type == PTU_TILT ? tr : pr);
  }

  /**
   * \param type 'p' or 't'
   * \return Minimum position in radians
   */
  double getMin(char type)
  {
    return getResolution(type) * (type == PTU_TILT ? TMin : PMin);
  }
  /**
   * \param type 'p' or 't'
   * \return Maximum position in radians
   */
  double getMax(char type)
  {
    return getResolution(type) * (type == PTU_TILT ? TMax : PMax);
  }

  /**
   * \param type 'p' or 't'
   * \return Minimum speed in radians/second
   */
  double getMinSpeed(char type)
  {
    return getResolution(type) * (type == PTU_TILT ? TSMin : PSMin);
  }

  /**
   * \param type 'p' or 't'
   * \return Maximum speed in radians/second
   */
  double getMaxSpeed(char type)
  {
    return getResolution(type) * (type == PTU_TILT ? TSMax : PSMax);
  }

  /**
   * \param type 'p' or 't'
   * \return Base speed in radians/second
   */
  double getBaseSpeed(char type)
  {
    return getResolution(type) * (type == PTU_TILT ? TSBase : PSBase);
  }

  /**
   * Moves the PTU to the desired position. If Block is true,
   * the call blocks until the desired position is reached
   * \param type 'p' or 't'
   * \param pos desired position in radians
   * \param Block block until ready
   * \return True if successfully sent command
  */
  bool setPosition(char type, double pos, bool Block = false);

  /**
   * sets the desired speed in radians/second
   * \param type 'p' or 't'
   * \param speed desired speed in radians/second
   * \return True if successfully sent command
  */
  bool setSpeed(char type, double speed);

  /**
   * sets the base speed in radians/second
   * \param type 'p' or 't'
   * \param speed base speed in radians/second
   * \return True if successfully sent command
  */
  bool setBaseSpeed(char type, double speed);

  /**
   * sets the desired acceleration in radians/second^2
   * \param type 'p' or 't'
   * \param speed desired acceleration in radians/second^2
   * \return True if successfully sent command
  */
  bool setAcceleration(char type, double accel);

  /**
   * set the control mode, position or velocity
   * \param type PTU_CONTROL_PURE_VELOCITY for velocity, PTU_CONTROL_INDEPENDENT_POSITION for position
   * \return True if successfully sent command
   */
  bool setMode(char type);

  /**
   * get the control mode, position or velocity
   * \return PTU_CONTROL_PURE_VELOCITY for velocity, PTU_CONTROL_INDEPENDENT_POSITION for position
   */
  char getMode();

  /**
   * set hold (stationary) power mode
   * \param type PTU_PAN or PTU_TILT
   * \param mode PTU_POWER_MODE_REGULAR, PTU_POWER_MODE_LOW, PTU_HOLD_POWER_OFF
   * \return True if successfully sent command
   */
  bool setHoldPowerMode(char type, char mode);

  /**
   * get the hold power mode
   * \return mode PTU_POWER_MODE_REGULAR, PTU_POWER_MODE_LOW, PTU_HOLD_POWER_OFF
   */
  char getHoldPowerMode(char type);

  /**
   * set move (in-motion) power mode
   * \param type PTU_PAN or PTU_TILT
   * \param mode PTU_MOVE_POWER_HIGH, PTU_POWER_MODE_REGULAR or PTU_POWER_MODE_LOW
   * \return True if successfully sent command
   */
  bool setMovePowerMode(char type, char mode);

  /**
   * get move (in-motion) power mode
   * \return mode PTU_MOVE_POWER_HIGH, PTU_POWER_MODE_REGULAR or PTU_POWER_MODE_LOW
   */
  char getMovePowerMode(char type);

  bool home();

private:
  /** get radian/count resolution
   * \param type 'p' or 't'
   * \return pan resolution if type=='p', tilt resolution if type=='t'
   */
  double getRes(char type);

  /** get limiting position/speed in counts or counts/second
   *
   * \param type 'p' or 't' (pan or tilt)
   * \param limType {'n', 'x', 'l', 'u'} (min position, max position, min speed, max speed)
   * \return limiting position/speed
   */
  int getLimit(char type, char limType);

  // Position Limits
  int TMin;  ///< Min Tilt in Counts
  int TMax;  ///< Max Tilt in Counts
  int PMin;  ///< Min Pan in Counts
  int PMax;  ///< Max Pan in Counts

  // Speed Limits
  int TSMin;  ///< Min Tilt Speed in Counts/second
  int TSMax;  ///< Max Tilt Speed in Counts/second
  int PSMin;  ///< Min Pan Speed in Counts/second
  int PSMax;  ///< Max Pan Speed in Counts/second

  int PSBase;  ///< Pase Pan Speed in Counts/second
  int TSBase;  ///< Base Tilt Speed in Counts/second

protected:
  /** Sends a string to the PTU
   *
   * \param command string to be sent
   * \return response string from unit.
   */
  std::string sendCommand(std::string command);

  /**
   * set configuration
   * \param type 'p' or 't'
   * \param conf configration name
   * \param command value that will be set for the "conf"
   * \return True if successfully sent command
   */
  bool setConfiguration(char type, char conf, std::string command);

  /**
   * get configuration
   * \param type 'p' or 't'
   * \param conf configration name
   * \return configuration value
   */
  char getConfiguration(char type, char conf);

  serial::Serial* ser_;
  bool initialized_;

  double tr;  ///< tilt resolution (rads/count)
  double pr;  ///< pan resolution (rads/count)
};

}  // namespace flir_ptu_driver

#endif  // FLIR_PTU_DRIVER_DRIVER_H
