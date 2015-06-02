/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (C) 2013, PAL Robotics S.L.
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
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *   * Neither the name of PAL Robotics S.L. nor the names of its
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
 *********************************************************************/

#ifndef EFFORT_CONTROLLERS__PTU_POSITION_CONTROLLER_H
#define EFFORT_CONTROLLERS__PTU_POSITION_CONTROLLER_H

/**
   @class effort_controllers::FlirPTUPositionController
   @brief PTU Position Controller

   This class controls positon using a pid loop.

   @section ROS ROS interface

   @param type Must be "effort_controllers::FlirPTUPositionController"
   @param joint Name of the joint to control.
   @param pid Contains the gains for the PID loop around position.  See: control_toolbox::Pid

   Subscribes to:

   - @b command (sensor_msgs::JointState) : The joint position to achieve.

   Publishes:

   - @b state (control_msgs::JointControllerState) :
     Current state of the controller, including pid error and gains.

*/


#include <string>
#include <vector>

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>


#define PTU_DEFAULT_MAX_VEL 0.5
#define PTU_EPS_VELOCITY  0.01

namespace effort_controllers
{

class FlirPTUPositionController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:

  /**
   * \brief Store position and velocity command in struct to allow easier realtime buffer usage
   */
  struct Commands
  {
    double position_; // Last commanded position
    double velocity_; // Last commanded velocity
    bool has_velocity_; // false if no velocity command has been specified
  };

  FlirPTUPositionController();
  ~FlirPTUPositionController();

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */  
  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param command
   */
  void setCommand(const size_t id, double pos_target);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *        Also supports a target velocity
   *
   * \param pos_target - position setpoint
   * \param vel_target - velocity setpoint
   */
  void setCommand(const size_t id, double pos_target, double vel_target);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time);
  
  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Get the PID parameters
   */
  void getGains(const size_t id, double &p, double &i, double &d, double &i_max, double &i_min);

  /**
   * \brief Print debug info to console
   */
  void printDebug(const size_t id);

  /**
   * \brief Get the PID parameters
   */
  void setGains(const size_t id, const double &p, const double &i, const double &d, const double &i_max, const double &i_min);

  /**
   * \brief Get the name of the joint this controller uses
   */
  std::string getJointName(const size_t id);

  /**
   * \brief Get the current position of the joint
   * \return current position
   */
  double getPosition(const size_t id);

  std::vector< hardware_interface::JointHandle > joints_;  
  std::vector< std::string > joint_names_;
  realtime_tools::RealtimeBuffer<std::vector<Commands> > commands_buffer_;
  unsigned int n_joints_;
  
  std::vector<Commands> commands_ ;
  Commands command_struct_; // pre-allocated memory that is re-used to set the realtime buffer

private:
  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pid_controllers_; /**< Internal PID controllers. */
  std::vector<PidPtr> pid_controllers_vel_; /**< Internal PID controllers. */
  
  ros::Subscriber sub_command_;
  std::vector<double> max_vels_;
  std::vector<double> curr_commanded_velocity_;
  double k_i_vel_; /**< Internal velocity to effort igain */
  double i_max_vel_,i_min_vel_;
  double max_acceleration_;
  
  /**
   * \brief Callback from /command subscriber for setpoint
   */
  void setCommandCB(const sensor_msgs::JointStateConstPtr& msg);


  boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_;
  ros::Time last_publish_time_;
  double publish_rate_;

};

} // namespace

#endif
