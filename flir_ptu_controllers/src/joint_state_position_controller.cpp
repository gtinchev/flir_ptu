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

/*
 Author: Guillaume Walck
 Original author: Vijay Pradeep , Adolfo Rodriguez Tsouroukdissian
 Contributors: Jonathan Bohren, Wim Meeussen, Dave Coleman
 Desc: Effort(force)-based position controller receiving joint_state as input using basic PID loop
*/

#include <flir_ptu_controllers/joint_state_position_controller.h>
#include <pluginlib/class_list_macros.h>

namespace effort_controllers {

JointStatePositionController::JointStatePositionController()
{}

JointStatePositionController::~JointStatePositionController()
{
  sub_command_.shutdown();
}

bool JointStatePositionController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  
  // List of controlled joints
  std::string param_name = "joints";
  if(!n.getParam(param_name, joint_names_))
  {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }
  n_joints_ = joint_names_.size();
  pid_controllers_.resize(n_joints_);
  Commands empty_cmd;
  empty_cmd.position_=0.0;
  empty_cmd.velocity_=0.0;
  empty_cmd.has_velocity_=false;
  
  commands_=std::vector<Commands>(n_joints_, empty_cmd);

  if(n_joints_ == 0){
    ROS_ERROR_STREAM("List of joint names is empty.");
    return false;
  }
  for(unsigned int i=0; i<n_joints_; i++)
  {
    try
    {
      joints_.push_back(robot->getHandle(joint_names_[i]));  
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }
        
    // Node handle to PID gains
    ros::NodeHandle joint_nh(n, std::string("gains/") + joint_names_[i]);

    // Init PID gains from ROS parameter server
    pid_controllers_[i].reset(new control_toolbox::Pid());
    if (!pid_controllers_[i]->init(joint_nh))
    {
      ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
      return false;
    }
  }
      
  commands_buffer_.writeFromNonRT(commands_);
  // Start command subscriber
  sub_command_ = n.subscribe<sensor_msgs::JointState>("command", 1, &JointStatePositionController::setCommandCB, this);
 
  return true;
}

void JointStatePositionController::setGains(const size_t id, const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  if(id <n_joints_)
    pid_controllers_[id]->setGains(p,i,d,i_max,i_min);
}

void JointStatePositionController::getGains(const size_t id, double &p, double &i, double &d, double &i_max, double &i_min)
{
  if(id <n_joints_)
    pid_controllers_[id]->getGains(p,i,d,i_max,i_min);
}

void JointStatePositionController::printDebug(const size_t id)
{
  if(id <n_joints_)
    pid_controllers_[id]->printValues();
}

std::string JointStatePositionController::getJointName(const size_t id)
{
  if(id <n_joints_)
    return joint_names_[id];
}

double JointStatePositionController::getPosition(const size_t id)
{
  //return joint_.getPosition();
  return 0.0;
}

// Set the joint position command
void JointStatePositionController::setCommand(const size_t id, double pos_command)
{
  if(id <n_joints_)
  {
    commands_[id].position_ = pos_command;
    commands_[id].has_velocity_ = false; // Flag to ignore the velocity command since our setCommand method did not include it

    // the writeFromNonRT can be used in RT, if you have the guarantee that
    //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
    //  * there is only one single rt thread
    commands_buffer_.writeFromNonRT(commands_);
  }
}

// Set the joint position command with a velocity command as well
void JointStatePositionController::setCommand(const size_t id, double pos_command, double vel_command)
{
  if(id <n_joints_)
  {
    commands_[id].position_ = pos_command;
    commands_[id].velocity_ = vel_command;
    commands_[id].has_velocity_ = true;
    commands_buffer_.writeFromNonRT(commands_);
  }
}

void JointStatePositionController::starting(const ros::Time& time)
{
  // Reset PIDs, zero effort commands, init commands to current pos
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    double pos_command = joints_[i].getPosition();
    
    pid_controllers_[i]->reset();
    
    joints_[i].setCommand(0.0);
 
    commands_[i].position_ = pos_command;
    commands_[i].has_velocity_ = false;
  }
  commands_buffer_.initRT(commands_);
}

void JointStatePositionController::update(const ros::Time& time, const ros::Duration& period)
{
  // Update PIDs
  commands_ = *(commands_buffer_.readFromRT());
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    double command_position = commands_[i].position_;
    double command_velocity = commands_[i].velocity_;
    bool has_velocity_ =  commands_[i].has_velocity_;
    
    double error, vel_error;
    double commanded_effort;
    
    double current_position = joints_[i].getPosition();
    
    //TODO reintroduce shortest_angular_distance
    error = command_position - current_position;
    
    // Decide which of the two PID computeCommand() methods to call
    if (has_velocity_)
    {
      // Compute velocity error if a non-zero velocity command was given
      vel_error = command_velocity - joints_[i].getVelocity();

      // Set the PID error and compute the PID command with nonuniform
      // time step size. This also allows the user to pass in a precomputed derivative error.
      commanded_effort = pid_controllers_[i]->computeCommand(error, vel_error, period);
    }
    else
    {
      // Set the PID error and compute the PID command with nonuniform
      // time step size.
      commanded_effort = pid_controllers_[i]->computeCommand(error, period);
    }
    // use the handle to set the effort    
    joints_[i].setCommand(commanded_effort);
  }
}

void JointStatePositionController::setCommandCB(const sensor_msgs::JointStateConstPtr& msg)
{
  bool has_velocity_=false;
  if(msg->name.size()!=n_joints_)
  { 
    ROS_ERROR_STREAM("Dimension of command (" << msg->name.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
    return; 
  }
  //empty velocity vector accepted and means no velocities
  if(msg->velocity.size()==n_joints_)
    has_velocity_=true;
    
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    commands_[i].position_=msg->position[i];
    
    if(has_velocity_)
      commands_[i].velocity_=msg->velocity[i];
    commands_[i].has_velocity_=has_velocity_;
  }
  commands_buffer_.writeFromNonRT(commands_);  
}


} // namespace

PLUGINLIB_EXPORT_CLASS( effort_controllers::JointStatePositionController, controller_interface::ControllerBase)
