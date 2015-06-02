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
 Original author: Vijay Pradeep , Adolfo Rodriguez Tsouroukdissian, Wim Meeussen
 Contributors: Jonathan Bohren, Wim Meeussen, Dave Coleman
 Desc: Effort(force)-based position controller receiving joint_state as input using basic PID loop
*/

#include <flir_ptu_controllers/ptu_position_controller.h>
#include <pluginlib/class_list_macros.h>

namespace effort_controllers {

FlirPTUPositionController::FlirPTUPositionController() 
 :k_i_vel_(20.0),
 i_min_vel_(-10.0),
 i_max_vel_(10.0),
 max_acceleration_(5.0),
 publish_rate_(0.0)
 {}

FlirPTUPositionController::~FlirPTUPositionController()
{
  sub_command_.shutdown();
}

bool FlirPTUPositionController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  
  // List of controlled joints
  std::string param_name = "joints";
  if(!n.getParam(param_name, joint_names_))
  {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }
  
  n_joints_ = joint_names_.size();
  
  if(n_joints_ == 0){
    ROS_ERROR_STREAM("List of joint names is empty.");
    return false;
  }
  
  pid_controllers_.resize(n_joints_);
  pid_controllers_vel_.resize(n_joints_);
  max_vels_ = std::vector<double>(n_joints_,PTU_DEFAULT_MAX_VEL);
  curr_commanded_velocity_ = std::vector<double>(n_joints_,0.0);
  
  // internal storage
  Commands empty_cmd;
  empty_cmd.position_=0.0;
  empty_cmd.velocity_=PTU_DEFAULT_MAX_VEL;
  empty_cmd.has_velocity_=false;
  commands_=std::vector<Commands>(n_joints_, empty_cmd);
  
  // get publishing period
  if (!n.getParam("publish_rate", publish_rate_)){
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }
  
  // realtime publisher
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(n, "joint_states", 4));

  // init handles, controller pids and joint_state publisher
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
    
    // Init joint state publisher
    realtime_pub_->msg_.name.push_back(joint_names_[i]);
    realtime_pub_->msg_.position.push_back(0.0);
    realtime_pub_->msg_.velocity.push_back(PTU_DEFAULT_MAX_VEL);
    realtime_pub_->msg_.effort.push_back(0.0);   
       
    // Node handle to PID gains
    ros::NodeHandle joint_nh(n, std::string("gains/") + joint_names_[i]);

    // Init PID gains from ROS parameter server
    pid_controllers_[i].reset(new control_toolbox::Pid());
    pid_controllers_vel_[i].reset(new control_toolbox::Pid());
    if (!pid_controllers_[i]->init(joint_nh))
    {
      ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
      return false;
    }
    // internal velocity controller
    pid_controllers_vel_[i]->initPid(0.0, k_i_vel_ , 0.0, i_max_vel_ , i_min_vel_,n);
    
  }  
  
  // output RT buffer    
  commands_buffer_.writeFromNonRT(commands_);
  // Start command subscriber
  sub_command_ = n.subscribe<sensor_msgs::JointState>("cmd", 1, &FlirPTUPositionController::setCommandCB, this);
 
  return true;
}

void FlirPTUPositionController::setGains(const size_t id, const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  if(id <n_joints_)
    pid_controllers_[id]->setGains(p,i,d,i_max,i_min);
}

void FlirPTUPositionController::getGains(const size_t id, double &p, double &i, double &d, double &i_max, double &i_min)
{
  if(id <n_joints_)
    pid_controllers_[id]->getGains(p,i,d,i_max,i_min);
}

void FlirPTUPositionController::printDebug(const size_t id)
{
  if(id <n_joints_)
    pid_controllers_[id]->printValues();
}

std::string FlirPTUPositionController::getJointName(const size_t id)
{
  if(id <n_joints_)
    return joint_names_[id];
}

double FlirPTUPositionController::getPosition(const size_t id)
{
  //return joint_.getPosition();
  return 0.0;
}

// Set the joint position command
void FlirPTUPositionController::setCommand(const size_t id, double pos_command)
{
  if(id <n_joints_)
  {
    commands_[id].position_ = pos_command;
    commands_[id].velocity_ = max_vels_[id];
    commands_[id].has_velocity_ = true; 

    // the writeFromNonRT can be used in RT, if you have the guarantee that
    //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
    //  * there is only one single rt thread
    commands_buffer_.writeFromNonRT(commands_);
  }
}

// Set the joint position command with a velocity command as well
void FlirPTUPositionController::setCommand(const size_t id, double pos_command, double vel_command)
{
  if(id <n_joints_)
  {
    commands_[id].position_ = pos_command;
    commands_[id].velocity_ = vel_command;
    max_vels_[id] = vel_command;
    commands_[id].has_velocity_ = true;
    commands_buffer_.writeFromNonRT(commands_);
  }
}

void FlirPTUPositionController::starting(const ros::Time& time)
{
  // initialize time
  last_publish_time_ = time;
  
  // Reset PIDs, zero effort commands, init commands to current pos
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    double pos_command = joints_[i].getPosition();
    
    pid_controllers_[i]->reset();
    pid_controllers_vel_[i]->reset();
    curr_commanded_velocity_[i]=0.0;
    
    joints_[i].setCommand(0.0);
 
    commands_[i].position_ = pos_command;
    commands_[i].velocity_ = max_vels_[i];
    commands_[i].has_velocity_ = true;
  }
  commands_buffer_.initRT(commands_);
}

void FlirPTUPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  // Update PIDs
  commands_ = *(commands_buffer_.readFromRT());
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    double command_position = commands_[i].position_;
    double command_velocity;
    bool has_velocity_ =  commands_[i].has_velocity_;
    
    double error, vel_error;
    double commanded_effort;
    
    double current_position = joints_[i].getPosition();
    
    //TODO reintroduce shortest_angular_distance ?
    error = command_position - current_position;
    
    // commanded velocity
    command_velocity = pid_controllers_[i]->computeCommand(error, period);
    // velocity profile
    
    double time_to_stop = max_vels_[i] / max_acceleration_;
    double dist_to_stop = 0.5 * max_acceleration_ * time_to_stop * time_to_stop;
    ROS_DEBUG_STREAM("dist to stop "<< dist_to_stop << " ,time to stop " << time_to_stop << " commanded vel " <<command_velocity);
    
    if(command_velocity > 0.0)
    {
      if (current_position > command_position) //time to ramp-down + 1.2*dist_to_stop
      {
        if(curr_commanded_velocity_[i] >= 0.0)
          curr_commanded_velocity_[i]-= max_acceleration_*period.toSec();
        else
          curr_commanded_velocity_[i] = 0.0;
      }
      else//ramp-up to max
      {
        if(curr_commanded_velocity_[i] <= command_velocity)
          curr_commanded_velocity_[i]+= max_acceleration_*period.toSec();
        else
          curr_commanded_velocity_[i] = command_velocity;
      }
    }
    else
    {
      if (current_position < command_position) //time to ramp-down - 1.2*dist_to_stop 
      {
        if(curr_commanded_velocity_[i] < 0.0)
          curr_commanded_velocity_[i]+= max_acceleration_*period.toSec();
        else
          curr_commanded_velocity_[i] = 0.0;
      }
      else//ramp-up to max
      {
         if(curr_commanded_velocity_[i] > command_velocity)
          curr_commanded_velocity_[i]-= max_acceleration_*period.toSec();
        else
          curr_commanded_velocity_[i] = command_velocity;
      }
    }
    
    //deadband ?
    //if(std::fabs( curr_commanded_velocity_[i] - command_velocity ) < PTU_EPS_VELOCITY)
    //{
    //  curr_commanded_velocity_[i]=  command_velocity;
    // }

    
    // reached max speed, stay at this speed
    curr_commanded_velocity_[i] = std::max( -max_vels_[i], std::min( curr_commanded_velocity_[i], max_vels_[i] ) );
   
    // compute effort according to velocity error
    vel_error = curr_commanded_velocity_[i] - joints_[i].getVelocity();
    ROS_DEBUG_STREAM("velerr "<< vel_error << " ,jnt vel " << joints_[i].getVelocity() << " , cur comm vel: "<< curr_commanded_velocity_[i] << " commanded vel " <<command_velocity);
    commanded_effort =  pid_controllers_vel_[i]->computeCommand(vel_error,period);
    
    // use the handle to set the effort
    joints_[i].setCommand(commanded_effort);
  }
  
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){

    // try to publish
    if (realtime_pub_->trylock()){
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

      // populate joint state message
      realtime_pub_->msg_.header.stamp = time;
      for (unsigned i=0; i<n_joints_; i++){
        realtime_pub_->msg_.position[i] = joints_[i].getPosition();
        realtime_pub_->msg_.velocity[i] = curr_commanded_velocity_[i];//max_vels_[i];
        realtime_pub_->msg_.effort[i] = joints_[i].getEffort();
      }
      realtime_pub_->unlockAndPublish();
    }
  }
}

void FlirPTUPositionController::setCommandCB(const sensor_msgs::JointStateConstPtr& msg)
{
  bool has_velocity_=false;
  if(msg->name.size()!=n_joints_ ||  msg->position.size()!=n_joints_ || msg->velocity.size()!=n_joints_)
  { 
    ROS_ERROR_STREAM("Dimension of command (name or position or velocity does not match number of joints (" << n_joints_ << ")! Not executing!");
    return; 
  }
  has_velocity_=true;
    
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    commands_[i].position_=msg->position[i];
    commands_[i].velocity_=msg->velocity[i];
    //store max_vels for joint_state publishing purposes
    max_vels_[i]=msg->velocity[i];
    commands_[i].has_velocity_=has_velocity_;
  }
  commands_buffer_.writeFromNonRT(commands_);  
}


} // namespace

PLUGINLIB_EXPORT_CLASS( effort_controllers::FlirPTUPositionController, controller_interface::ControllerBase)
