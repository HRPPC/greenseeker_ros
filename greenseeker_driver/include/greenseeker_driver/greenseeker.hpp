#pragma once

#include <ros/ros.h>
#include <greenseeker_msgs/GreenSeekerPacket.h>

#include "simple_serial.h"

namespace greenseeker_driver
{
class GreenSeeker
{
public:
  GreenSeeker(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh), loop_rate_(100.0), frame_id_("greenseeker")
  {
    // get loop rate
    loop_rate_ = pnh_.param("loop_rate", loop_rate_);
    // frame id
    frame_id_ = pnh_.param("frame_id", frame_id_);

    // get serial connection details
    if (!pnh_.getParam("port", port_) && !pnh_.getParam("baud", baud_))
    {
      ROS_ERROR("greenseeker_driver missing serial parametes");
      return;
    }

    // try and open serial port
    connect();

    // create a greenseeker packet message publisher
    packet_pub_ = pnh.advertise<greenseeker_msgs::GreenSeekerPacket>("packet", 20);

    // start polling timer
    read_poll_timer_ = pnh_.createTimer(loop_rate_.expectedCycleTime(), &GreenSeeker::read_timer_cb, this);
  }

  ~GreenSeeker()
  {
    disconnect();
  }

  const bool connect();
  const bool disconnect();
  const bool read();

  void read_timer_cb(const ros::TimerEvent& event);

private:
  // ros network
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Rate loop_rate_;
  std::string frame_id_;

  // connection details
  std::string port_;
  int baud_;

  // serial port
  std::shared_ptr<SimpleSerial> serial_;

  // packet publisher
  ros::Publisher packet_pub_;

  // timers
  ros::Timer read_poll_timer_;  // polling timer
};

}  // namespace greenseeker_driver
