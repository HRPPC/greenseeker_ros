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
    double loop_hz = pnh_.param("loop_rate", 100.0);
    loop_rate_ = ros::Rate(loop_hz);

    // frame id
    frame_id_ = pnh_.param("frame_id", frame_id_);

    // get serial connection details
    if (!pnh_.getParam("port", port_) || !pnh_.getParam("baud", baud_))
    {
      ROS_ERROR("greenseeker_driver missing serial parametes");
      return;
    }

    ROS_INFO_STREAM("greenseeker_driver connecting " << port_ << " " << baud_);

    // try and open serial port
    if (!connect())
    {
      ROS_FATAL("greenseeker_driver connection failed");
      ros::shutdown();
      return;
    }

    // create a greenseeker packet message publisher
    packet_pub_ = pnh.advertise<greenseeker_msgs::GreenSeekerPacket>("packet", 20);

    // start polling timer
    read_poll_timer_ = pnh_.createTimer(loop_rate_.expectedCycleTime(), &GreenSeeker::read_timer_cb, this, true);
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
