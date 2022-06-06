#pragma once

#include <ros/ros.h>
#include <greenseeker_msgs/GreenSeekerPacket.h>

namespace greenseeker_driver
{
class GreenSeeker
{
public:
  GreenSeeker(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh), loop_rate_(200.0)
  {
    // get loop rate

    // get serial connection details

    // try and open serial port
    connect();

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

  // connection details
  std::string port_;
  int baud_;

  // serial port

  // packet publisher
  ros::Publisher packet_pub_;

  // timers
  ros::Timer read_poll_timer_;  // polling timer
};

}  // namespace greenseeker_driver
