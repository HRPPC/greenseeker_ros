#include <greenseeker_driver/greenseeker.hpp>

namespace greenseeker_driver
{

const bool GreenSeeker::connect()
{
  try
  {
    serial_ = std::make_shared<SimpleSerial>(port_, baud_);
    return true;
  }
  catch (boost::system::system_error& e)
  {
    ROS_ERROR_STREAM("Error: " << e.what());
    return false;
  }
}

const bool GreenSeeker::disconnect()
{
  serial_.reset();
}

const bool GreenSeeker::read()
{
  std::string line = "";

  try
  {
    // read serial
    line = serial_->readLine();
  }
  catch (boost::system::system_error& e)
  {
    ROS_ERROR_STREAM("Error: " << e.what());
    return false;
  }

  // create a packet message
  greenseeker_msgs::GreenSeekerPacket msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  msg.red_rfl = -1.0;
  msg.nir_rfl = -1.0;

  // decode packet
  // split by comma
  std::string delimiter = ",";

  // how many columns
  size_t cols = std::count(line.begin(), line.end(), ',');

  if (cols >= 5)
  {
    // time
    msg.time = std::strtoull(line.substr(0, line.find(delimiter)).c_str(), NULL, 0);
    line.erase(0, line.find(delimiter) + delimiter.length());
    // plot
    msg.plot = std::strtoll(line.substr(0, line.find(delimiter)).c_str(), NULL, 0);
    line.erase(0, line.find(delimiter) + delimiter.length());
    // count
    msg.count = std::strtoll(line.substr(0, line.find(delimiter)).c_str(), NULL, 0);
    line.erase(0, line.find(delimiter) + delimiter.length());
    // ndvi
    msg.ndvi = std::stod(line.substr(0, line.find(delimiter)));
    line.erase(0, line.find(delimiter) + delimiter.length());
    // irvi
    msg.irvi = std::stod(line.substr(0, line.find(delimiter)));
    line.erase(0, line.find(delimiter) + delimiter.length());
  }

  if (cols >= 7)
  {
    // red_rfl
    msg.red_rfl = std::stod(line.substr(0, line.find(delimiter)));
    line.erase(0, line.find(delimiter) + delimiter.length());
    // nir_rfl
    msg.nir_rfl = std::stod(line.substr(0, line.find(delimiter)));
    line.erase(0, line.find(delimiter) + delimiter.length());
  }

  // publish message
  packet_pub_.publish(msg);

  return true;
}

void GreenSeeker::read_timer_cb(const ros::TimerEvent& event)
{
  // read and publish packet
  read();
  ROS_DEBUG_STREAM("GreenSeeker::read_timer_cb() - read() called resetting timer");

  // start polling timer again
  read_poll_timer_ = pnh_.createTimer(loop_rate_.expectedCycleTime(), &GreenSeeker::read_timer_cb, this, true);
}

}  // namespace greenseeker_driver
