#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "dynamixel.h"

std::string DEVICE_NAME;
int         BAUDRATE;

int main(int argc, char **argv) {
  ros::init(argc, argv, "connector_control");
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  if (!nh_p.getParam("DEVICE_NAME", DEVICE_NAME)) DEVICE_NAME = "/dev/ttyUSB0";
  if (!nh_p.getParam("BAUDRATE",    BAUDRATE)   ) BAUDRATE    =  57600;
  
  DynamixelComunicator dyn_comm(DEVICE_NAME.c_str(), BAUDRATE);
  dyn_comm.OpenPort();
}
