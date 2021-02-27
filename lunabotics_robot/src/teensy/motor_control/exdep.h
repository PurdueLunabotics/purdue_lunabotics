#ifndef EXDEP_H
#define EXDEP_H

#include <ros.h>
#include <std_msgs/Byte.h>

namespace exdep {
  void init();
  void moveExdepMotor(int ind, int command);
  void actuateExdep(const std_msgs::Byte& command);  
}

#endif
