#include <ctime>
#include <iostream>
#include <lunabot_embedded/sensor_proc.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>

extern "C" {
#include "RobotMsgs.pb.h"
#include "hid.h"
#include "pb_decode.h"
#include "pb_encode.h"
}

using namespace std;

RobotSensors state = RobotSensors_init_zero;
RobotEffort effort = RobotEffort_init_zero;

int main(int argc, char **argv) {

  state.lead_screw_curr = 65535;
  state.act_right_curr = 65535;
  state.dep_curr = 65535;
  state.exc_curr = 65535;
  state.drive_left_curr = 65535;
  state.drive_right_curr = 65535;

  state.drive_left_ang = DEG2RAD(360.0);
  state.drive_right_ang = DEG2RAD(360.0);
  state.exc_ang = DEG2RAD(360.0);
  state.uwb_dist_0 = 10.0;
  state.uwb_dist_1 = 10.0;
  state.uwb_dist_2 = 10.0;

  effort.lin_act = 126;
  effort.left_drive = 126;
  effort.right_drive = 126;
  effort.excavate = 126;
  effort.deposit = 126;
  size_t sensors_size;
  size_t effort_size;
  bool res = pb_get_encoded_size(&sensors_size, RobotSensors_fields, &state);

  cout << "sensors nanopb msg encoding success: " << res << endl;
  res = pb_get_encoded_size(&effort_size, RobotEffort_fields, &effort);
  cout << "effort nanopb msg encoding success: " << res << endl;

  cout << "sensors nanopb msg size (bytes): " << sensors_size << endl;
  cout << "effort nanopb msg size (bytes): " << effort_size << endl;
}
