// include ROS and all messages
#include <RobotMsgs.pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <robot.hpp>

#define TX_PERIOD 10               // ms
#define ENC_TRANSFER_PERIOD 1000   // microsec
#define UWB_TRANSFER_PERIOD 10'000 // microsec
#define CURR_UPDATE_PERIOD 8       // ms

RobotSensors state = RobotSensors_init_zero;
RobotEffort effort = RobotEffort_init_zero;
size_t effort_msg_size;

uint8_t buffer[64];
uint8_t flags = 0;

void recv() {
  /* Create a stream that reads from the buffer. */
  pb_istream_t stream = pb_istream_from_buffer(buffer, sizeof(buffer));

  /* Now we are ready to decode the message. */
  pb_decode(&stream, RobotEffort_fields, &effort);

  actuation::cb(effort.lead_screw, effort.lin_act);
  drivetrain::cb(effort.left_drive, effort.right_drive);
  deposition::cb(effort.deposit);
  excavation::cb(effort.excavate);
}

void send() {
  actuation::update(state.act_right_curr, state.lead_screw_curr, state.act_ang,
                    state.lead_screw_ang);
  drivetrain::update(state.drive_left_curr, state.drive_right_curr, state.drive_left_ang,
                     state.drive_right_ang);
  deposition::update(state.dep_curr, state.dep_ang);

  excavation::update(state.exc_curr);

  uwb::update(state.uwb_dist_0, state.uwb_dist_1, state.uwb_dist_2);

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  pb_encode(&stream, RobotSensors_fields, &state);
}
IntervalTimer enc_timer;
IntervalTimer uwb_timer;

void setup() {
  Sabertooth_MotorCtrl::init_serial(ST_SERIAL, ST_BAUD_RATE);
  ACS711_Current_Bus::init_ads1115();
  VLH35_Angle_Bus::init();
  M5Stack_UWB_Trncvr::init();

  enc_timer.begin(VLH35_Angle_Bus::transfer, ENC_TRANSFER_PERIOD);
  uwb_timer.begin(M5Stack_UWB_Trncvr::transfer, UWB_TRANSFER_PERIOD);

  // disable timeout
  MC1.setTimeout(0);
  MC2.setTimeout(0);
  MC3.setTimeout(0);
  MC4.setTimeout(0);

  // set to fast ramp (1-10 - fast, 11-20 slow, 20-80 intermed)
  MC1.setRamping(1);
  MC2.setRamping(1);
  MC3.setRamping(1);
  MC4.setRamping(1);
}

elapsedMillis ms_until_send;
elapsedMillis ms_curr_update;

void loop() {
  int n;
  n = RawHID.recv(buffer, 0); // 0 timeout = do not wait
  if (n > 0) {
    recv();
  }

  if (ms_until_send > TX_PERIOD) {
    ms_until_send -= TX_PERIOD;
    send();
    n = RawHID.send(buffer, 0);
  }

  if (ms_curr_update > CURR_UPDATE_PERIOD) {
    ms_curr_update -= CURR_UPDATE_PERIOD;
    ACS711_Current_Bus::transfer();
  }
}
