#include <ros/ros.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include <ctime>
#include <sys/ioctl.h>
#include <termios.h>

#include <std_msgs/Int32.h>

#include <lunabot_embedded/sensor_proc.h>
#include <lunabot_msgs/RobotEffort.h>
#include <lunabot_msgs/RobotSensors.h>

extern "C" {
#include "RobotMsgs.pb.h"
#include "hid.h"
#include "pb_decode.h"
#include "pb_encode.h"
}

using namespace std;

#define BUF_SIZE 64

uint8_t buf[BUF_SIZE];

RobotSensors state = RobotSensors_init_zero;
RobotEffort effort = RobotEffort_init_zero;

pb_ostream_t sizestream = {0};

void recv(ros::Publisher &pub) {
  int status;
  /* Create a stream that reads from the buffer. */
  pb_istream_t stream = pb_istream_from_buffer(buf, sizeof(buf));
  /* Now we are ready to decode the message. */
  pb_decode(&stream, RobotSensors_fields, &state);
  lunabot_msgs::RobotSensors state_msg;

  state_msg.act_right_curr = state.act_right_curr;
  state_msg.drive_right_curr = state.drive_right_curr;
  state_msg.drive_left_curr = state.drive_left_curr;
  state_msg.dep_curr = state.dep_curr;
  state_msg.exc_curr = state.exc_curr;
  state_msg.drive_left_torque = state.drive_left_torque;
  state_msg.drive_right_torque = state.drive_right_torque;
  state_msg.drive_left_vel = state.drive_left_vel;
  state_msg.drive_right_vel = state.drive_right_vel;
  state_msg.exc_torque = state.exc_torque;
  state_msg.exc_vel = state.exc_vel;

  pub.publish(state_msg);
}

void effort_cb(const lunabot_msgs::RobotEffort &msg) {
  effort.lin_act = msg.lin_act;
  effort.left_drive = msg.left_drive;
  effort.right_drive = msg.right_drive;
  effort.excavate = msg.excavate;
  effort.deposit = msg.deposit;
  effort.should_reset = msg.should_reset;
}

void color_cb(const std_msgs::Int32 &msg) { effort.led_color = msg.data; }

void publish(const ros::TimerEvent &) {
  memset(buf, 0, sizeof(buf));
  pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
  pb_encode(&stream, RobotEffort_fields, &effort);
  rawhid_send(0, buf, 64, 0);
}

int main(int argc, char **argv) {
  int num_read_fails = 0;
  int i, r, num;

  r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
  if (r <= 0) {
    printf("no rawhid device found\n");
    return -1;
  }
  printf("found rawhid device\n");

  ros::init(argc, argv, "teensy_driver_node");
  ros::NodeHandle nh;

  ros::Subscriber effort_sub = nh.subscribe("/effort", 10, &effort_cb);
  ros::Subscriber color_sub = nh.subscribe("/led_color", 10, &color_cb);
  ros::Publisher state_pub = nh.advertise<lunabot_msgs::RobotSensors>("/sensors", 10);


  ros::Rate rate(100);

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), publish);

  while (ros::ok()) {
    // check if any Raw HID packet has arrived
    ros::spinOnce();
    num = rawhid_recv(0, buf, BUF_SIZE, 10);
    if (num < 0) {
      printf("error reading. Retrying connection\n");
      num_read_fails += 1;
      if (num_read_fails >= 5) {
        printf("Sorry, too many read errors. Giving up.\n");
        printf("sleeping\n!");
        // ros::Duration(1).sleep();
        r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
        printf("Reopened rawhid, r is %d\n", r);
        num_read_fails = 0;
      }
    }

    if (num > 0) {
      recv(state_pub);
    }

    rate.sleep();
  }
  rawhid_close(0);
}
