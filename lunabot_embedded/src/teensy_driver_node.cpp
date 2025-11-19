#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <ratio>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include <ctime>
#include <sys/ioctl.h>
#include <termios.h>

#include "std_msgs/msg/int32.hpp"

#include "lunabot_embedded/sensor_proc.h"
#include "lunabot_msgs/msg/robot_effort.hpp"
#include "lunabot_msgs/msg/robot_sensors.hpp"

extern "C" {
#include "RobotMsgs.pb.h"
#include "hid.h"
#include "pb_decode.h"
#include "pb_encode.h"
}

using namespace std;

#define BUF_SIZE 64

class TeensyDriverNode : public rclcpp::Node {
  private:
    rclcpp::Subscription<lunabot_msgs::msg::RobotEffort>::SharedPtr effort_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr color_sub;
    rclcpp::Publisher<lunabot_msgs::msg::RobotSensors>::SharedPtr state_pub;

    rclcpp::TimerBase::SharedPtr state_timer;
    rclcpp::TimerBase::SharedPtr timer;

    uint8_t buf[BUF_SIZE];

    RobotSensors state;
    RobotEffort effort;

    int r;
    int num_read_fails;

  public:
    TeensyDriverNode() : Node("teensy_driver_node"), state(), effort() {
      effort_sub = this->create_subscription<lunabot_msgs::msg::RobotEffort>("/effort", 10, std::bind(&TeensyDriverNode::effort_cb, this, placeholders::_1));
      color_sub = this->create_subscription<std_msgs::msg::Int32>("/led_color", 10, std::bind(&TeensyDriverNode::color_cb, this, placeholders::_1));
      state_pub = this->create_publisher<lunabot_msgs::msg::RobotSensors>("/sensors", 10);

      num_read_fails = 0;

      r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
      if (r <= 0) {
        RCLCPP_WARN(this->get_logger(), "no rawhid device found\n");
        throw -1;
      }
      printf("found rawhid device\n");

      state_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TeensyDriverNode::publish, this));
      timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TeensyDriverNode::loop_cb, this));
    }

    ~TeensyDriverNode() {
      rawhid_close(0);
    }

  private:
    void loop_cb() {
      int num = rawhid_recv(0, buf, BUF_SIZE, 10);
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
        this->recv();
      }
    }

    void recv() {
      /* Create a stream that reads from the buffer. */
      pb_istream_t stream = pb_istream_from_buffer(buf, sizeof(buf));
      /* Now we are ready to decode the message. */
      pb_decode(&stream, RobotSensors_fields, &state); // This will probably not work
      lunabot_msgs::msg::RobotSensors state_msg;

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

      state_pub->publish(state_msg);
    }

    void effort_cb(const lunabot_msgs::msg::RobotEffort &msg) {
      effort.lin_act = msg.lin_act;
      effort.left_drive = msg.left_drive;
      effort.right_drive = msg.right_drive;
      effort.excavate = msg.excavate;
      effort.deposit = msg.deposit;
      effort.should_reset = msg.should_reset;
    }

    void color_cb(const std_msgs::msg::Int32 &msg) { effort.led_color = msg.data; }

    void publish() {
      memset(buf, 0, sizeof(buf));
      pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
      pb_encode(&stream, RobotEffort_fields, &effort);
      rawhid_send(0, buf, 64, 0);
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyDriverNode>());
  rclcpp::shutdown();
  return 0;
}
