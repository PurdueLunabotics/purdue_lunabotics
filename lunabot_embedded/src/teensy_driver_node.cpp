#include <ros/ros.h>
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

#define BUF_SIZE 64

uint8_t buf[BUF_SIZE];
RobotState state = RobotState_init_zero;
RobotEffort effort = RobotEffort_init_zero;

void recv() {
    int status;
    /* Create a stream that reads from the buffer. */
    pb_istream_t stream = pb_istream_from_buffer(buf, BUF_SIZE);

    /* Now we are ready to decode the message. */
    status = pb_decode(&stream, RobotState_fields, &state);
    std::cout << state.act_right_curr << std::endl;
    std::cout << state.drive_right_curr << std::endl;
    std::cout << state.drive_left_curr << std::endl;
    std::cout << state.lead_screw_curr << std::endl;
    std::cout << state.dep_curr << std::endl;
    std::cout << state.exc_curr << std::endl;

    std::cout << state.act_ang << std::endl;
    std::cout << state.drive_right_ang << std::endl;
    std::cout << state.drive_left_ang << std::endl;
    std::cout << state.lead_screw_ang << std::endl;
    std::cout << state.dep_ang << std::endl;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "driver_node");
    ros::NodeHandle nh;

    int i, r, num;

    r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
    if (r <= 0) {
        printf("no rawhid device found\n");
        return -1;
    }
    printf("found rawhid device\n");

    ros::Rate rate(100);

    while (ros::ok()) {
        // check if any Raw HID packet has arrived
        num = rawhid_recv(0, buf, BUF_SIZE, 0);
        if (num < 0) {
            printf("\nerror reading, device went offline\n");
            break;
        }

        if (num > 0) {
            printf("\nrecv %d bytes:\n", num);
            recv();
            for (i = 0; i < num; i++) {
                printf("%02X ", buf[i] & 255);
                if (i % 16 == 15 && i < num - 1)
                    printf("\n");
            }
            printf("\n");
        }
        rate.sleep();
    }
    rawhid_close(0);
}
