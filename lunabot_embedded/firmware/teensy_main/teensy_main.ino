// include ROS and all messages
#include <RobotMsgs.pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <robot.hpp>
#include <stdio.h>

#define TX_PERIOD 10 // ms

size_t message_length;

RobotState state = RobotState_init_zero;
RobotEffort effort = RobotEffort_init_zero;

uint8_t buffer[64];

void recv() {
    int status;
    /* Create a stream that reads from the buffer. */
    pb_istream_t stream = pb_istream_from_buffer(buffer, sizeof(buffer));

    /* Now we are ready to decode the message. */
    status = pb_decode(&stream, RobotEffort_fields, &effort);

    actuation::cb(effort.lead_screw, effort.lin_act);
    drivetrain::cb(effort.left_drive, effort.right_drive);
    deposition::cb(effort.deposit);
    excavation::cb(effort.excavate);
}

void send() {
    actuation::update(&(state.act_right_curr), &(state.lead_screw_curr));
    drivetrain::update(&(state.drive_left_curr), &(state.drive_right_curr));
    excavation::update(&(state.exc_curr));
    deposition::update(&(state.dep_curr));

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    message_length = stream.bytes_written;
    pb_encode(&stream, RobotState_fields, &state);

    buffer[62] = highByte(message_length);
    buffer[63] = lowByte(message_length);
}

void setup() {
    STMotorInterface::init_serial(ST_SERIAL, ST_BAUD_RATE);
    CurrentSensor::init_ads1115(&adc0, &adc1);

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

elapsedMillis msUntilNextSend;

void loop() {
    int n;
    n = RawHID.recv(buffer, 0); // 0 timeout = do not wait
    if (n > 0) {
        // recv();
    }

    if (msUntilNextSend > TX_PERIOD) {
        msUntilNextSend -= TX_PERIOD;
        send();
        n = RawHID.send(buffer, 0);
    }
}
