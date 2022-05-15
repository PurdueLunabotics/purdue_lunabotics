#include <Encoder.h>
#include <IntervalTimer.h>

//Max Speed of wheel = 0.54 m/s

IntervalTimer ctrl_timer;
Encoder enc(17, 18);

volatile int prev_pos = 0;
const float dt = 0.6;
const float dt_ms = 600000;
volatile float goal = 0.5;
const float f = 435;
const int dir_pin = 2;
const int drive_pin = 1;
volatile float prev_err = 0;
void setup() {
    Serial.begin(9600);
    pinMode(dir_pin, OUTPUT);
    pinMode(drive_pin, OUTPUT);
    ctrl_timer.begin(ctrl_loop, dt_ms);
}

void ctrl_loop(void) {
    int pos = enc.read();
    float curr_vel = (pos - prev_pos) / dt;
    curr_vel /= f;
    Serial.println(curr_vel);

    float goal_vel = calc_pid(goal, curr_vel);
    if(goal_vel > 1) {
        goal_vel = 1;
    } else if(goal_vel < -1) {
        goal_vel = -1;
    }
    digitalWrite(dir_pin, goal_vel > 0 ? LOW : HIGH);
    analogWrite(drive_pin, abs(goal_vel) * 255);

    prev_pos = pos;
    //delay(dt * 1000);
}

float calc_pid(float goal, float curr) {
    float err = goal - curr;
    float p = .2;
    float vel = goal;
    vel += p * err;
    float d = 0.02;
    vel += d * (err - prev_err);
    prev_err = err;
    return vel;
}

void run_control_loop(){
    int pos = enc.read();
    float curr_vel = (pos - prev_pos) / dt;
    curr_vel /= f;
    //Serial.println(curr_vel);

    float goal_vel = calc_pid(goal, curr_vel);
    if(goal_vel > 1) {
        goal_vel = 1;
    } else if(goal_vel < -1) {
        goal_vel = -1;
    }
    digitalWrite(dir_pin, goal_vel > 0 ? LOW : HIGH);
    analogWrite(drive_pin, abs(goal_vel) * 255);

    prev_pos = pos;
    //delay(dt * 1000);
} // 1.67 Hz

void loop() {
  goal = 0;
  // analogWrite(drive_pin, 0);
//  for(float i = 0; i <= 1; i += 0.1) {
//    goal = i;
//    delay(1000);
//  }
//  for(float i = 1; i >= 0; i -= 0.1) {
//    goal = i;
//    delay(1000);
//  }
//    digitalWrite(dir_pin, LOW);
////    analogWrite(drive_pin, 255);
//    for(int i = 0; i <= 255; i += 10) {
//      analogWrite(drive_pin, i);
//      int pos = enc.read();
//      Serial.println((pos-prev_pos)/dt);
//      prev_pos = pos;
//      delay(dt * 1000);
//    }
//    for(int i = 255; i >= 0; i -= 10) {
//      analogWrite(drive_pin, i);
//      int pos = enc.read();
//      Serial.println((pos-prev_pos)/dt);
//      prev_pos = pos;
//      delay(dt * 1000);
//    }
//    delay(1000);
//    int loop_count = 100;
//    float vel_sum = 0;
//    for(int i = 0; i < loop_count; ++i) {
//        int pos = enc.read();
//        vel_sum += (pos - prev_pos) / dt;
//        prev_pos = pos;
//        delay(dt * 1000);
//    }
//    Serial.println(vel_sum / loop_count);
}