#include "StepperLib.hpp"

// HOW TO SETUP MOTOR
// set ID using rotating dial
// set DIP switch 1 to on to set baud rate
// set DIP switch 3 to on on the final motor to enable resistor

// USER MANUAL HERE:
// https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=3354/User%20Manual%20Of%20iSV2-RS.pdf

// TODO RJN - deal with alarms and error codes

struct ISV2Addrs {                   // addrs of various motor params
  uint16_t ControlMode = 0x0003; // position or velocity control
  uint16_t Reset = 0x0033;       // reset alarms or the entire motor
  // there is a bunch of PID stuff here - probably easier to tune with motionstudio
  uint16_t InternalEnable = 0x0405; // use to enable servo (must set before sending movement commands)
  uint16_t RS485_Mode = 0x053B;     // *SET TO 4* to enable Serial_8N1 (see manual page 42)
  uint16_t RS485_BAUD = 0x053D;     // USE SWITCHES INSTEAD - BAUD SHOULD BE 19200
  uint16_t RS485_ID = 0x053F;       // USE DIAL INSTEAD - different for each motor
  // there is some stuff with regen breaking here - could be interesting for power saving

  // TIME FOR THE STUFF WE WANT TO READ:
  // software version stuff
  uint16_t Read_ErrorCode = 0x0B03;      // error code
  uint16_t Read_MotorState = 0x0B05;     // see page 53
  uint16_t Read_RawVelocity = 0x0B06;    // rpm - unfilitered
  uint16_t Read_Torque = 0x0B07;         // % of rated
  uint16_t Read_Current = 0x0B08;        // 0.01A
  uint16_t Read_Velocity = 0x0B09;       // rpm
  uint16_t Read_Voltage = 0x0B0A;        // V
  uint16_t Read_Temperature = 0x0B0B;    // deg C
  uint16_t Read_OverLoadRatio = 0x0B0F;  // %
  uint16_t Read_RegenLoadRatio = 0x0B10; // %
  // digital input stuff
  // motor position stuff
  uint16_t Read_MotorPosition = 0x602C; // unit: pulses - 32 bit! (low is +1)

  uint16_t SpeedMode = 0x0301;
  uint16_t Speed = 0x0309;
  uint16_t Acceleration = 0x0319;
  uint16_t Deceleration = 0x031B;

} ISV2Addrs;

struct BLD305SAddrs {                   // addrs of various motor params
  //uint16_t ControlMode = 0xFFFF; // Not applicable
  uint16_t AutoReset = 0x0106;       // whether motor auto resets
  uint16_t InternalEnable = 0x0136; 
  //uint16_t RS485_Mode = 0xFFFF;     // Not applicable
  uint16_t RS485_BAUD = 0x00F6;     // USE SWITCHES INSTEAD - BAUD SHOULD BE 19200
  uint16_t RS485_ID = 0x00A6;       // Spec sheet refers to this as driver address

  uint16_t ErrorCode = 0x0076;      // 1-Overcurrent, 2-Over temperature, 3-Overpressure, 4-Undervoltage, 5-Sensor abnormality, 6-Overspeed, 8-Stalled, 9-Peak current
  uint16_t MotorState = 0x0066;     // 0 - Stop, 1 - Forward rotation, 2 - Reverse rotation, 3 - Stop brake
  //uint16_t Read_Torque = ;         // % of rated
  uint16_t Read_Current = 0x00B6;        // 1 /25 A
  uint16_t Read_Speed = 0x005F;       // rpm, does not differentiate between filtered or unfiltered
  uint16_t Read_Voltage = 0x00C6;        // V
  uint16_t Read_Temperature = 0x0096;    // deg C
  //uint16_t Read_OverLoadRatio = 0xFFFF;  // %
  //uint16_t Read_RegenLoadRatio = 0xFFFF; // %
  // digital input stuff
  // motor position stuff
  //uint16_t Read_MotorPosition = 0xFFFF; // unit: pulses - 32 bit! (low is +1)
  uint16_t Set_Speed = 0X0056; // 0 to 4000 in rpm
  uint16_t Acceleration = 0x00E6; // in units of time. No differentiation betwen acceleration and deceleration
  uint16_t Save_State = 0x80FF; // write save code to save state

} BLD305SAddrs;

struct ISV2ControlCmds {
  uint8_t ESTOP = 0x40;
  uint8_t ENABLE = 0x83;
  uint16_t VelocityMode = 0x0001;
  uint16_t DigitalSpeedMode = 0x0001;
  uint16_t Save = 0x2211;
  uint16_t ResetAlarm = 0x1111;
} ISV2ControlCmds;

struct BLD305SControlCmds {
  uint16_t ESTOP = 0x0000;
  uint16_t ENABLE = 0x0001;
  uint16_t Save = 0x55AA;
  uint16_t Reset = 0x0000;
} BLD305SControlCmds;


// Create a StepperMotor object
// MotorID and baudrate are required
// Optional default values for speed, acceleration, deceleration
// speed in rpm
// acceleration and deceleration in ms/1000 rpm
StepperMotor::StepperMotor(uint8_t MotorID, StepperLibMotorType motor_type, uint16_t def_acceleration, uint16_t def_deceleration) {
  this->MotorID = MotorID;
  this->motor_type = motor_type;
  this->def_acceleration = def_acceleration;
  this->def_deceleration = def_deceleration;
}

void StepperMotor::begin() {
  modbus_begin();
  if (motor_type == ISV2) {
    write_register(ISV2Addrs.InternalEnable, ISV2ControlCmds.ENABLE);

    write_register(ISV2Addrs.ControlMode, ISV2ControlCmds.VelocityMode);
    write_register(ISV2Addrs.SpeedMode, ISV2ControlCmds.DigitalSpeedMode);
    write_register(ISV2Addrs.Reset, ISV2ControlCmds.Save);

    write_register(ISV2Addrs.Acceleration, def_acceleration);
    write_register(ISV2Addrs.Deceleration, def_deceleration);
    write_register(ISV2Addrs.Speed, 0);
  }
  else if (motor_type == BLD305S) {
    write_register(BLD305SAddrs.InternalEnable, BLD305SControlCmds.ENABLE);
    write_register(BLD305SAddrs.Save_State, BLD305SControlCmds.Save);
    write_register(BLD305SAddrs.Acceleration, def_acceleration); // no differentiation between acceleration and deceleration
    write_register(BLD305SAddrs.Set_Speed, 0);
  }
}

// emergency stops motor
void StepperMotor::write_estop() {
  if (motor_type == ISV2) {
    move_at_speed(0);
  }
  else if (motor_type == BLD305S) {
    write_register(BLD305SAddrs.MotorState, 3);
  }
  //Note: forBLD305S, you can set the motor state to stop.
}

// clears errors
void StepperMotor::clear_errors() {
  if (motor_type == ISV2) {
    write_register(ISV2Addrs.Reset, ISV2ControlCmds.ResetAlarm);
  }
  else if (motor_type == BLD305S) {
    write_register(BLD305SAddrs.ErrorCode, 0);
  }
}

// move the motor at constant velocity, using default values for acceleration and deceleration unless specified
// speed in rpm
//NEEDS to be signed since negative and positive values can be passed in. doesn't change underlying bits stored, 
// but changes how c code interprets the value in arthmetic operations.
void StepperMotor::move_at_speed(int16_t speed) {
  if (motor_type == ISV2) {
    write_register(ISV2Addrs.Speed, speed);
  }
  else if (motor_type == BLD305S) {
    if (speed < 0) {
      write_register(BLD305SAddrs.MotorState, 2);
    }
    else {
      write_register(BLD305SAddrs.MotorState, 1);
    }
    write_register(BLD305SAddrs.Set_Speed, abs(speed));
  }
}

void StepperMotor::write_register(uint16_t addr, uint16_t data) {
  modbus_write_register(MotorID, addr, data);
}

int StepperMotor::read_register(uint16_t addr, uint16_t num_to_read) {
  return modbus_read_register(MotorID, addr, num_to_read);
}

// rpm - unfilitered
int StepperMotor::read_raw_velocity() {
  if (motor_type == ISV2) {
    return (int16_t)read_register(ISV2Addrs.Read_RawVelocity);
  }
  else if (motor_type == BLD305S) {
    if (read_register(BLD305SAddrs.MotorState) == 2) {
      return -1 * (int16_t)read_register(BLD305SAddrs.Read_Speed);
    }
    else {
      return (int16_t)read_register(BLD305SAddrs.Read_Speed);
    }
  }
  return -1;
}

// rpm
int StepperMotor::read_velocity() {
  if (motor_type == ISV2) {
    return (int16_t)read_register(ISV2Addrs.Read_Velocity);
  }
  else if (motor_type == BLD305S) {
    if (read_register(BLD305SAddrs.MotorState) == 2) {
      return -1 * (int16_t)read_register(BLD305SAddrs.Read_Speed);
    }
    else {
      return (int16_t)read_register(BLD305SAddrs.Read_Speed);
    }
  }
  return -1;
  
}

// % of rated
int StepperMotor::read_torque() {
  if (motor_type == ISV2) {
    return (int16_t)read_register(ISV2Addrs.Read_Torque);
  }
  else if (motor_type == BLD305S) {
    return -1;
  }
  return -1;
  
}

// amps
float StepperMotor::read_current() {
  if (motor_type == ISV2) {
    int x = (int16_t)read_register(ISV2Addrs.Read_Current);
    if (x == -1) {
      return -1;
    }
    return x * 0.1;
  }
  else if (motor_type == BLD305S) {
    int x = (int16_t)read_register(BLD305SAddrs.Read_Current);
    if (x == -1) {
      return -1;
    }
    return x / 25.0;
  }
  return -1;
}

// volts
int StepperMotor::read_voltage() {
  if (motor_type == ISV2) {
    return read_register(ISV2Addrs.Read_Voltage);
  }
  else if (motor_type == BLD305S) {
    return read_register(BLD305SAddrs.Read_Voltage);
  }
  return -1;
  
}

// deg C
int StepperMotor::read_temperature() {
  if (motor_type == ISV2) {
    return read_register(ISV2Addrs.Read_Temperature);
  }
  else if (motor_type == BLD305S) {
    return read_register(BLD305SAddrs.Read_Temperature);
  }
  return -1;
  
}

// %
int StepperMotor::read_over_load_ratio() {
  if (motor_type == ISV2) {
    return read_register(ISV2Addrs.Read_OverLoadRatio);
  }
  else if (motor_type == BLD305S) {
    return -1;
  }
  return -1;
  
}

// %
int StepperMotor::read_regen_load_ratio() {
  if (motor_type == ISV2) {
    return read_register(ISV2Addrs.Read_RegenLoadRatio);
  }
  else if (motor_type == BLD305S) {
    return -1;
  }
  return -1;
  
}

// pulses
int StepperMotor::read_motor_position_raw() {
  if (motor_type == ISV2) {
    return read_register(ISV2Addrs.Read_MotorPosition, 2); // this is a 2 byte value
  }
  else if (motor_type == BLD305S) {
    return -1;
  }
  return -1;
}

// radians
float StepperMotor::read_motor_position_radians() {
  if (motor_type == ISV2) {
    int x = read_motor_position_raw();
    if (x == -1) {
      return -1;
    }
    return x / 10000.0 * 2 * PI; // 2500 pulses per rotation, * 2 * PI
  }
  else if (motor_type == BLD305S) {
    return -1;
  }
  return -1;
  
}

// print the motor state from the state register
void StepperMotor::print_motor_state() {
  if (motor_type == ISV2) {
    switch (read_register(ISV2Addrs.Read_MotorState)) {
    case 0:
      Serial.println("Motor State: Ready");
      break;
    case 1:
      Serial.println("Motor State: Run");
      break;
    case 2:
      Serial.println("Motor State: Error");
      break;
    case 3:
      Serial.println("Motor State: Home OK");
      break;
    case 4:
      Serial.println("Motor State: at pos");
      break;
    case 5:
      Serial.println("Motor State: at speed");
      break;
    default:
      Serial.println("Motor State: ???");
      break;
    }
  }
  else if (motor_type == BLD305S) {
    switch (read_register(BLD305SAddrs.Read_MotorState)) {
    case 0:
      Serial.println("Motor State: Stop");
      break;
    case 1:
      Serial.println("Motor State: Run Forward");
      break;
    case 2:
      Serial.println("Motor State: Run Backwards");
      break;
    case 3:
      Serial.println("Motor State: Brake Stop");
      break;
    default:
      Serial.println("Motor State: ???");
      break;
    }
    switch (read_register(BLD305SAddrs.ErrorCode)) {
    case 0:
      Serial.println("Error State: Nothing");
      break;
    case 1:
      Serial.println("Error State: Overcurrent");
      break;
    case 2:
      Serial.println("Error State: Over temperature");
      break;
    case 3:
      Serial.println("Error State: Overpressure");
      break;
    case 4:
      Serial.println("Error State: Undervoltage");
      break;
    case 5:
      Serial.println("Error State: Sensor abnormality");
      break;
    case 6:
      Serial.println("Error State: Overspeed");
      break;
    case 7:
      Serial.println("Error State: ????");
      break;
    case 8:
      Serial.println("Error State: Stalled");
      break;
    case 9:
      Serial.println("Error State: Peak current");
      break;
    default:
      Serial.println("Error State: ???");
      break;
    }
  }
  else {
    Serial.println("Invalid motor type");
  }
}
