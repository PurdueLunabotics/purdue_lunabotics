# lunabot_control

This package includes control code that allows for commanding robot joints through autonomy or manual commands and ensures realtime execution through the sim or a real robot.

### Controllers

Actuation control

- lead-screw endstop via current 
  - inputs:
    - dir: integer 0,1 (down,up)
    - speed: float 0-1 (zero-max speed)
  - outputs: int8 effort with safety limits from current hardstops
- linear actuator setpoints via encoders

Drivetrain control

- wheel velocity PID
- MPC for trajectory following

Deposition control

- bin homing controller
- automated dumping with current feedback

Excavation 
- "leaky bucket" controller using current feedback. when bucket is full, retract excavation controller
