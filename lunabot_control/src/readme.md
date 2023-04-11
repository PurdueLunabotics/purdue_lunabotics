### Controllers

Actuation control

- lead-screw endstop via current 
  - inputs:
    - dir: integer 0,1 (down,up)
    - speed: float 0-1 (zero-max speed)
  - outputs: int8 effort with safety limits from current hardstops
- linear actuator setpoints via encoders

Drivetrain control

- wheel velocity
- MPC for trajectory following

Deposition control

- bin homing controller
- automated dumping with current feedback

Excavation 
