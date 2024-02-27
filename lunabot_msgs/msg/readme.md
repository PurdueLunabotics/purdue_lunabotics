# List of things in messages

## RobotSensors.msg

| Message Name      | Data Type | Units     | Range       | Notes                         |
|-------------------|-----------|-----------|-------------|-------------------------------|
| act_right_curr    | float32   | Amps      |             | Not used for 2024             |
| dep_curr          | float32   | Amps      | -50 to 50   | Nominal 10A, Set stall 25A    |
| exc_curr          | float32   | Amps      | -50 to 50   | Nominal 10A, Set stall 25A    |
| drive_left_curr   | float32   | Amps      | -25 to 25   | Nominal 4A, Set stall 7A      |
| drive_right_curr  | float32   | Amps      | -25 to 25   | Nominal 4A, Set stall 7A      |
| drive_left_ang    | float32   | Radians   | -inf to inf | Old encoders bounded 0 to 2pi |
| drive_right_ang   | float32   | Radians   | -inf to inf | Old encoders bounded 0 to 2pi |
| drive_left_vel    | float32   | Rad/sec   | -inf to inf |                               |
| drive_right_vel   | float32   | Rad/sec   | -inf to inf |                               |
| dep_ang           | float32   | Radians   | -inf to inf | Old encoders bounded 0 to 2pi |
| uwb_dists         | float32[] | Meters    | 0 to inf    | 3 values                      |
| load_cell_weights | float32[] | Kilograms | -20 to 20   | 2 values                      |

## RobotEffort.msg

| Message Name | Data Type | Units | Range | Notes |
|--------------|-----------|-------|-------|-------|
| lin_act      | int8      |       |       |       |
| left_drive   | int8      |       |       |       |
| right_drive  | int8      |       |       |       |
| excavate     | int8      |       |       |       |
| deposit      | int8      |       |       |       |
