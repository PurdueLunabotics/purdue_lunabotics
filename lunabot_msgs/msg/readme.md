# List of things in messages

## RobotSensors.msg

| Message Name      | Data Type | Units     | Range       | Notes                         |
|-------------------|-----------|-----------|-------------|-------------------------------|
| act_right_curr    | float32   | Amps      |             |                               |
| dep_curr          | float32   | Amps      |             |                               |
| exc_curr          | float32   | Amps      |             |                               |
| drive_left_curr   | float32   | Amps      |             |                               |
| drive_right_curr  | float32   | Amps      |             |                               |
| drive_left_ang    | float32   | Radians   | -inf to inf | Old encoders bounded 0 to 2pi |
| drive_right_ang   | float32   | Radians   | -inf to inf | Old encoders bounded 0 to 2pi |
| drive_left_vel    | float32   |           |             |                               |
| drive_right_vel   | float32   |           |             |                               |
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
