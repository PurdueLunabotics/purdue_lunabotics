# lunabot_embedded



## Jetson Code

- code in `src` folder
- see `RobotEffort.msg`
- see `RobotState.msg`

## Teensy Code

- code in `firmware/teensy_main/`



##  Regenerate Proto files

```
pip3 install nanopb
python3 -m nanopb.generator.nanopb_generator -L quote RobotMsgs.proto
```
