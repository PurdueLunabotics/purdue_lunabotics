# lunabot_embedded

## Jetson Teensy C++ Driver

- code in `src` folder
- see `RobotEffort.msg`
- see `RobotSensors.msg`

## Teensy Firmware

code in `firmware/`

### Quick start

1. Setup development environment 

```
cd lunabot_embedded/firmware
./setup_teensy_toolchain.sh
```

2. Setup CMake (run this in the firmware folder)
```
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=./teensyduino/toolchain.cmake
```

3. Compile (no hardware needed) + Upload

```
cmake -B build --build
cmake --build build --target flash_teensy_main
```

## Regenerate Proto files

```
pip3 install nanopb
pip3 install grpcio-tools
cd lunabot_embedded/firmware/teensy_main/lib/lunabot_drivers
python3 -m nanopb.generator.nanopb_generator -L quote RobotMsgs.proto
```
