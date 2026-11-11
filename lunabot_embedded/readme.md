# lunabot_embedded

## Jetson Teensy C++ Driver

- code in `src` folder
- see `RobotEffort.msg`
- see `RobotSensors.msg`

## Teensy Firmware

code in `firmware/`

### Quick start

1. Setup development environment 

```bash
cd lunabot_embedded/firmware
./setup_teensy_toolchain.sh
```

2. Setup CMake (run this in the firmware folder)
```bash
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=./teensyduino/toolchain.cmake
```

3. Compile (no hardware needed) + Upload

```bash
cmake --build build
cmake --build build --target flash_teensy_main # This also rebuilds so you don't need the above command if you are going to flash
```
