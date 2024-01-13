# lunabot_embedded

## Jetson Teensy C++ Driver

- code in `src` folder
- see `RobotEffort.msg`
- see `RobotSensors.msg`

## Teensy Firmware

code in `firmware/teensy_main/`

### Prerequisites
- [Install arduino-cli compilation toolchain](https://arduino.github.io/arduino-cli/0.35/installation/#use-the-install-script)

### Quick start

1. Setup development environment 

```
roscd lunabot_embedded/firmware
./setup_flash_tools.sh
```

2. Add aliases to simplify compilation (in `.bashrc` or `.zshrc`):
```
alias teensy_compile="arduino-cli compile --build-path build --libraries lib --fqbn teensy:avr:teensy41:usb=rawhid teensy_main"
alias teensy_upload="teensy_loader_cli --mcu=TEENSY41 -wv build/teensy_main.ino.hex"
```

3. Compile (no hardware needed) + Upload

```
roscd lunabot_embedded/firmware/teensy_main
teensy_compile
teensy_upload
```

## Regenerate Proto files

```
pip3 install nanopb
roscd lunabot_embedded/firmware/teensy_main/lib/lunabot_drivers
python3 -m nanopb.generator.nanopb_generator -L quote RobotMsgs.proto
```
