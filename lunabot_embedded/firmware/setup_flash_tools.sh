#!/bin/sh

## Installs compilation tools
# get arduino-cli executable (replace with Linux_x86 if necessary)
wget https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_ARM64.tar.gz
tar -xvf arduino-cli_latest_Linux_ARM64.tar.gz
./arduino-cli config init
python3 update_board_manager.py 
./arduino-cli core install teensy:avr

## Installs teensy flasher
