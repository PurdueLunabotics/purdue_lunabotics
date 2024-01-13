#!/bin/sh

## Installs compilation tools
arduino-cli config init
python3 update_board_manager.py 
arduino-cli core install teensy:avr
arduino-cli lib install HX711-master
arduino-cli lib install ADS1115-Lite

## Installs teensy flasher
sudo apt-get install libusb-dev
git clone https://github.com/PaulStoffregen/teensy_loader_cli
cd teensy_loader_cli
make
sudo make install
cd ..
rm -rf teensy_loader_cli
