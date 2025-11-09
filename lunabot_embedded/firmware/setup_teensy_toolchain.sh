#!/bin/sh
## Installs compilation tools
arduino-cli config init
python3 update_board_manager.py
arduino-cli core install teensy:avr
arduino-cli lib install Stepper

## Installs teensy flasher
sudo apt-get install libusb-dev
git clone https://github.com/PaulStoffregen/teensy_loader_cli
cd teensy_loader_cli
make
echo "Installing teensy_loader_cli..."
read -p "Enter the teensy_loader_cli install directory (press Enter for default: /usr/local/bin): " install_dir
if [ -z "$install_dir" ]; then
  install_dir="/usr/local/bin"
fi
sudo ln -s $(readlink -f teensy_loader_cli) $install_dir/teensy_loader_cli
cd ..
