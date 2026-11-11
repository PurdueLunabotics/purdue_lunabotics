#!/bin/sh
## Installs teensy flasher
sudo apt-get install libusb-dev gcc-arm-none-eabi
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
