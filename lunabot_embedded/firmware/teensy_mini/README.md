alias teensy_compile="arduino-cli compile --build-path build --libraries lib --fqbn teensy:avr:teensy41:usb=rawhid teensy_mini.ino"
alias teensy_upload="teensy_loader_cli --mcu=TEENSY41 -wv build/teensy_mini.ino.hex"

