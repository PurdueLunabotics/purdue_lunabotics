help:
	echo "run make teensy_main_compile to compile for the main robot, and teensy_main_upload to compile for the robot"

%_compile:
	arduino-cli compile --build-path build --libraries $*/lib --fqbn teensy:avr:teensy41:usb=rawhid $*/$*.ino

%_upload:
	teensy_loader_cli --mcu=TEENSY41 -wv $*/build/$*.ino.hex

main_compile:
	arduino-cli compile --build-path build --libraries $*/lib --fqbn teensy:avr:teensy41:usb=rawhid teensy_main/teensy_main.ino

main_upload:
	teensy_loader_cli --mcu=TEENSY41 -wv teensy_main/build/teensy_main.ino.hex

mini_compile:
	arduino-cli compile --build-path build --libraries $*/lib --fqbn teensy:avr:teensy41:usb=rawhid teensy_mini/teensy_mini.ino

mini_upload:
	teensy_loader_cli --mcu=TEENSY41 -wv teensy_mini/build/teensy_mini.ino.hex