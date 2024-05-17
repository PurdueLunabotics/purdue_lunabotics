/**
*  Arduino Library for Texas Instruments ADS1119 - 16-Bit 4ch Analog-to-Digital Converter
*  
*  @author Oktawian Chojnacki <oktawian@elowro.com>
*  https://www.elowro.com
*
*/

/**
 * The MIT License
 *
 * Copyright 2020 Oktawian Chojnacki <oktawian@elowro.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef ADS1119_h
#define ADS1119_h

#include "Arduino.h"
#include <Wire.h>

/** default I2C address **/
#define ADS1119_DEFAULT_ADDRESS (0x40) // 1000000 (A0+A1=GND)

#define ADS1119_RANGE ((uint16_t)32767) 
#define ADS1119_INTERNAL_REFERENCE_VOLTAGE ((float)2.048) 

#define ADS1119_MUX_P_AIN0_N_AIN1 (0B000) 
#define ADS1119_MUX_P_AIN2_N_AIN3 (0B001) 
#define ADS1119_MUX_P_AIN1_N_AIN2 (0B010) 
#define ADS1119_MUX_P_AIN0_N_AGND (0B011) 
#define ADS1119_MUX_P_AIN1_N_AGND (0B100) 
#define ADS1119_MUX_P_AIN2_N_AGND (0B101) 
#define ADS1119_MUX_P_AIN3_N_AGND (0B110) 
#define ADS1119_MUX_SHORTED_H_AVDD (0B111) 

enum struct ADS1119MuxConfiguration: uint8_t {
	positiveAIN0negativeAIN1 = ADS1119_MUX_P_AIN0_N_AIN1,
	positiveAIN2negativeAIN3 = ADS1119_MUX_P_AIN2_N_AIN3, 
	positiveAIN1negativeAIN2 = ADS1119_MUX_P_AIN1_N_AIN2,
	positiveAIN0negativeAGND = ADS1119_MUX_P_AIN0_N_AGND, 
	positiveAIN1negativeAGND = ADS1119_MUX_P_AIN1_N_AGND, 
	positiveAIN2negativeAGND = ADS1119_MUX_P_AIN2_N_AGND, 
	positiveAIN3negativeAGND = ADS1119_MUX_P_AIN3_N_AGND,
	shortedToHalvedAVDD = ADS1119_MUX_SHORTED_H_AVDD 
};

enum struct ADS1119RegisterToRead: uint8_t {
	configuration = 0B0,
	status = 0B1
};

/**
	ADS1119Configuration
	@author Oktawian Chojnacki <oktawian@elowro.com>
*/
struct ADS1119Configuration
{
	enum struct Gain: uint8_t {
		one = 0B0,
		four = 0B1
	};

	enum struct DataRate: uint8_t {
		sps20 = 0B00,
		sps90 = 0B01,
		sps330 = 0B10,
		sps1000 = 0B11
	};

	enum struct ConversionMode: uint8_t {
		singleShot = 0B0,
		continuous = 0B1
	};

	enum struct VoltageReferenceSource: uint8_t {
		internal = 0B0,
		external = 0B1
	};

	// Bitmask
	ADS1119MuxConfiguration mux;

	// 0B0: Gain=1, 0B1: Gain=4
	Gain gain; 

	// 0B00: 20SP, 0B01: 90SPS, 0B10: 330SPS, 0B11: 1000SPS
	DataRate dataRate; 

	// 0B0: Single-shot conversion mode, 0B1: Continuous conversion mod
	ConversionMode conversionMode; 

	// 0B0: Internal 2.048-V reference selected (default), 0B1: External reference selected using the REFP and REFN inputs
	VoltageReferenceSource voltageReference; 

	// This is needed to convert bytes to volts
	float externalReferenceVoltage = 0;
};

/**
 * ADS1119 IC
 * @author Oktawian Chojnacki <oktawian@elowro.com>
 */
class ADS1119
{
public:
	ADS1119(uint8_t address = ADS1119_DEFAULT_ADDRESS);

	/**
	Begin using the library instance.
	*/
	void begin(TwoWire *theWire = &Wire);

	/**
	Will perform conversion and save it as internal offset.
	Make sure the input being measure is at VREF!
	*/
	float performOffsetCalibration(ADS1119Configuration config);

	/**
	This command will save the configuration and then attempt to read two bytes, then convert it to voltage.
	*/
	float readVoltage(ADS1119Configuration config);

	/**
	This command will save the configuration and then attempt to read two bytes.
	*/
	uint16_t readTwoBytes(ADS1119Configuration config);
	
	/**
	The POWERDOWN command places the device into power-down mode. 
	This command shuts down all internal analog components, but holds all register values. 
	In case the POWERDOWN command is issued when a conversion is ongoing, the conversion completes 
	before the ADS1119 enters power-down mode. As soon as a START/SYNC command is issued, all analog
	components return to their previous states.
	*/
	bool powerDown();

	/**
	This command resets the device to the default states. No delay time is required after the RESET 
	command is latched before starting to communicate with the device as long as the timing requirements 
	(see the I2C Timing Requirements table) for the (repeated) START and STOP conditions are met.
	*/
	bool reset();

	/**
	This command reads the value of the selected register. 
	*/
	uint8_t readRegister(ADS1119RegisterToRead registerToRead);

private:
	TwoWire *_i2c;
	uint8_t _address;
	float _offset = 0.0; 

	bool commandStart();
	bool commandReadData();
	bool write(uint8_t registerValue, uint8_t value);
	bool writeByte(uint8_t value);

	float gainAsFloat(ADS1119Configuration config);
	float referenceVoltageAsFloat(ADS1119Configuration config);

	uint16_t read();
};

#endif