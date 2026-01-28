/**
*  Arduino Library for Texas Instruments ADS1119 - 4ch 16-Bit Analog-to-Digital Converter
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

#include "ADS1119.h"
#include "Arduino.h"
#include <Wire.h>

ADS1119::ADS1119(uint8_t address) 
{
    _address = address;
    this->mode = ADS1119InputMode::SINGLE_ENDED;
}

void ADS1119::begin(ADS1119Configuration *config, TwoWire *theWire) 
{
    _i2c = theWire;
    _i2c->begin();
    this->config = config;
}

void ADS1119::configADCSingleEnded()
{
    this->mode = ADS1119InputMode::SINGLE_ENDED;
}

void ADS1119::configADCDifferential()
{
    this->mode = ADS1119InputMode::DIFFERENTIAL;
}

void ADS1119::selectChannel(uint8_t channel)
{
    if (this->mode == ADS1119InputMode::SINGLE_ENDED)
    {
        switch (channel)
        {
        case 0:
            this->config->mux = ADS1119MuxConfiguration::positiveAIN0negativeAGND;
            break;
        case 1:
            this->config->mux = ADS1119MuxConfiguration::positiveAIN1negativeGND;
            break;
        case 2:
            this->config->mux = ADS1119MuxConfiguration::positiveAIN2negativeAGND;
            break;
        case 3:
            this->config->mux = ADS1119MuxConfiguration::positiveAIN3negativeAGND;
            break;
        default:
            this->config->mux = ADS1119MuxConfiguration::positiveAIN0negativeAGND;
            break;
        }
    }
    else if (this->mode == ADS1119InputMode::DIFFERENTIAL)
    {
        switch (channel)
        {
        case 0:
            this->config->mux = ADS1119MuxConfiguration::positiveAIN0negativeAIN1;
            break;
        case 1:
            this->config->mux = ADS1119MuxConfiguration::positiveAIN2negativeAIN3;
            break;
        case 2:
            this->config->mux = ADS1119MuxConfiguration::positiveAIN1negativeAIN2;
            break;
        default:
            this->config->mux = ADS1119MuxConfiguration::positiveAIN0negativeAIN1;
            break;
        }
    }  
}

float ADS1119::readVoltage() 
{
    uint16_t twoBytesRead = readTwoBytes();
    if (twoBytesRead > 0x7FFF) 
    {
        twoBytesRead = 0x0;
    }
    uint16_t value = twoBytesRead - _offset;
    float gain = gainAsFloat();
    float referenceVoltage = referenceVoltageAsFloat();  
    float voltage = referenceVoltage * (float(value) / ADS1119_RANGE) * gain;

    return voltage;
}

uint16_t ADS1119::readRawValue()
{
    uint16_t twoBytesRead = readTwoBytes();
    if (twoBytesRead > 0x7FFF) 
    {
        twoBytesRead = 0x0;
    }
    return (twoBytesRead - _offset);
}

float ADS1119::performOffsetCalibration(ADS1119MuxConfiguration muxConfig) 
{
    config->mux = muxConfig;

    float totalOffset = 0;
    for (int i = 0; i <= 100; i++) {
        totalOffset += referenceVoltageAsFloat() - readVoltage();
        delay(10);
    }
    
    _offset = totalOffset / 100.0;

    return _offset;
}

float ADS1119::gainAsFloat() 
{
    return uint8_t(config->gain) == uint8_t(0B0) ? 1.0 : 4.0;
}

float ADS1119::referenceVoltageAsFloat() 
{
    return bool(config->voltageReference) ? config->externalReferenceVoltage : ADS1119_INTERNAL_REFERENCE_VOLTAGE;
}

bool ADS1119::reset() 
{
    // 8.5.3.2 RESET (0000 011x) / Page 25
    // http://www.ti.com/lit/ds/sbas925a/sbas925a.pdf
    return writeByte(0B00000110); // 0x06
}

bool ADS1119::powerDown() 
{
    // 8.5.3.4 POWERDOWN (0000 001x) / Page 25
    // http://www.ti.com/lit/ds/sbas925a/sbas925a.pdf
    return writeByte(0B00000010); // 0x02
}

uint16_t ADS1119::readTwoBytes() 
{
    uint8_t value = 0x0;
    // 0. Calculate conversion time
    unsigned long conversionTime;
    switch (config->dataRate) 
    {
        case ADS1119Configuration::DataRate::sps20: conversionTime = 1000.0/20.0; break;
        case ADS1119Configuration::DataRate::sps90: conversionTime = 1000.0/90.0; break;
        case ADS1119Configuration::DataRate::sps330: conversionTime = 1000.0/330.0; break;
        default: conversionTime = 1.0; break;
    }
    // 1. Configure the device
    value |= (uint8_t(config->mux) << 5);                // XXX00000
    value |= (uint8_t(config->gain) << 4);               // 000X0000
    value |= (uint8_t(config->dataRate) << 2);           // 0000XX00
    value |= (uint8_t(config->conversionMode) << 1);     // 000000X0
    value |= (uint8_t(config->voltageReference) << 0);   // 0000000X
    // 2. Write the respective register configuration with the WREG command
    // 8.5.3.6 RREG (0010 0rxx) / Page 26
    // http://www.ti.com/lit/ds/sbas925a/sbas925a.pdf
    write(ADS1119_WREG_CMD, value);
    // 3. Send the START/SYNC command (08h) to start converting in continuous conversion mode;
    commandStart();
    // 4. Wait for an ADC Conversion
    delay(conversionTime);
    // 5. Send the RDATA command 
    commandReadData();
    // 6. Read 2 bytes of conversion data
    return read();
}

bool ADS1119::commandReadData()
{
    // 8.5.3.5 RDATA (0001 xxxx) / Page 26
    // http://www.ti.com/lit/ds/sbas925a/sbas925a.pdf
    return writeByte(ADS1119_RDATA_CMD); 
}

bool ADS1119::commandStart()
{
    // 8.5.3.3 START/SYNC (0000 100x) / Page 25
    // http://www.ti.com/lit/ds/sbas925a/sbas925a.pdf
    return writeByte(ADS1119_START_SYNC_CMD);
}

uint16_t ADS1119::read()
{
    _i2c->requestFrom(_address, (uint8_t)2);

    if (_i2c->available() < 2) {
         reset();
         return 0x0;
    }

    return ((_i2c->read() << 8) | _i2c->read());
}

bool ADS1119::write(uint8_t registerValue, uint8_t value)
{
    _i2c->beginTransmission(_address);
    _i2c->write(registerValue);
    _i2c->write(value);

    return _i2c->endTransmission() == 0;
}

bool ADS1119::writeByte(uint8_t value)
{
    _i2c->beginTransmission(_address);
    _i2c->write(value);

    return _i2c->endTransmission() == 0;
}

uint8_t ADS1119::readRegister(ADS1119RegisterToRead registerToRead)
{
    // 8.5.3.6 RREG (0010 0rxx) / Page 26
    // http://www.ti.com/lit/ds/sbas925a/sbas925a.pdf
    /*
    Reading a register must be performed as by using two I2C communication frames.
    */

    // 1. The first frame, the host sends the RREG command including the register address to the ADS1119.
    uint8_t byteToWrite = ADS1119_WREG_CMD || (uint8_t(registerToRead) << 2);
    _i2c->beginTransmission(_address);
    _i2c->write(byteToWrite);
    if (_i2c->endTransmission() != 0)
    {
        return 0;
    }

    delay(1);
    // 2. The second frame the ADS1119 reports the contents of the requested register.
    _i2c->requestFrom(_address, (uint8_t)1);

    return _i2c->read();
}

