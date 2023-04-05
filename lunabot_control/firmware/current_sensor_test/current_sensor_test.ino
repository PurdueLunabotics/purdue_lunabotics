/*
ADS1115 lite library - Adapted from adafruit ADS1015/ADS1115 library

        This library is stripped down version with bug fixes from the adafruit
ADS1015/ADS1115 library in order to save as much space as possible
                1. No explicit "Read ADC" functions exist to save space.  Simply
set your mux manually, trigger a conversion manually, and read the value
manually.  See the example program
                2. ADS1015 functionality removed to save space.  Note that this
library should work for the ADS1015, the only difference is the sample rate
constants.
                3. Continuous Conversion removed
                        a. If "continuous conversion" is required, perhaps you
should rethink your requirements.  Continuous conversion is never REALLY
required. b. That being said, if you want a continuous conversion simply start a
conversion after reading a value by executing "adc.triggerConversion()"
immediatly AFTER an "adc.getConversion();".  That's all the chip does anyway...
                4. Comparator functionality removed to save space.  Cmon..you
can program that functionality if you REALLY need it, this is a lite library
                5. I2C abstraction removed to save space.  Is anyone using old
Arduino IDE versions?
                6. keywords.txt added in order to display proper keyword
coloring in the Arduino IDE
                7. All report bug fixes from the adafruit library on github
                        a. Most notably the incorrect sample rate used when
using the ADS1115.  Yes that's right, if you used the Adafruit library the
sample rate was set to the fastest, noisiest sample rate by default with no way
to change it!  Opps!
                8. Proper ifndef added to prevent multiple calls
                9. Removed explicit delays and intead poll the Operational
Status bit (conversion done)

        https://github.com/terryjmyers/ADS1115-Lite.Git
    v1.0 - First release
*/

#include <ADS1115_lite.h>
ADS1115_lite adc(
    ADS1115_ADDRESS_ADDR_SDA); // Initializes wire library, sets private
                               // configuration variables to ADS1115
                               // default(2.048V, 128SPS, Differential mode
                               // between  AIN0 and AIN1.  The Address parameter
                               // is not required if you want default
ADS1115_lite adc1(
    ADS1115_ADDRESS_ADDR_SCL); // Initializes wire library, sets private
                               // configuration variables to ADS1115
                               // default(2.048V, 128SPS, Differential mode
                               // between  AIN0 and AIN1.  The Address parameter
                               // is not required if you want default

// Setup some variables used to show how long the ADC conversion takes place
unsigned long starttime;
unsigned long endtime;
int16_t Raw;

void setup(void) {
    // Setup Serial
    // In case the user uses the default setting in Arduino, give them a message
    // to change BAUD rate
    Serial.begin(9600);
    Serial.println(F("Set Serial baud rate to 250 000"));
    Serial.flush(); // wait for send buffer to empty
    delay(2);       // let last character be sent
    Serial.end();   // close serial
    Serial.begin(250000);
    Serial.println();

    // Typically the Gain and sample data rate should be set in setup as these
    // typically don't change for a project, but you can change them any time!
    adc.setGain(ADS1115_REG_CONFIG_PGA_0_256V);
    adc.setSampleRate(ADS1115_REG_CONFIG_DR_64SPS); // Set the slowest and most
                                                    // accurate sample rate

    adc1.setGain(ADS1115_REG_CONFIG_PGA_0_256V);
    adc1.setSampleRate(ADS1115_REG_CONFIG_DR_64SPS); // Set the slowest and most
                                                     // accurate sample rate

    // Test to see if you can communicate to the ADS1115
    if (!adc.testConnection()) {
        Serial.println(
            "ADS1115 0 Connection failed"); // oh man...something is wrong
        return;
    }

    // Test to see if you can communicate to the ADS1115
    if (!adc1.testConnection()) {
        Serial.println(
            "ADS1115 1 Connection failed"); // oh man...something is wrong
        return;
    }
}

void loop(void) {

    // The mux setting must be set every time each channel is read, there is NOT
    // a separate function call for each possible mux combination.
    adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0); // Set single ended mode
                                                 // between AIN0 and GND
    // To lower the size of the library to bare minimum you must manually
    // trigger the conversion, its really not hard...
    starttime = micros();      // Record a start time for demonstration
    adc.triggerConversion();   // Start a conversion.  This immediatly returns
    Raw = adc.getConversion(); // This polls the ADS1115 and wait for conversion
                               // to finish, THEN returns the value
    endtime = micros();
    Serial.println("=======================");
    Serial.print(Raw);
    Serial.print(", ADC 0  ");
    Serial.print(endtime - starttime);
    Serial.println("us");
    Serial.println("=======================");

    delay(2000); // Delay just because

    // The mux setting must be set every time each channel is read, there is NOT
    // a separate function call for each possible mux combination.
    adc1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0); // Set single ended mode
                                                  // between AIN0 and GND
    // To lower the size of the library to bare minimum you must manually
    // trigger the conversion, its really not hard...
    starttime = micros();       // Record a start time for demonstration
    adc1.triggerConversion();   // Start a conversion.  This immediatly returns
    Raw = adc1.getConversion(); // This polls the ADS1115 and wait for
                                // conversion to finish, THEN returns the value
    endtime = micros();
    Serial.println("=======================");
    Serial.print(Raw);
    Serial.print(", ADC 1 ");
    Serial.print(endtime - starttime);
    Serial.println("us");
    Serial.println("=======================");

    delay(2000); // Delay just because
}
