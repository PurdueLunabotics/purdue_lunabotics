#include <Arduino.h>
#include <SPI.h>

#define CLOCK_SPEED 1'000'000u // 2.5 MHz SSI Clock
#define SCK_PIN 13             // SSI CLK line

#define SCK_PIN_13_INDEX 0
#define SCK_PIN_14_INDEX 1
#define SCK_PIN_27_INDEX 2

SPISettings settingsA(CLOCK_SPEED, MSBFIRST, SPI_MODE1);

uint8_t encoderBuf[3] = {0, 0, 0};

void makeTransfer();
uint32_t decodeEncoderFrame();

void calculateAndPrintPosition(uint32_t &encoderData);

void setup() {
    SerialUSB.begin(9600);
    SPI.begin(); // SPI.begin() will initialize the SPI port, as well as CLK pin

    pinMode(SCK_PIN, OUTPUT); // pinMode() will initialize the CLK pin as output
                              // (SPI port can't use it now!)
    digitalWriteFast(
        SCK_PIN,
        HIGH); // Set CLK line HIGH (to meet the requirements of SSI interface)

    while (!SerialUSB) {
    }
    SerialUSB.println("Started!");
}

void loop() {
    makeTransfer();
    uint32_t encoderData = decodeEncoderFrame();
    calculateAndPrintPosition(encoderData);

    delay(50);
}

void makeTransfer() {
    digitalWriteFast(SCK_PIN,
                     LOW); // Set CLK line LOW (to inform encoder -> latch data)

    // Before in setup() the pinMode() change the CLK pin function to output,
    // now we have to enable usage of this pin by SPI port with calling
    // SPI.begin():
    SPI.begin();

    // Or use this one below - a bit faster but SPI port and SCK pin dependent
    // option. I belive there is a more elegant and more scalable solution,
    // maybe even supported by hardware for this purpose? - if anyone can point
    // me it out I would be grateful:

    SPI.beginTransaction(settingsA); // We use transactional API

    for (int i = 0; i < 2; i++) {
        encoderBuf[i] =
            SPI.transfer(0xAA); // Transfer anything and read data back
    }

    SPI.endTransaction(); // We use transactional API

    pinMode(SCK_PIN, OUTPUT); // A while before we set CLK pin to be used by SPI
                              // port, now we have to change it manually...
    digitalWrite(SCK_PIN, HIGH); // ... back to idle HIGH

    delayMicroseconds(
        27); // Running above 500kHz perform Delay First Clock function
}

uint32_t decodeEncoderFrame() {
    uint32_t data =
        static_cast<uint32_t>(encoderBuf[0] << 8) |
        static_cast<uint32_t>(encoderBuf[1]); // here transfer8 was made

    // Shift one bit right - (MSB of position was placed on at MSB of uint32_t,
    // so make it right and shift to make MSB position at 30'th bit):
    // data = data >> 1;

    return data;
}

void calculateAndPrintPosition(uint32_t &encoderData) {
    // encoderPosition is placed in front (starting from MSB of uint32_t) so
    // shift it for 11 bits to align position data right
    uint32_t encoderPosition = encoderData;

    float angularAbsolutePosition = static_cast<float>(encoderPosition) /
                                    static_cast<float>(1 << 16) * 360.0F;

    SerialUSB.println("Encoder Positon = " + String(encoderPosition) +
                      " Angle = " + String(angularAbsolutePosition, 4));
}
