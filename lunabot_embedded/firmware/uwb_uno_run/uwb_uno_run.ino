#include <Arduino.h>
#include <Wire.h>
#include <string.h>
String DATA = " ";
int UWB_T_NUMBER = 2;

void setup() {
    // Initialize Serial1 and Serial2 with baud rates
    Serial.begin(115200);

    for (int b = 0; b < 2; b++) {
      delay(50);
      Serial.write("AT+anchor_tag=1,");  // Set the base station 设置基站
      Serial.print(UWB_T_NUMBER);  // UWB_B_NUMBER is base station ID0~ID3
      Serial.write("\r\n");
      delay(1);
      delay(50);
      if (b == 0) {
          Serial.write("AT+RST\r\n");
      }
    }
}

void loop() {

}