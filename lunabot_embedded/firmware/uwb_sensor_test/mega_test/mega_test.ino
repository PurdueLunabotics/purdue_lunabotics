#include <Wire.h>
#include <Arduino.h>
#include <string.h>
String DATA = " ";
String DATA1 = " ";
String DATA2 = " ";
int UWB_T_NUMBER = 0;
float d0, d1, d2; // Declare variables to store distances

void setup() {
  // Initialize Serial1 and Serial2 with baud rates
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);

  //serial 1 set up as base on ground (send data)
  for (int b = 0; b < 2; b++) {
    delay(50);
    Serial1.write("AT+anchor_tag=1,0");  // Set the base station 设置基站
    Serial1.print("ID0");  // UWB_B_NUMBER is base station ID0~ID3
    Serial1.write("\r\n");
    delay(1);
    delay(50);
    if (b == 0) 
    {
      Serial1.write("AT+RST\r\n");
    }
  }
    if (Serial1.available()) 
    {
      delay(3);
      DATA = Serial1.readString();
    }
    DATA = "";

  // serial 2 setup as tag on bot (does dist calc)
  for (int b = 0; b < 2; b++) { // Repeat twice to stabilize the connection
      delay(50);
      Serial2.write("AT+anchor_tag=0\r\n"); // Set up the Tag
      delay(50);
      Serial2.write("AT+interval=50\r\n"); // Set the calculation precision,
                                            // the larger the response is, the
                                            // slower it will be
      delay(50); // 设置计算精度，越大响应越慢
      Serial2.write("AT+switchdis=1\r\n"); // Began to distance 开始测距
      delay(50);
      if (b == 0) {
          Serial2.write("AT+RST\r\n"); // RESET 复位
      }
  }
  if (Serial2.available()) {
      delay(3);
      DATA = Serial2.readString();
  }
  DATA = "";

  for (int b = 0; b < 2; b++) {
    delay(50);
    Serial3.write("AT+anchor_tag=1,1");  // Set the base station 设置基站
    Serial3.print("ID2");  // UWB_B_NUMBER is base station ID0~ID3
    Serial3.write("\r\n");
    delay(1);
    delay(50);
    if (b == 0) 
    {
      Serial3.write("AT+RST\r\n");
    }
  }
    if (Serial3.available()) 
    {
      delay(3);
      DATA2 = Serial3.readString();
    }
    DATA2 = "";
}

void loop()
{
  if (Serial2.available()) {
    // Read the original input
    String originalInput = Serial2.readStringUntil('\n');
    originalInput.trim();

    // Check if the message starts with "anX:" where X is a digit
    if (originalInput.startsWith("an") &&
        isdigit(originalInput.charAt(2)) &&
        originalInput.charAt(3) == ':') {
        // Extract sensor number
        int sensorNumber = originalInput.charAt(2) - '0';

        // Find the index of the colon
        int colonIndex = originalInput.indexOf(':');

        // Extract the value part and trim spaces
        String valueString = originalInput.substring(colonIndex + 1);
        valueString.trim();

        // Convert the trimmed string to a float
        float num = valueString.toFloat();

        // Assign the values to d0, d1, or d2 based on sensor number
        if (sensorNumber == 0) {
            d0 = num;
        } else if (sensorNumber == 1) {
            d1 = num;
        } else if (sensorNumber == 2) {
            d2 = num;
        }

        // Print the processed output with a blank line before it
        Serial.print("Dist ");
        Serial.print(sensorNumber);
        Serial.print(": ");
        Serial.println(num, 2);
    }
  } else {
    Serial.println(Serial2.available());
  }
}