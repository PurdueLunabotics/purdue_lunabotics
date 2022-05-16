
#define DATA_PIN 35

volatile int state = 0; // 1 = NEAR MAGNET, 0 = FREE 

void cb(void) {
    if(digitalRead(DATA_PIN) == LOW) {
        state++;
    }
}

void setup() {
    Serial.begin(9600);
    pinMode(DATA_PIN, INPUT_PULLUP);
    attachInterrupt(DATA_PIN, cb, FALLING);
}

void loop() {
    if(digitalRead(DATA_PIN) == LOW) {
        Serial.println("DETECTED!");
    }
    else {
        Serial.println("NOTHING.");
    }

    Serial.println("STATE: ");
    Serial.println(state);
    delay(50);
}
