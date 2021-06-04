#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#include <Servo.h>
Servo servo;

#define SERVOPIN 32
#define ONBOARD 2

#define UP 0
#define DOWN 1
int state = UP;


void penUp() {
  servo.attach(SERVOPIN);
  for (int i=150; i>=90; i--) {
    servo.write(i);
    delay(15);
  }
  delay(50);
  servo.detach();
  state = UP;
}

void penDown() {
  servo.attach(SERVOPIN);
  for (int i=90; i<150; i++) {
    servo.write(i);
    delay(15);
  }
  delay(50);
  servo.detach();
  state = DOWN;
}
 
void setup() {
  Serial.begin(115200);

  SerialBT.begin("PenServer"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  pinMode(ONBOARD, OUTPUT);
 
 
  penDown();
  delay(200);
  penUp();

  
}

unsigned long blinker = 0;
 
void loop() {

  if (SerialBT.available()) {
    char c =  SerialBT.read();
    Serial.println(c);
    if (c!=0) {
      penDown();
      SerialBT.write((byte)state);
      Serial.println("Pen down");
    } else {
      penUp();
      SerialBT.write((byte)state);
      Serial.println("Pen up");
    }
    while (SerialBT.available()) {
      SerialBT.read();
      SerialBT.write((byte)state);
    }
  }
 
  if (millis()-blinker>10) {
    digitalWrite(ONBOARD,LOW);
  }
  if (millis()-blinker>2000) {
    digitalWrite(ONBOARD,HIGH);
    blinker = millis();
  }
}
