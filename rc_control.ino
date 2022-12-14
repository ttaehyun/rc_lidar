#include <SoftwareSerial.h>
#include <Servo.h>

#define st_up 'w'
#define st_down 's'

SoftwareSerial esp32(7,6);
Servo steer;

int st = 90; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  esp32.begin(115200);
  steer.attach(9);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (esp32.available()) {
    int temp_st = st;
    char temp = esp32.read();
    Serial.write(temp);
    if (temp == st_up) {
      steer.write(40);
    }
    else if (temp == st_down) {
      steer.write(140);
    }
    
  }
  steer.write(st);
}
