#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

Servo throttle; //speed control - ESC
Servo steering; //maneuvering control - Servo
int8_t recvDeg;
bool update = false;

unsigned long lastReceiveTime = 0;

void setup() {
  throttle.attach(25); //attach to pins on ESP32
  steering.attach(26);

  Serial.begin(9600); //baud rate

  WiFi.mode(WIFI_STA);

  esp_now_init();
  esp_now_register_recv_cb(onRecv);

  throttle.writeMicroseconds(1500); //initialize neutral throttle
  delay(1000);

  steering.writeMicroseconds(1500); //initialize neutral steering
  delay(1000);
}

void loop() {
  if (update == true) {
    steering.writeMicroseconds(1500 + recvDeg * (500.00/90.00));
    update = false;
  }

  if (millis() - lastReceiveTime > 500) { //if no signal, correct steering to straighten
    steering.writeMicroseconds(1500);
  }
}

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
  Serial.print("tilt degree data received: ");
  Serial.println(*data);
  memcpy(&recvDeg, data, sizeof(recvDeg));
  lastReceiveTime = millis();
  update = true;
}