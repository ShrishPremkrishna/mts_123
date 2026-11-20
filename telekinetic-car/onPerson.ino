#include <WiFi.h>
#include <esp_now.h>

uint8_t slaveMAC[] = {0x88, 0x57, 0x21, 0x78, 0xA7, 0xEC};

struct Button { //all information needed to handle toggling button state
  const int pin;
  int reading;
  int previous;
  bool state;
  unsigned long lastTime;
};

Button buttons[] = {
  {25, HIGH, HIGH, false, 0}, //reverse direction
  {26, HIGH, HIGH, false, 0}, //stop
  {27, HIGH, HIGH, false, 0} //advance calibration state
};

const unsigned long debounce = 200;
const int numButtons = 3;

void setup() {
  Serial.begin(9600);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onSend);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, slaveMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  for (int i = 0; i < numButtons; i++) {
    pinMode(buttons[i].pin, INPUT_PULLUP); //initialize each button in the button list with mode input pullup
  }
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    
    esp_err_t result = esp_now_send(slaveMAC, (uint8_t *)line.c_str(), line.length());
    
    if (result != ESP_OK) {
      Serial.println("Send error");
    }
  }

  readButtons();
}

//allow interface between breadboard button circuit and Python code for additional functionality
void readButtons() {
  for (int i = 0; i < numButtons; i++) { //repeat state reading code for each button in buttons[]
    buttons[i].reading = digitalRead(buttons[i].pin);

    if (buttons[i].reading == LOW && buttons[i].previous == HIGH && millis() - buttons[i].lastTime > debounce) {
      buttons[i].state = !buttons[i].state; //update buttons
      buttons[i].lastTime = millis();

      //send update to Python code if button is toggled on/off
      Serial.print("EVT_BTN");
      Serial.print(i + 1);
      Serial.print(":");
      Serial.println(buttons[i].state ? "1" : "0");
    }
    buttons[i].previous = buttons[i].reading; //update buttons
  }
}

void onSend(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // Callback when data is sent
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Sent successfully!");
  } 
  else {
    Serial.println("Send failed!");
  }
}
