#include <esp_now.h>
#include <WiFi.h>

#define SERIAL_PORT 1 // Port where MAVLINK device is connected

#define TX_LED_PIN 20
#define RX_LED_PIN 21

uint8_t peerAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Replace with the peer's MAC address



HardwareSerial Serial_port(SERIAL_PORT); // Use UART1 for Serial1

typedef struct struct_message {
  uint8_t payload[250];
  int len;
} struct_message;

struct_message myData;

// Callback function when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  digitalWrite(RX_LED_PIN,HIGH);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial_port.write(myData.payload, myData.len);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  digitalWrite(TX_LED_PIN,HIGH);
}

void setup() {
  // Initialize Serial1 for MAVLink communication (adjust baud rate as needed)
  Serial_port.begin(115200, SERIAL_8N1, 16, 17); // RX on GPIO16, TX on GPIO17
  WiFi.begin();
  //esp_wifi_set_channel(10,WIFI_SECOND_CHAN_NONE);
  //esp_wifi_set_bandwidth(WIFI_IF_STA,WIFI_BW_HT20);
  WiFi.enableLongRange(true);
  delay(100);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  if (Serial1.available()) {
    myData.len = Serial1.readBytes(myData.payload, 250);
    esp_now_send(peerAddress, (uint8_t *) &myData, sizeof(myData));
  }
  digitalWrite(TX_LED_PIN,HIGH);
  digitalWrite(RX_LED_PIN,HIGH);
}
