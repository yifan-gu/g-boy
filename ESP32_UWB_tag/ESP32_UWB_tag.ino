// currently tag is module #5
// The purpose of this code is to set the tag address and antenna delay to default.
// this tag will be used for calibrating the anchors.

#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <SPI.h>

#include "DW1000Ranging.h"
#include "DW1000.h"

using namespace websockets;

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// TAG antenna delay defaults to 16384
// leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

// Replace with your WiFi credentials
const char* ssid = "RC-Controller";
const char* password = "12345678";

// Replace with your server's address and port
const char* serverAddress = "ws://autocam.local:80/ws";

WebsocketsClient wsClient;

void setup() {
  // Start Serial for debugging
  unsigned long startTime = millis();
  unsigned long timeout = 1000; // 1 seconds timeout
                                //
  Serial.begin(115200);
  while (!Serial && (millis() - startTime < timeout)) {
    // Wait for Serial or timeout
  }

  setupUWBTag();
  setupWebClient();
}

void loop()
{
  DW1000Ranging.loop();

  // Keep WebSocket client alive
  wsClient.poll();

  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();

  // Send data every 5 seconds
  if (wsClient.available() && currentTime - lastSendTime >= 1000) {
    sendSensorData();
    lastSendTime = currentTime;
  }

}

void setupUWBTag() {
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // start as tag, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void setupWebClient() {
  // Connect to WiFi
  Serial.printf("Connecting to WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());

  // Configure WebSocket client
  wsClient.onMessage(onMessageCallback);
  wsClient.onEvent(onEventsCallback);

  // Connect to the WebSocket server
  if (wsClient.connect(serverAddress)) {
    Serial.println("WebSocket connected!");
  } else {
    Serial.println("WebSocket connection failed!");
  }
}

void sendSensorData() {
  // Simulate sensor data
  float dF = random(10, 100) / 10.0;
  float dL = random(10, 100) / 10.0;
  float dR = random(10, 100) / 10.0;

  // Create JSON message
  String message = "dF=" + String(dF) + "&dL=" + String(dL) + "&dR=" + String(dR);

  // Send the message
  wsClient.send(message);
  Serial.println("Sent: " + message);
}

void onMessageCallback(WebsocketsMessage message) {
  // Handle incoming messages from the server
  Serial.printf("Received: %s\n", message.data().c_str());
}

void onEventsCallback(WebsocketsEvent event, String data) {
  // Handle WebSocket events
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("WebSocket connection opened!");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("WebSocket connection closed!");
  } else if (event == WebsocketsEvent::GotPing) {
    Serial.println("Ping received!");
  } else if (event == WebsocketsEvent::GotPong) {
    Serial.println("Pong received!");
  }
}

void newRange() {
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print(",");
  Serial.println(DW1000Ranging.getDistantDevice()->getRange());
}

void newDevice(DW1000Device *device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
