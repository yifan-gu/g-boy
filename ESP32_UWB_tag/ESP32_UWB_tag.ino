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

float dF = 0;
float dL = 0;
float dR = 0;

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
  runWebClientLoop();
}

void runWebClientLoop() {
  // Keep WebSocket client alive
  wsClient.poll();

  static unsigned long lastSendTime = 0;
  static unsigned long lastReconnectAttempt = 0;
  unsigned long currentTime = millis();

  // Check WebSocket connection and attempt reconnection if disconnected
  if (!wsClient.available() && currentTime - lastReconnectAttempt >= 1000) {
    Serial.println("WebSocket disconnected, attempting to reconnect...");
    setupWebClient();
    lastReconnectAttempt = currentTime;
  }

  // Send data every 5 seconds
  if (wsClient.available() && currentTime - lastSendTime >= 100) {
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
  String message = "dF=" + String(dF, 2) + "&dL=" + String(dL, 2) + "&dR=" + String(dR, 2);
  // Send the message
  wsClient.send(message);
  Serial.println("Sent: " + message);
}

void sendSensorDataCircular() {
  static float angle = 0.0; // Current angle in radians
  const float radius = 3.0; // Radius of the circle in meters
  const float step = 0.1;   // Angle increment per step in radians (controls speed)

  // Calculate current target position
  float targetX = radius * cos(angle);
  float targetY = radius * sin(angle);

  // Sensor positions
  const float xF = 0, yF = 1;  // Forward sensor
  const float xL = -1, yL = 0; // Left sensor
  const float xR = 1, yR = 0;  // Right sensor

  // Calculate distances to the target for each sensor
  dF = sqrt(pow(targetX - xF, 2) + pow(targetY - yF, 2));
  dL = sqrt(pow(targetX - xL, 2) + pow(targetY - yL, 2));
  dR = sqrt(pow(targetX - xR, 2) + pow(targetY - yR, 2));

  // Create JSON message
  String message = "dF=" + String(dF, 2) + "&dL=" + String(dL, 2) + "&dR=" + String(dR, 2);

  // Send the message
  wsClient.send(message);
  Serial.println("Sent: " + message);

  // Increment angle for the next step
  angle += step;
  if (angle >= 2 * PI) {
    angle -= 2 * PI; // Wrap angle to stay within 0 to 2π
  }
}

void sendSensorDataSquare() {
  static int corner = 0; // Current corner of the square
  static float t = 0.0;  // Interpolation factor between corners
  const float step = 0.01; // Increment per step (controls speed)

  // Square corner positions
  const float squareSize = 5.0; // Half the side length of the square (radius equivalent)
  const float corners[4][2] = {
      { squareSize, squareSize },  // Top-right corner
      { -squareSize, squareSize }, // Top-left corner
      { -squareSize, -squareSize }, // Bottom-left corner
      { squareSize, -squareSize }   // Bottom-right corner
  };

  // Current and next corners
  float currentCornerX = corners[corner][0];
  float currentCornerY = corners[corner][1];
  float nextCornerX = corners[(corner + 1) % 4][0];
  float nextCornerY = corners[(corner + 1) % 4][1];

  // Interpolate position between current and next corner
  float targetX = currentCornerX + t * (nextCornerX - currentCornerX);
  float targetY = currentCornerY + t * (nextCornerY - currentCornerY);

  // Sensor positions
  const float xF = 0, yF = 1;  // Forward sensor
  const float xL = -1, yL = 0; // Left sensor
  const float xR = 1, yR = 0;  // Right sensor

  // Calculate distances to the target for each sensor
  dF = sqrt(pow(targetX - xF, 2) + pow(targetY - yF, 2));
  dL = sqrt(pow(targetX - xL, 2) + pow(targetY - yL, 2));
  dR = sqrt(pow(targetX - xR, 2) + pow(targetY - yR, 2));

  // Create JSON message
  String message = "dF=" + String(dF, 2) + "&dL=" + String(dL, 2) + "&dR=" + String(dR, 2);

  // Send the message
  wsClient.send(message);
  Serial.println("Sent: " + message);

  // Increment interpolation factor
  t += step;
  if (t >= 1.0) {
    t = 0.0; // Reset interpolation factor
    corner = (corner + 1) % 4; // Move to the next corner
  }
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
  DW1000Device *device = DW1000Ranging.getDistantDevice();
  int addr = device->getShortAddress();
  Serial.print(addr, HEX);
  switch (addr) {
    case 0x81:
      dF = device->getRange();
      break;

    case 0x82:
      dL = device->getRange();
      break;

    case 0x83:
      dR = device->getRange();
      break;
  }
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
