//anchor #1 setup


// be sure to edit anchor_addr and select the previously calibrated anchor delay
// my naming convention is anchors 1, 2, 3, ... have the lowest order byte of the MAC address set to 81, 82, 83, ...

#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <SPI.h>
#include <QMC5883LCompass.h>

#include "DW1000Ranging.h"
#include "DW1000.h"

using namespace websockets;

#define PI 3.14159265359

// leftmost two bytes below will become the "short address"
char anchor_addr[] = "81:00:5B:D5:A9:9A:E2:9C"; //#4

//calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16630;


// previously determined calibration results for antenna delay
// #1 16630
// #2 16610
// #3 16607
// #4 16580

float distance; // The distance to the target.
float camHeading; // The heading of the camera.

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// Replace with your WiFi credentials
const char* ssid = "RC-Controller";
const char* password = "12345678";

// Replace with your server's address and port
const char* serverAddress = "ws://autocam.local:80/ws";

WebsocketsClient wsClient;

QMC5883LCompass compass;

void setup() {
  // Start Serial for debugging
  unsigned long startTime = millis();
  unsigned long timeout = 1000; // 1 seconds timeout
                                //
  Serial.begin(115200);
  while (!Serial && (millis() - startTime < timeout)) {
    // Wait for Serial or timeout
  }

  setupUWBAnchor();
  setupWebClient();
  setupMPU();
}

void loop() {
  DW1000Ranging.loop();
  runMPUMeasurement();
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

  // Send data every 100ms
  if (wsClient.available() && currentTime - lastSendTime >= 10) {
    sendSensorData();
    lastSendTime = currentTime;
  }
}

void runMPUMeasurement() {
  compass.read();

  float x = compass.getX();
  float y = compass.getY();

  camHeading = atan2(x, y) * 180 / PI;
  if (camHeading < 0) {
    camHeading += 360; //  N=0/360, W=90, S=180, E=270
  }
}

void setupUWBAnchor() {
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  // set antenna delay for anchors only. Tag is default (16384)
  DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  //start the module as an anchor, do not assign random short address
  DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
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

void setupMPU() {
  compass.init();
  Serial.println("Calibration will begin in 5 seconds.");
  delay(5000);

  Serial.println("CALIBRATING. Keep moving your sensor...");
  compass.calibrate();
  Serial.println("DONE");
}

void sendSensorData() {
  String distanceMessage = "distance=" + String(distance, 2) + "&camHeading=" + String(camHeading, 2);
  wsClient.send(distanceMessage);
  Serial.println("Sent: " + distanceMessage);
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
  float dF = sqrt(pow(targetX - xF, 2) + pow(targetY - yF, 2));
  float dL = sqrt(pow(targetX - xL, 2) + pow(targetY - yL, 2));
  float dR = sqrt(pow(targetX - xR, 2) + pow(targetY - yR, 2));

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
  const float step = 0.1; // Increment per step (controls speed)

  // Square corner positions
  const float squareSize = 3.0; // Half the side length of the square (radius equivalent)
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
  const float carX = 0, carY = 0;  // Car's current position
  const float carOrientation = 0; // Assume the car's orientation is 0 (facing along x-axis)

  // Calculate distance to the target
  float distance = sqrt(pow(targetX - carX, 2) + pow(targetY - carY, 2));

  // Calculate orientation to the target
  float camHeading = atan2(targetY - carY, targetX - carX) * 180.0 / PI; // Convert to degrees

  // Relative orientation of the car
  float carHeading = camHeading; // Replace with the actual car orientation if it's dynamic

  // Send each value as a separate message
  String distanceMessage = "distance=" + String(distance, 2);
  wsClient.send(distanceMessage);
  Serial.println("Sent: " + distanceMessage);

  String orientationTargetMessage = "camHeading=" + String(camHeading, 2);
  wsClient.send(orientationTargetMessage);
  Serial.println("Sent: " + orientationTargetMessage);

  String orientationCarMessage = "carHeading=" + String(carHeading, 2);
  wsClient.send(orientationCarMessage);
  Serial.println("Sent: " + orientationCarMessage);

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
  //    Serial.print("from: ");
  //Serial.print("address: "); Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), DEC);
  distance = DW1000Ranging.getDistantDevice()->getRange();
  //Serial.print(",");
  //Serial.print("distance = "); Serial.print(distance); Serial.println("m");
}

void newDevice(DW1000Device *device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
