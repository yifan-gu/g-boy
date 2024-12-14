#include <WiFi.h>
#include <SPI.h>
#include <QMC5883LCompass.h>

#include "DW1000Ranging.h"
#include "DW1000.h"

// #0 is for the anchor that's facing the target.
// #1 is for the anchor that's fixed on the car.
#define ANCHOR_INDEX 1

#define PI 3.14159265359

// leftmost two bytes below will become the "short address"
// "FF" is the place_holder.
char anchor_addr[] = "FF:00:5B:D5:A9:9A:E2:9C";
const uint8_t anchor_addr_prefix = 0x80;

//calibrated Antenna Delay setting for this anchor
uint16_t Adelay[] = {16630, 16610};

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
IPAddress serverIP, udpServerIP;

// UDP Configuration
WiFiUDP udpClient;
const char* udpServerDomain = "autocam.local"; // Replace with the receiver's domain
const unsigned int udpServerPort = 12345;

float distance = 0;
float heading = 0;

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
  setupWiFi();
  setupMPU();
}

void loop() {
  DW1000Ranging.loop();
  runMPUMeasurement();
  sendSensorData();
}

void setupUWBAnchor() {
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  // set antenna delay for anchors only. Tag is default (16384)
  DW1000.setAntennaDelay(Adelay[ANCHOR_INDEX]);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  //start the module as an anchor, do not assign random short address
  DW1000Ranging.startAsAnchor(getAnchorAddr(), DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

char* getAnchorAddr() {
  // Convert char* anchor_addr_base to a String and concatenate
  String addrPrefix = String(anchor_addr_prefix + ANCHOR_INDEX, HEX);
  const char *prefix = addrPrefix.c_str();
  anchor_addr[0] = prefix[0];
  anchor_addr[1] = prefix[1];
  return anchor_addr;
}

void setupWiFi() {
  // Connect to WiFi
  Serial.printf("Connecting to WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  serverIP = WiFi.localIP();
  Serial.printf("IP Address: %s\n", serverIP.toString().c_str());

  if (WiFi.hostByName(udpServerDomain, udpServerIP)) {
    Serial.print("Resolved UDP IP: ");
    Serial.println(udpServerIP);
  } else {
    Serial.println("Failed to resolve domain!");
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
  // Send `distance` data
  String distanceMessage = "distance"+ String(ANCHOR_INDEX) + "=" + String(distance, 2);
  sendUDP(distanceMessage);
  String headingMessage = "heading" + String(ANCHOR_INDEX) + "=" + String(heading, 2);
  sendUDP(headingMessage);
  delay(10); // 100Hz.
}

void sendUDP(String data) {
  udpClient.beginPacket(udpServerIP, udpServerPort);
  udpClient.print(data);
  udpClient.endPacket();
  Serial.println("Sent via UDP: " + data);
}

void newRange() {
  distance = DW1000Ranging.getDistantDevice()->getRange();
}

void newDevice(DW1000Device *device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

void runMPUMeasurement() {
  compass.read();

  float x = compass.getX();
  float y = compass.getY();

  heading = atan2(x, y) * 180 / PI;
  if (heading < 0) {
    heading += 360; //  N=0/360, W=90, S=180, E=270
  }
}
