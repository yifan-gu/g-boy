#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <ESPmDNS.h>
#include <math.h>

#include "index_html.h"


// Pin configuration
#define STEERING_PIN 2  // GPIO2 for steering
#define THROTTLE_PIN 3  // GPIO3 for throttle

// Access Point credentials
const char* ssid = "RC-Controller";
const char* password = "12345678"; // Minimum 8 characters
const char* hostname = "autocam"; // mDNS hostname


// Async WebSocket server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Servo objects
Servo throttle;
Servo steering;

// Define throttle and steering ranges
const int minThrottle = 1000, maxThrottle = 2000, midThrottle = 1500;
const int minSteering = 1000, maxSteering = 2000, midSteering = 1500;

// Variable to toggle the free vs lock mode.
boolean isLockMode = false;

// Variables to store values
int throttleValue = midThrottle, steeringValue = midSteering;

// The reported distance from all three UWB anchors to the tag.
float dF = 0; // Distance of the tag to the front anchor.
float dL = 0; // Distance of the tag to the left anchor.
float dR = 0; // Distance of the tag to the right anchor.

// The coordinates of the anchor.
const float xF = 0, yF = 0.5;
const float xL = -0.5, yL = 0;
const float xR = 0.5, yR = 0;

const float delta = 0.5; // Allowed measurement error delta.

// The calculated coordinates of the tag.
float currentX = 0, currentY = 0;

// The target coordinates to match.
float targetX = 0, targetY = 0;

// Heartbeat tracking
unsigned long lastPingTime = 0;           // Time of last received ping
const unsigned long heartbeatTimeout = 1000; // 1 second timeout

const int steeringChangeValue = 500;
const int steeringThrottleChangeValue = 200;

int moveThrottle = 200;


void setup() {
  // Start Serial for debugging
  unsigned long startTime = millis();
  unsigned long timeout = 1000; // 1 seconds timeout

  Serial.begin(115200);
  while (!Serial && (millis() - startTime < timeout)) {
    // Wait for Serial or timeout
  }

  setupESC();
  setupWebServer();
}

void loop() {
  calculateCoordinates();
  calculateSteeringThrottle();
  runESCController();
  runClientHealthCheck();
}

void setupESC() {
  Serial.println("Initializing ESC...");
  delay(1000); // Wait 1 seconds to ensure the ESC ready to arm.

  // Attach servos
  throttle.attach(THROTTLE_PIN, minThrottle, maxThrottle);
  steering.attach(STEERING_PIN, minSteering, maxSteering);

  // Initialize to neutral positions to arm the ESC.
  throttle.writeMicroseconds(midThrottle);
  steering.writeMicroseconds(midSteering);

  Serial.println("ESC Initialized!");
}

void setupWebServer() {
 // Start Access Point
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started!");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Start mDNS
  if (!MDNS.begin(hostname)) {
    Serial.println("Error starting mDNS");
    return;
  }
  Serial.println("mDNS responder started!");
  Serial.print("Access the device at http://");
  Serial.print(hostname);
  Serial.println(".local");

  // Configure WebSocket events
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Serve static files
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", index_html);
    });

  // Start server
  server.begin();
}

// Handle WebSocket events
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.println("WebSocket connected");
      break;

    case WS_EVT_DISCONNECT:
      Serial.println("WebSocket disconnected");
      throttleValue = midThrottle;
      steeringValue = midSteering;
      Serial.println("Throttle and Steering reset to middle positions");
      break;

    case WS_EVT_DATA: {
      String message = String((char*)data).substring(0, len);
      //Serial.println("Message received: " + message);
      lastPingTime = millis();

      if (message == "REQUEST_DATA") {
        String response = "{";
        response += "\"throttle\":" + String(throttleValue) + ",";
        response += "\"steering\":" + String(steeringValue) + ",";
        response += "\"dF\":" + String(dF) + ",";
        response += "\"dL\":" + String(dL) + ",";
        response += "\"dR\":" + String(dR) + ",";
        response += "\"currentX\":" + String(currentX) + ",";
        response += "\"currentY\":" + String(currentY) + ",";
        response += "\"targetX\":" + String(targetX) + ",";
        response += "\"targetY\":" + String(targetY);
        response += "}";
        client->text(response); // Send data to the requesting client
        // Serial.println("Sent data to client: " + response);
      } else if (message.startsWith("mode=")) {
        if (message == "mode=lock") {
          isLockMode = true;
          Serial.println("Switched to Lock Mode");
        } else if (message == "mode=free") {
          isLockMode = false;
          throttleValue = midThrottle;
          steeringValue = midSteering;
          Serial.println("Switched to Free Mode");
        }
      } else if (message.startsWith("dF=")) {
        // Parse the combined dF, dL, dR message
        int dFIndex = message.indexOf("dF=") + 3;
        int dLIndex = message.indexOf("&dL=") + 4;
        int dRIndex = message.indexOf("&dR=") + 4;

        if (dFIndex != -1 && dLIndex != -1 && dRIndex != -1) {
          dF = message.substring(dFIndex, message.indexOf("&dL=")).toFloat();
          dL = message.substring(dLIndex, message.indexOf("&dR=")).toFloat();
          dR = message.substring(dRIndex).toFloat();
        }
      } else if (message.startsWith("throttle=")) {
        throttleValue = map(message.substring(9).toInt(), 1000, 2000, minThrottle, maxThrottle);
      } else if (message.startsWith("steering=")) {
        steeringValue = map(message.substring(9).toInt(), 1000, 2000, minSteering, maxSteering);
      }
      break;
    }

    case WS_EVT_ERROR:
      Serial.println("WebSocket error");
      break;

    default:
      Serial.println("Unknown WebSocket event");
      break;
  }
}

void runESCController() {
  steering.writeMicroseconds(steeringValue);
  throttle.writeMicroseconds(throttleValue);

   /*Serial.print("Throttle: ");
   Serial.print(throttleValue);
   Serial.print(" | ");
   Serial.print("Steering: ");
   Serial.println(steeringValue);*/
}

void runClientHealthCheck() {
  // Check for heartbeat timeout
  if (millis() - lastPingTime > heartbeatTimeout) {
    Serial.println("Heartbeat timeout, reset");
    throttleValue = midThrottle; // Reset throttle and steering
    steeringValue = midSteering;
    lastPingTime = millis(); // Reset timer to avoid repeated timeouts
  }
}

void calculateSteeringThrottle() {
  if (!isLockMode) {
    return;
  }

  if (calculateDistance(currentX, currentY) - calculateDistance(targetX, targetY) <= delta) {
    return;
  }

  if (currentY - targetY > delta) {
    setMoveForward();
  } else if (targetY - currentY > delta) {
    setMoveBackward();
  }

  if (currentX - targetX > delta) {
    setSteeringRight();
  } else if (targetX - currentX > delta) {
    setSteeringLeft();
  }
}

float calculateDistance(float X, float Y) {
  // Euclidean distance formula
  return sqrt(pow(X, 2) + pow(Y, 2));
}

void calculateCoordinates() {
  // Coefficients for the linear equations
  float vA1 = 2 * (xL - xF);
  float vB1 = 2 * (yL - yF);
  float vC1 = dF * dF - dL * dL - xF * xF + xL * xL - yF * yF + yL * yL;

  float vA2 = 2 * (xR - xF);
  float vB2 = 2 * (yR - yF);
  float vC2 = dF * dF - dR * dR - xF * xF + xR * xR - yF * yF + yR * yR;

  // Solve for x and y using the determinant method
  float determinant = vA1 * vB2 - vA2 * vB1;

  if (fabs(determinant) < 1e-6) { // Check for near-zero determinant (sensors might be collinear)
    Serial.println("Error: Sensors are collinear or input is invalid.");
    currentX = 0;
    currentY = 0;
  } else {
    currentX = (vC1 * vB2 - vC2 * vB1) / determinant;
    currentY = (vA1 * vC2 - vA2 * vC1) / determinant;
  }

  // Serial.print("current x, current y:");
  // Serial.println(String(currentX) + ", " + String(currentY));

  if (!isLockMode) { // Update the target coordinates as well if it's not in lock mode to prevent large variation when the lock mode is turned on.
    targetX = currentX;
    targetY = currentY;
  }
  return;
}

void setSteeringLeft() {
  steeringValue = midSteering + steeringChangeValue;
}

void setSteeringRight() {
  steeringValue = midSteering - steeringChangeValue;
}

void setMoveForward() {
  throttleValue = midThrottle + moveThrottle;
}

void setMoveBackward() {
  throttleValue = midThrottle - moveThrottle;
}
