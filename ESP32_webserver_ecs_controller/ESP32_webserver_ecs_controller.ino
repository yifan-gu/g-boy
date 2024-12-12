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

// The reported distances and angle.
float distance = 0; // Distance of the tag to the front anchor.
float orientationTarget = 0; // [0, 360)
float orientationCar = 0; // [0, 360)

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
      lastPingTime = millis();

      if (message == "REQUEST_DATA") {
        String response = "{";
        response += "\"throttle\":" + String(throttleValue) + ",";
        response += "\"steering\":" + String(steeringValue) + ",";
        response += "\"distance\":" + String(distance) + ",";
        response += "\"orientationTarget\":" + String(orientationTarget) + ",";
        response += "\"orientationCar\":" + String(orientationCar) + ",";
        response += "\"currentX\":" + String(currentX) + ",";
        response += "\"currentY\":" + String(currentY) + ",";
        response += "\"targetX\":" + String(targetX) + ",";
        response += "\"targetY\":" + String(targetY);
        response += "}";
        client->text(response); // Send data to the requesting client
      } else if (message.startsWith("mode=")) {
        if (message == "mode=lock") {
          isLockMode = true;
        } else if (message == "mode=free") {
          isLockMode = false;
          throttleValue = midThrottle;
          steeringValue = midSteering;
        }
      } else if (message.startsWith("distance=")) {
        distance = message.substring(9).toFloat();
      } else if (message.startsWith("orientationTarget=")) {
        orientationTarget = message.substring(18).toFloat();
      } else if (message.startsWith("orientationCar=")) {
        orientationCar = message.substring(15).toFloat();
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
  // Calculate the relative angle in radians
  float relativeAngle = radians(orientationTarget - orientationCar);

  // Use trigonometry to calculate the coordinates
  currentX = distance * cos(relativeAngle);
  currentY = distance * sin(relativeAngle);

  // Serial output for debugging
  //Serial.print("Calculated coordinates (currentX, currentY): ");
  //Serial.print(currentX);
  //Serial.print(", ");
  //Serial.println(currentY)

  if (!isLockMode) { // Update the target coordinates if not in lock mode
    targetX = currentX;
    targetY = currentY;
    //Serial.print("Updated target coordinates (targetX, targetY): ");
    //Serial.print(targetX);
    //Serial.print(", ");
    //Serial.println(targetY);
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
