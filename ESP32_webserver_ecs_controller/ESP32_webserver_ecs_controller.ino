#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <ESPmDNS.h>

#include "index_html.h"


// Pin configuration
#define STEERING_PIN 3  // GPIO3 for steering
#define THROTTLE_PIN 9  // GPIO9 for throttle

// Access Point credentials
const char* ssid = "RC_Controller";
const char* password = "12345678"; // Minimum 8 characters
const char* hostname = "g-boy"; // mDNS hostname


// Async WebSocket server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Servo objects
Servo throttle;
Servo steering;

// Define throttle and steering ranges
const int minThrottle = 1000, maxThrottle = 2000, midThrottle = 1500;
const int minSteering = 1000, maxSteering = 2000, midSteering = 1500;

// Variables to store values
int throttleValue = midThrottle, steeringValue = midSteering;

// Heartbeat tracking
unsigned long lastPingTime = 0;           // Time of last received ping
const unsigned long heartbeatTimeout = 1000; // 1 second timeout

void setup() {
  // Start Serial for debugging
  Serial.begin(115200);
  while (!Serial);

  setupESC();
  setupWebServer();
}

void loop() {
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
      Serial.println("Message received: " + message);
      lastPingTime = millis();
      
      if (message == "REQUEST_DATA") {
        String response = String("{\"throttle\":") + throttleValue + ",\"steering\":" + steeringValue + "}";
        client->text(response); // Send data to the requesting client
        //Serial.println("Sent data to client: " + response);
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
   throttle.writeMicroseconds(throttleValue);
   steering.writeMicroseconds(steeringValue);

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
