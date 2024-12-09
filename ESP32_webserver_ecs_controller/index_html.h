const char* index_html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RC Car Controller</title>
    <style>
        html, body {
            height: 100%;
            margin: 0;
            padding: 0;
            overflow: hidden;
        }
        body {
            font-family: Arial, sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            background-color: #2c3e50;
            color: #ecf0f1;
        }
        .output-horizontal {
            position: absolute;
            top: 10px;
            left: 10px;
            font-size: 18px;
            text-align: left;
        }
        .output-vertical {
            position: absolute;
            top: 50px;
            left: 10px;
            font-size: 18px;
            text-align: left;
        }
        .slider-container-horizontal {
            position: absolute;
            bottom: 5%;
            width: 100%;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        .slider-container-vertical {
            position: absolute;
            right: 10%;
            width: 10px;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        .vertical-slider {
            -webkit-appearance: none;
            appearance: none;
            width: 400px;
            height: 50px;
            background: #bdc3c7;
            outline: none;
            border-radius: 10px;
            transform: rotate(-90deg);
            transform-origin: 50% 50%;
            cursor: pointer;
        }
        .vertical-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 60px;
            height: 60px;
            background: #3498db;
            border-radius: 50%;
            cursor: pointer;
        }
        .horizontal-slider {
            -webkit-appearance: none;
            appearance: none;
            width: 90%;
            height: 50px;
            background: #bdc3c7;
            outline: none;
            border-radius: 10px;
            cursor: pointer;
        }
        .horizontal-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 60px;
            height: 60px;
            background: #3498db;
            border-radius: 50%;
            cursor: pointer;
        }
    </style>
</head>
<body>
    <div class="output-horizontal">Steering: <span id="steering-value">0</span></div>
    <div class="output-vertical">Throttle: <span id="throttle-value">0</span></div>
    <div class="slider-container-horizontal">
        <input type="range" id="steering" min="-1000" max="1000" value="0" class="horizontal-slider">
    </div>
    <div class="slider-container-vertical">
        <input type="range" id="throttle" min="-1000" max="1000" value="0" class="vertical-slider">
    </div>
    <script>
        // WebSocket connection and heartbeat logic
        let ws;
        let reconnectInterval = 500; // Reconnection interval (ms)
        let heartbeatInterval; // Interval for sending "PING"

        const connectWebSocket = () => {
            ws = new WebSocket(`ws://${location.host}/ws`);

            ws.onopen = () => {
                console.log("WebSocket connected");

                // Start sending "PING" messages every 500ms
                clearInterval(heartbeatInterval); // Clear existing interval
                heartbeatInterval = setInterval(() => {
                    if (ws.readyState === WebSocket.OPEN) {
                        ws.send("PING"); // Send "PING" to server
                        console.log("PING sent");
                    }
                }, 500);
            };

            ws.onmessage = (event) => {
                console.log("Message from server:", event.data);

                // Handle server's "PONG" response
                if (event.data === "PONG") {
                    console.log("PONG received");
                }
            };

            ws.onclose = () => {
                console.log("WebSocket disconnected. Attempting to reconnect...");
                clearInterval(heartbeatInterval); // Stop sending "PING" when disconnected
                setTimeout(connectWebSocket, reconnectInterval); // Attempt to reconnect
            };

            ws.onerror = (error) => {
                console.error("WebSocket error:", error);
                ws.close(); // Ensure the connection is closed before reconnecting
            };
        };

        // Establish initial WebSocket connection
        connectWebSocket();

        // Slider elements and value displays
        const throttleSlider = document.getElementById("throttle");
        const throttleValue = document.getElementById("throttle-value");
        const steeringSlider = document.getElementById("steering");
        const steeringValue = document.getElementById("steering-value");

        // Send slider data via WebSocket
        const sendData = (param, value) => {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(`${param}=${value}`);
            }
        };

        // Function to smoothly reset a slider to the center
        const resetSlider = (slider, valueElement, param, duration = 300) => {
            const startValue = parseInt(slider.value, 10);
            const targetValue = 0;
            const startTime = performance.now();

            const animateReset = (currentTime) => {
                const elapsed = currentTime - startTime;
                const progress = Math.min(elapsed / duration, 1);
                const newValue = Math.round(startValue + (targetValue - startValue) * progress);

                slider.value = newValue;
                valueElement.textContent = newValue;

                // Send intermediate value
                sendData(param, newValue);

                if (progress < 1) {
                    requestAnimationFrame(animateReset);
                } else {
                    // Ensure the final value is sent as exactly 0
                    slider.value = targetValue;
                    valueElement.textContent = targetValue;
                    sendData(param, targetValue);
                }
            };

            requestAnimationFrame(animateReset);
        };

        // Event listeners for throttle slider
        throttleSlider.addEventListener("input", () => {
            const value = throttleSlider.value;
            throttleValue.textContent = value;
            sendData("throttle", value);
        });

        throttleSlider.addEventListener("mouseup", () => {
            resetSlider(throttleSlider, throttleValue, "throttle");
        });

        throttleSlider.addEventListener("touchend", () => {
            resetSlider(throttleSlider, throttleValue, "throttle");
        });

        // Event listeners for steering slider
        steeringSlider.addEventListener("input", () => {
            const value = steeringSlider.value;
            steeringValue.textContent = value;
            sendData("steering", value);
        });

        steeringSlider.addEventListener("mouseup", () => {
            resetSlider(steeringSlider, steeringValue, "steering");
        });

        steeringSlider.addEventListener("touchend", () => {
            resetSlider(steeringSlider, steeringValue, "steering");
        });
    </script>
</body>
</html>
)rawliteral";
