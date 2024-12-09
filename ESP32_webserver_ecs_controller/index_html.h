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
            transform: rotate(180deg); /* Flip the slider */
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
    <div class="output-horizontal">Throttle: <span id="throttle-value">1500</span>, Steering: <span id="steering-value">1500</span></div>
    <div class="slider-container-horizontal">
        <input type="range" id="steering" min="1000" max="2000" value="1500" class="horizontal-slider">
    </div>
    <div class="slider-container-vertical">
        <input type="range" id="throttle" min="1000" max="2000" value="1500" class="vertical-slider">
    </div>
    <script>
        // WebSocket connection and polling logic
        let ws;
        const reconnectInterval = 500; // Reconnection interval (ms)
        const pollInterval = 100; // Polling interval (ms)

        const throttleValue = document.getElementById("throttle-value");
        const steeringValue = document.getElementById("steering-value");
        const throttleSlider = document.getElementById("throttle");
        const steeringSlider = document.getElementById("steering");

        const connectWebSocket = () => {
            ws = new WebSocket(`ws://${location.host}/ws`);

            ws.onopen = () => {
                console.log("WebSocket connected");

                // Start polling server for data every `pollInterval` ms
                setInterval(() => {
                    if (ws.readyState === WebSocket.OPEN) {
                        ws.send("REQUEST_DATA"); // Send request to server
                    }
                }, pollInterval);
            };

            ws.onmessage = (event) => {
                console.log("Message from server:", event.data);

                // Parse the JSON response and update UI
                try {
                    const data = JSON.parse(event.data);
                    if (data.throttle !== undefined) {
                        throttleValue.textContent = data.throttle;
                    }
                    if (data.steering !== undefined) {
                        steeringValue.textContent = data.steering;
                    }
                } catch (err) {
                    console.error("Error parsing message:", err);
                }
            };

            ws.onclose = () => {
                console.log("WebSocket disconnected. Attempting to reconnect...");
                setTimeout(connectWebSocket, reconnectInterval);
            };

            ws.onerror = (error) => {
                console.error("WebSocket error:", error);
                ws.close(); // Ensure the connection is closed before reconnecting
            };
        };

        // Establish initial WebSocket connection
        connectWebSocket();

        // Function to smoothly reset a slider to the center
        const resetSlider = (slider, param, duration = 300) => {
            const startValue = parseInt(slider.value, 10);
            const targetValue = 1500;
            const startTime = performance.now();

            const animateReset = (currentTime) => {
                const elapsed = currentTime - startTime;
                const progress = Math.min(elapsed / duration, 1);
                const newValue = Math.round(startValue + (targetValue - startValue) * progress);

                slider.value = newValue;

                // Send intermediate value
                if (ws.readyState === WebSocket.OPEN) {
                    ws.send(`${param}=${newValue}`);
                }

                if (progress < 1) {
                    requestAnimationFrame(animateReset);
                } else {
                    // Ensure the final value is sent as exactly 1500
                    slider.value = targetValue;
                    if (ws.readyState === WebSocket.OPEN) {
                        ws.send(`${param}=${targetValue}`);
                    }
                }
            };

            requestAnimationFrame(animateReset);
        };

        // Event listeners for throttle slider
        throttleSlider.addEventListener("input", () => {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(`throttle=${throttleSlider.value}`);
            }
        });

        throttleSlider.addEventListener("mouseup", () => {
            resetSlider(throttleSlider, "throttle");
        });

        throttleSlider.addEventListener("touchend", () => {
            resetSlider(throttleSlider, "throttle");
        });

        // Event listeners for steering slider
        steeringSlider.addEventListener("input", () => {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(`steering=${steeringSlider.value}`);
            }
        });

        steeringSlider.addEventListener("mouseup", () => {
            resetSlider(steeringSlider, "steering");
        });

        steeringSlider.addEventListener("touchend", () => {
            resetSlider(steeringSlider, "steering");
        });
    </script>
</body>
</html>
)rawliteral";
