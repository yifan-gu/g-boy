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
        // WebSocket connection
        const ws = new WebSocket(`ws://${location.host}/ws`);

        ws.onopen = () => {
            console.log("WebSocket connected");
        };

        ws.onclose = () => {
            console.log("WebSocket disconnected");
        };

        ws.onerror = (error) => {
            console.error("WebSocket error:", error);
        };

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

        // Event listeners for sliders
        throttleSlider.addEventListener("input", () => {
            const value = throttleSlider.value;
            throttleValue.textContent = value;
            sendData("throttle", value);
        });

        steeringSlider.addEventListener("input", () => {
            const value = steeringSlider.value;
            steeringValue.textContent = value;
            sendData("steering", value);
        });
    </script>
</body>
</html>
)rawliteral";
