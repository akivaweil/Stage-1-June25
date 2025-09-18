#include "WebSocketDashboard/websocket_dashboard.h"

//* ************************************************************************
//* ********************** WEBSOCKET DASHBOARD ****************************
//* ************************************************************************
// Simple websocket dashboard for tracking cutting cycles
// Updates only when motors are not moving to avoid timing interference

// Global variables
AsyncWebServer server(WEB_SERVER_PORT);
AsyncWebSocket ws("/ws");
unsigned long cuttingCycleCount = 0;

// HTML content for the dashboard
const char* dashboardHTML = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Table Saw Dashboard</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #1a1a1a;
            color: #ffffff;
            margin: 0;
            padding: 20px;
            text-align: center;
        }
        .container {
            max-width: 600px;
            margin: 0 auto;
            background-color: #2d2d2d;
            border-radius: 10px;
            padding: 30px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
        }
        h1 {
            color: #4CAF50;
            margin-bottom: 30px;
        }
        .counter {
            font-size: 4em;
            font-weight: bold;
            color: #4CAF50;
            margin: 20px 0;
            text-shadow: 0 0 10px rgba(76, 175, 80, 0.5);
        }
        .status {
            font-size: 1.2em;
            margin: 20px 0;
            padding: 10px;
            border-radius: 5px;
        }
        .connected {
            background-color: #4CAF50;
            color: white;
        }
        .disconnected {
            background-color: #f44336;
            color: white;
        }
        .info {
            background-color: #2196F3;
            color: white;
            margin-top: 20px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🔧 Table Saw Dashboard</h1>
        <div id="status" class="status disconnected">Disconnected</div>
        <div class="counter" id="counter">0</div>
        <div class="info">
            <p>Cutting Cycles Completed</p>
            <p>Last updated: <span id="lastUpdate">Never</span></p>
        </div>
    </div>

    <script>
        let ws;
        let reconnectInterval;
        
        function connect() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.hostname}/ws`;
            
            ws = new WebSocket(wsUrl);
            
            ws.onopen = function() {
                document.getElementById('status').textContent = 'Connected';
                document.getElementById('status').className = 'status connected';
                clearInterval(reconnectInterval);
            };
            
            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                if (data.type === 'counter') {
                    document.getElementById('counter').textContent = data.count;
                    document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();
                }
            };
            
            ws.onclose = function() {
                document.getElementById('status').textContent = 'Disconnected';
                document.getElementById('status').className = 'status disconnected';
                reconnectInterval = setInterval(connect, 3000);
            };
            
            ws.onerror = function() {
                document.getElementById('status').textContent = 'Connection Error';
                document.getElementById('status').className = 'status disconnected';
            };
        }
        
        // Connect on page load
        connect();
    </script>
</body>
</html>
)rawliteral";

void setupWebSocketDashboard() {
    // Initialize SPIFFS for serving files (if needed in future)
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
    }
    
    // Setup web server to serve the dashboard
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", dashboardHTML);
    });
    
    // Setup WebSocket event handler
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    
    // Start the server
    server.begin();
    Serial.println("Web server and WebSocket started on port 80");
    
    // Print IP address for easy access
    Serial.print("Dashboard available at: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch(type) {
        case WS_EVT_CONNECT: {
            Serial.printf("Client %u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            
            // Send current counter value to newly connected client
            String message = "{\"type\":\"counter\",\"count\":" + String(cuttingCycleCount) + "}";
            client->text(message);
            break;
        }
            
        case WS_EVT_DISCONNECT:
            Serial.printf("Client %u disconnected\n", client->id());
            break;
            
        case WS_EVT_DATA:
            // Handle any incoming data if needed
            break;
            
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

void incrementCuttingCycleCounter() {
    cuttingCycleCount++;
    Serial.print("Cutting cycle completed. Total cycles: ");
    Serial.println(cuttingCycleCount);
    
    // Broadcast the updated count to all connected clients
    broadcastCuttingCycleCount();
}

unsigned long getCuttingCycleCount() {
    return cuttingCycleCount;
}

void broadcastCuttingCycleCount() {
    if (ws.getClients().size() > 0) {
        String message = "{\"type\":\"counter\",\"count\":" + String(cuttingCycleCount) + "}";
        ws.textAll(message);
    }
}
