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
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Table Saw Dashboard</title>
    <link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='.9em' font-size='120'>⚙</text></svg>">
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap" rel="stylesheet">
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 20px;
            overflow-x: hidden;
        }
        
        .background-animation {
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            z-index: -1;
        }
        
        .background-animation::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: radial-gradient(circle at 20% 80%, rgba(120, 119, 198, 0.3) 0%, transparent 50%),
                        radial-gradient(circle at 80% 20%, rgba(255, 255, 255, 0.1) 0%, transparent 50%),
                        radial-gradient(circle at 40% 40%, rgba(120, 119, 198, 0.2) 0%, transparent 50%);
            animation: float 40s ease-in-out infinite;
        }
        
        @keyframes float {
            0%, 100% { transform: translateY(0px) rotate(0deg); }
            50% { transform: translateY(-20px) rotate(180deg); }
        }
        
        .container {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(20px);
            border: 1px solid rgba(255, 255, 255, 0.2);
            border-radius: 24px;
            padding: 40px;
            max-width: 500px;
            width: 100%;
            box-shadow: 0 25px 50px rgba(0, 0, 0, 0.25);
            text-align: center;
            position: relative;
            overflow: hidden;
        }
        
        .container::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 1px;
            background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.4), transparent);
        }
        
        .header {
            margin-bottom: 40px;
        }
        
        .title {
            font-size: 2.5rem;
            font-weight: 700;
            background: linear-gradient(135deg, #ffffff 0%, #e0e7ff 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            margin-bottom: 8px;
            letter-spacing: -0.02em;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 12px;
        }
        
        .saw-icon {
            width: 90px;
            height: 90px;
            position: relative;
            display: flex;
            align-items: center;
            justify-content: center;
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.2);
            border-radius: 50%;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
        }
        
        .saw-icon:hover {
            background: rgba(255, 255, 255, 0.15);
            transform: translateY(-2px);
            box-shadow: 0 12px 40px rgba(0, 0, 0, 0.15);
        }
        
        .saw-icon svg {
            width: 54px;
            height: 54px;
            fill: #1e3a8a;
            filter: drop-shadow(0 2px 8px rgba(30, 58, 138, 0.3));
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
        }
        
        .saw-icon:hover svg {
            transform: rotate(15deg);
            filter: drop-shadow(0 4px 12px rgba(30, 58, 138, 0.4));
        }
        
        .subtitle {
            color: rgba(255, 255, 255, 0.7);
            font-size: 1rem;
            font-weight: 400;
        }
        
        .status-container {
            margin-bottom: 40px;
        }
        
        .status {
            display: inline-flex;
            align-items: center;
            gap: 8px;
            padding: 12px 24px;
            border-radius: 50px;
            font-size: 0.9rem;
            font-weight: 500;
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.2);
        }
        
        .status.connected {
            background: rgba(34, 197, 94, 0.2);
            color: #22c55e;
            border-color: rgba(34, 197, 94, 0.3);
        }
        
        .status.disconnected {
            background: rgba(239, 68, 68, 0.2);
            color: #ef4444;
            border-color: rgba(239, 68, 68, 0.3);
        }
        
        .status-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            animation: pulse 2s infinite;
        }
        
        .status.connected .status-dot {
            background: #22c55e;
        }
        
        .status.disconnected .status-dot {
            background: #ef4444;
        }
        
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        
        .counter-section {
            margin-bottom: 40px;
        }
        
        .counter-label {
            color: rgba(255, 255, 255, 0.8);
            font-size: 1rem;
            font-weight: 500;
            margin-bottom: 16px;
            text-transform: uppercase;
            letter-spacing: 0.05em;
        }
        
        .counter {
            font-size: 4.5rem;
            font-weight: 700;
            background: linear-gradient(135deg, #ffffff 0%, #f0f9ff 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            margin: 0;
            text-shadow: 0 0 30px rgba(255, 255, 255, 0.3);
            animation: counterGlow 3s ease-in-out infinite alternate;
        }
        
        @keyframes counterGlow {
            0% { filter: brightness(1); }
            100% { filter: brightness(1.1); }
        }
        
        .info-card {
            background: rgba(255, 255, 255, 0.05);
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 16px;
            padding: 24px;
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
        }
        
        .info-card:hover {
            background: rgba(255, 255, 255, 0.08);
            transform: translateY(-2px);
        }
        
        .info-title {
            color: rgba(255, 255, 255, 0.9);
            font-size: 1.1rem;
            font-weight: 600;
            margin-bottom: 8px;
        }
        
        .info-subtitle {
            color: rgba(255, 255, 255, 0.6);
            font-size: 0.9rem;
            font-weight: 400;
        }
        
        .last-update {
            color: rgba(255, 255, 255, 0.7);
            font-weight: 500;
        }
        
        /* Responsive design */
        @media (max-width: 640px) {
            .container {
                padding: 30px 20px;
                margin: 10px;
            }
            
            .title {
                font-size: 2rem;
            }
            
            .counter {
                font-size: 3.5rem;
            }
        }
        
        /* Loading animation */
        .loading {
            opacity: 0.7;
            animation: loading 1.5s ease-in-out infinite;
        }
        
        @keyframes loading {
            0%, 100% { opacity: 0.7; }
            50% { opacity: 1; }
        }
    </style>
</head>
<body>
    <div class="background-animation"></div>
    
    <div class="container">
        <div class="header">
            <h1 class="title">
                <div class="saw-icon">
                    <svg viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                        <path d="M12 15.5A3.5 3.5 0 0 1 8.5 12A3.5 3.5 0 0 1 12 8.5a3.5 3.5 0 0 1 3.5 3.5a3.5 3.5 0 0 1-3.5 3.5m7.43-2.53c.04-.32.07-.64.07-.97c0-.33-.03-.66-.07-1l2.11-1.63c.19-.15.24-.42.12-.64l-2-3.46c-.12-.22-.39-.31-.61-.22l-2.49 1c-.52-.39-1.06-.73-1.69-.98l-.37-2.65A.506.506 0 0 0 14 2h-4c-.25 0-.46.18-.5.42l-.37 2.65c-.63.25-1.17.59-1.69.98l-2.49-1c-.22-.09-.49 0-.61.22l-2 3.46c-.13.22-.07.49.12.64L4.57 11c-.04.34-.07.67-.07 1c0 .33.03.65.07.97l-2.11 1.66c-.19.15-.25.42-.12.64l2 3.46c.12.22.39.3.61.22l2.49-1.01c.52.4 1.06.74 1.69.99l.37 2.65c.04.24.25.42.5.42h4c.25 0 .46-.18.5-.42l.37-2.65c.63-.26 1.17-.59 1.69-.99l2.49 1.01c.22.08.49 0 .61-.22l2-3.46c.12-.22.07-.49-.12-.64l-2.11-1.66Z"/>
                    </svg>
                </div>
                Table Saw
            </h1>
            <p class="subtitle">Real-time Monitoring Dashboard</p>
        </div>
        
        <div class="status-container">
            <div id="status" class="status disconnected">
                <div class="status-dot"></div>
                <span>Disconnected</span>
            </div>
        </div>
        
        <div class="counter-section">
            <div class="counter-label">Cutting Cycles</div>
            <div class="counter" id="counter">0</div>
        </div>
        
        <div class="info-card">
            <div class="info-title">System Status</div>
            <div class="info-subtitle">
                Last cut: <span class="last-update" id="lastUpdate">Never</span>
            </div>
        </div>
    </div>

    <script>
        let ws;
        let reconnectInterval;
        let isConnected = false;
        let reconnectAttempts = 0;
        const maxReconnectAttempts = 10;
        
        function updateStatus(message, isConnected) {
            const statusEl = document.getElementById('status');
            statusEl.innerHTML = `<div class="status-dot"></div><span>${message}</span>`;
            statusEl.className = `status ${isConnected ? 'connected' : 'disconnected'}`;
        }
        
        function connect() {
            // Show connecting status immediately
            updateStatus('Connecting...', false);
            
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.hostname}/ws`;
            
            ws = new WebSocket(wsUrl);
            
            ws.onopen = function() {
                isConnected = true;
                reconnectAttempts = 0;
                updateStatus('Connected', true);
                clearInterval(reconnectInterval);
                reconnectInterval = null;
            };
            
            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                if (data.type === 'counter') {
                    const counterEl = document.getElementById('counter');
                    const lastUpdateEl = document.getElementById('lastUpdate');
                    
                    // Add loading animation during update
                    counterEl.classList.add('loading');
                    
                    setTimeout(() => {
                        counterEl.textContent = data.count;
                        counterEl.classList.remove('loading');
                        lastUpdateEl.textContent = new Date().toLocaleTimeString();
                    }, 150);
                }
            };
            
            ws.onclose = function() {
                isConnected = false;
                updateStatus('Disconnected', false);
                
                // Exponential backoff for reconnection
                if (reconnectAttempts < maxReconnectAttempts) {
                    reconnectAttempts++;
                    const delay = Math.min(1000 * Math.pow(1.5, reconnectAttempts), 5000);
                    
                    setTimeout(() => {
                        if (!isConnected) {
                            connect();
                        }
                    }, delay);
                } else {
                    updateStatus('Connection Failed', false);
                }
            };
            
            ws.onerror = function() {
                isConnected = false;
                updateStatus('Connection Error', false);
            };
        }
        
        // Connect on page load
        connect();
        
        // Add some interactive effects
        document.addEventListener('DOMContentLoaded', function() {
            const container = document.querySelector('.container');
            
            // Add subtle hover effects
            container.addEventListener('mouseenter', function() {
                this.style.transform = 'translateY(-5px)';
            });
            
            container.addEventListener('mouseleave', function() {
                this.style.transform = 'translateY(0)';
            });
        });
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
