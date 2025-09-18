#ifndef DASHBOARD_HTML_H
#define DASHBOARD_HTML_H

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
            max-width: 1400px;
            margin: 0 auto;
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
        }
        
        .card {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(20px);
            border: 1px solid rgba(255, 255, 255, 0.2);
            border-radius: 16px;
            padding: 24px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
        }
        
        .card:hover {
            background: rgba(255, 255, 255, 0.15);
            transform: translateY(-2px);
        }
        
        .card-header {
            display: flex;
            align-items: center;
            gap: 12px;
            margin-bottom: 20px;
            padding-bottom: 12px;
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
        }
        
        .card-icon {
            width: 40px;
            height: 40px;
            background: rgba(255, 255, 255, 0.1);
            border-radius: 8px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 20px;
        }
        
        .card-title {
            color: #ffffff;
            font-size: 1.2rem;
            font-weight: 600;
        }
        
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
            gap: 12px;
        }
        
        .status-item {
            background: rgba(255, 255, 255, 0.05);
            border-radius: 8px;
            padding: 12px;
            text-align: center;
            transition: all 0.3s ease;
        }
        
        .status-item.active {
            background: rgba(34, 197, 94, 0.2);
            border: 1px solid rgba(34, 197, 94, 0.3);
        }
        
        .status-item.inactive {
            background: rgba(107, 114, 128, 0.2);
            border: 1px solid rgba(107, 114, 128, 0.3);
        }
        
        /* LED-specific colors */
        .status-item.led-red.active {
            background: rgba(239, 68, 68, 0.3);
            border: 1px solid rgba(239, 68, 68, 0.5);
            box-shadow: 0 0 10px rgba(239, 68, 68, 0.3);
        }
        
        .status-item.led-yellow.active {
            background: rgba(245, 158, 11, 0.3);
            border: 1px solid rgba(245, 158, 11, 0.5);
            box-shadow: 0 0 10px rgba(245, 158, 11, 0.3);
        }
        
        .status-item.led-green.active {
            background: rgba(34, 197, 94, 0.3);
            border: 1px solid rgba(34, 197, 94, 0.5);
            box-shadow: 0 0 10px rgba(34, 197, 94, 0.3);
        }
        
        .status-item.led-blue.active {
            background: rgba(59, 130, 246, 0.3);
            border: 1px solid rgba(59, 130, 246, 0.5);
            box-shadow: 0 0 10px rgba(59, 130, 246, 0.3);
        }
        
        .status-label {
            color: rgba(255, 255, 255, 0.7);
            font-size: 0.8rem;
            margin-bottom: 4px;
        }
        
        .status-value {
            color: #ffffff;
            font-size: 1rem;
            font-weight: 600;
        }
        
        .metric-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 16px;
        }
        
        .metric-item {
            text-align: center;
        }
        
        .metric-value {
            color: #ffffff;
            font-size: 2rem;
            font-weight: 700;
            margin-bottom: 4px;
        }
        
        .metric-label {
            color: rgba(255, 255, 255, 0.7);
            font-size: 0.9rem;
        }
        
        .connection-status {
            position: fixed;
            top: 20px;
            right: 20px;
            background: rgba(0, 0, 0, 0.8);
            color: white;
            padding: 12px 20px;
            border-radius: 8px;
            font-size: 0.9rem;
            z-index: 1000;
        }
        
        .connection-status.connected {
            background: rgba(34, 197, 94, 0.9);
        }
        
        .connection-status.disconnected {
            background: rgba(239, 68, 68, 0.9);
        }
        
        .event-log {
            max-height: 300px;
            overflow-y: auto;
            background: rgba(0, 0, 0, 0.2);
            border-radius: 8px;
            padding: 12px;
        }
        
        .event-item {
            color: rgba(255, 255, 255, 0.8);
            font-size: 0.8rem;
            margin-bottom: 4px;
            padding: 4px 8px;
            border-radius: 4px;
            background: rgba(255, 255, 255, 0.05);
        }
        
        .full-width {
            grid-column: 1 / -1;
        }
        
        .chart-container {
            height: 200px;
            background: rgba(0, 0, 0, 0.2);
            border-radius: 8px;
            display: flex;
            align-items: center;
            justify-content: center;
            color: rgba(255, 255, 255, 0.7);
        }
        
        @media (max-width: 768px) {
            .container {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="background-animation"></div>
    
    <div class="connection-status disconnected" id="connectionStatus">
        <span id="connectionText">Disconnected</span>
    </div>
    
    <div class="container">
        <!-- System Status Card -->
        <div class="card">
            <div class="card-header">
                <div class="card-icon">⚙️</div>
                <div class="card-title">System Status</div>
            </div>
            <div class="status-grid">
                <div class="status-item">
                    <div class="status-label">Current State</div>
                    <div class="status-value" id="currentState">-</div>
                </div>
                <div class="status-item">
                    <div class="status-label">System Health</div>
                    <div class="status-value" id="systemHealth">-</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Uptime</div>
                    <div class="status-value" id="uptime">-</div>
                </div>
            </div>
        </div>
        
        <!-- Performance Metrics Card -->
        <div class="card">
            <div class="card-header">
                <div class="card-icon">📊</div>
                <div class="card-title">Performance</div>
            </div>
            <div class="metric-grid">
                <div class="metric-item">
                    <div class="metric-value" id="cuttingCycles">0</div>
                    <div class="metric-label">Total Cycles</div>
                </div>
                <div class="metric-item">
                    <div class="metric-value" id="lastCycleTime">-</div>
                    <div class="metric-label">Last Cycle (ms)</div>
                </div>
                <div class="metric-item">
                    <div class="metric-value" id="averageCycleTime">-</div>
                    <div class="metric-label">Avg Cycle (ms)</div>
                </div>
                <div class="metric-item">
                    <div class="metric-value" id="efficiency">-</div>
                    <div class="metric-label">Efficiency %</div>
                </div>
            </div>
        </div>
        
        <!-- Sensor Status Card -->
        <div class="card">
            <div class="card-header">
                <div class="card-icon">🔍</div>
                <div class="card-title">Sensors</div>
            </div>
            <div class="status-grid">
                <div class="status-item" id="sensor_2x4">
                    <div class="status-label">2x4 Present</div>
                    <div class="status-value">-</div>
                </div>
                <div class="status-item" id="sensor_suction">
                    <div class="status-label">Wood Suction</div>
                    <div class="status-value">-</div>
                </div>
                <div class="status-item" id="sensor_firstcut">
                    <div class="status-label">First Cut</div>
                    <div class="status-value">-</div>
                </div>
                <div class="status-item" id="sensor_cuthome">
                    <div class="status-label">Cut Home</div>
                    <div class="status-value">-</div>
                </div>
                <div class="status-item" id="sensor_feedhome">
                    <div class="status-label">Feed Home</div>
                    <div class="status-value">-</div>
                </div>
                <div class="status-item" id="sensor_reload">
                    <div class="status-label">Reload</div>
                    <div class="status-value">-</div>
                </div>
            </div>
        </div>
        
        <!-- Clamp Status Card -->
        <div class="card">
            <div class="card-header">
                <div class="card-icon">🔧</div>
                <div class="card-title">Clamps</div>
            </div>
            <div class="status-grid">
                <div class="status-item" id="clamp_feed">
                    <div class="status-label">Feed Clamp</div>
                    <div class="status-value">-</div>
                </div>
                <div class="status-item" id="clamp_2x4">
                    <div class="status-label">2x4 Secure</div>
                    <div class="status-value">-</div>
                </div>
                <div class="status-item" id="clamp_rotation">
                    <div class="status-label">Rotation</div>
                    <div class="status-value">-</div>
                </div>
            </div>
        </div>
        
        <!-- LED Status Card -->
        <div class="card">
            <div class="card-header">
                <div class="card-icon">💡</div>
                <div class="card-title">Status LEDs</div>
            </div>
            <div class="status-grid">
                <div class="status-item" id="led_red">
                    <div class="status-label">Red</div>
                    <div class="status-value">-</div>
                </div>
                <div class="status-item" id="led_yellow">
                    <div class="status-label">Yellow</div>
                    <div class="status-value">-</div>
                </div>
                <div class="status-item" id="led_green">
                    <div class="status-label">Green</div>
                    <div class="status-value">-</div>
                </div>
                <div class="status-item" id="led_blue">
                    <div class="status-label">Blue</div>
                    <div class="status-value">-</div>
                </div>
            </div>
        </div>
        
        <!-- Network Info Card -->
        <div class="card">
            <div class="card-header">
                <div class="card-icon">📡</div>
                <div class="card-title">Network & System</div>
            </div>
            <div class="status-grid">
                <div class="status-item">
                    <div class="status-label">WiFi Signal</div>
                    <div class="status-value" id="wifiSignal">-</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Free Heap</div>
                    <div class="status-value" id="freeHeap">-</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Temperature</div>
                    <div class="status-value" id="temperature">-</div>
                </div>
            </div>
        </div>
        
        <!-- Error Status Card -->
        <div class="card">
            <div class="card-header">
                <div class="card-icon">⚠️</div>
                <div class="card-title">Error Status</div>
            </div>
            <div class="status-grid">
                <div class="status-item">
                    <div class="status-label">Last Error</div>
                    <div class="status-value" id="lastError">None</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Error Count</div>
                    <div class="status-value" id="errorCount">0</div>
                </div>
            </div>
        </div>
        
        <!-- Event Log Card -->
        <div class="card full-width">
            <div class="card-header">
                <div class="card-icon">📝</div>
                <div class="card-title">Event Log</div>
            </div>
            <div class="event-log" id="eventLog">
                <div class="event-item">System initializing...</div>
            </div>
        </div>
        
        <!-- Performance Chart Card -->
        <div class="card full-width">
            <div class="card-header">
                <div class="card-icon">📈</div>
                <div class="card-title">Performance Chart</div>
            </div>
            <div class="chart-container">
                Cycle time chart will be displayed here
            </div>
        </div>
    </div>

    <script>
        let ws;
        let reconnectTimeout;
        let heartbeatInterval;
        let isConnected = false;
        let reconnectAttempts = 0;
        const maxReconnectAttempts = 20;
        const heartbeatIntervalMs = 1000;
        
        function updateConnectionStatus(connected, message) {
            const statusEl = document.getElementById('connectionStatus');
            const textEl = document.getElementById('connectionText');
            
            statusEl.className = `connection-status ${connected ? 'connected' : 'disconnected'}`;
            textEl.textContent = message;
        }
        
        function startHeartbeat() {
            if (heartbeatInterval) {
                clearInterval(heartbeatInterval);
            }
            
            heartbeatInterval = setInterval(() => {
                if (ws && ws.readyState === WebSocket.OPEN) {
                    try {
                        ws.send(JSON.stringify({type: 'ping'}));
                    } catch (error) {
                        console.log('Error sending ping:', error);
                        forceDisconnect();
                    }
                } else {
                    forceDisconnect();
                }
            }, heartbeatIntervalMs);
        }
        
        function forceDisconnect() {
            isConnected = false;
            if (heartbeatInterval) {
                clearInterval(heartbeatInterval);
                heartbeatInterval = null;
            }
            if (ws) {
                ws.close();
                ws = null;
            }
            updateConnectionStatus(false, 'Disconnected');
            attemptReconnect();
        }
        
        function attemptReconnect() {
            if (reconnectAttempts >= maxReconnectAttempts) {
                updateConnectionStatus(false, 'Connection Failed - Click to Retry');
                return;
            }
            
            reconnectAttempts++;
            const delay = Math.min(500 + (reconnectAttempts * 200), 3000);
            
            updateConnectionStatus(false, 'Reconnecting...');
            
            reconnectTimeout = setTimeout(() => {
                if (!isConnected) {
                    connect();
                }
            }, delay);
        }
        
        function connect() {
            if (reconnectTimeout) {
                clearTimeout(reconnectTimeout);
                reconnectTimeout = null;
            }
            
            updateConnectionStatus(false, 'Connecting...');
            
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.hostname}/ws`;
            
            ws = new WebSocket(wsUrl);
            
            const connectionTimeout = setTimeout(() => {
                if (ws.readyState === WebSocket.CONNECTING) {
                    ws.close();
                    forceDisconnect();
                }
            }, 2000);
            
            ws.onopen = function() {
                clearTimeout(connectionTimeout);
                isConnected = true;
                reconnectAttempts = 0;
                updateConnectionStatus(true, 'Connected');
                startHeartbeat();
            };
            
            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                
                if (data.type === 'pong') {
                    return;
                }
                
                if (data.type === 'counter') {
                    document.getElementById('cuttingCycles').textContent = data.count;
                }
                
                if (data.type === 'system_status') {
                    document.getElementById('currentState').textContent = data.currentState;
                    document.getElementById('systemHealth').textContent = data.systemHealth;
                    document.getElementById('uptime').textContent = formatUptime(data.uptime);
                }
                
                if (data.type === 'sensor_status') {
                    updateSensorStatus(data);
                }
                
                if (data.type === 'clamp_status') {
                    updateClampStatus(data);
                }
                
                if (data.type === 'led_status') {
                    updateLEDStatus(data);
                }
                
                if (data.type === 'performance_metrics') {
                    document.getElementById('lastCycleTime').textContent = data.lastCycleTime || '-';
                    document.getElementById('averageCycleTime').textContent = data.averageCycleTime || '-';
                    document.getElementById('efficiency').textContent = data.efficiency ? data.efficiency.toFixed(1) + '%' : '-';
                }
                
                if (data.type === 'error_status') {
                    document.getElementById('lastError').textContent = data.lastError;
                    document.getElementById('errorCount').textContent = data.errorCount;
                }
                
                if (data.type === 'network_info') {
                    document.getElementById('wifiSignal').textContent = data.wifiSignal + ' dBm';
                    document.getElementById('freeHeap').textContent = Math.round(data.freeHeap / 1024) + ' KB';
                    document.getElementById('temperature').textContent = data.temperature ? data.temperature.toFixed(1) + '°C' : '-';
                }
                
                if (data.type === 'event_log') {
                    updateEventLog(data.events);
                }
            };
            
            ws.onclose = function(event) {
                clearTimeout(connectionTimeout);
                if (isConnected) {
                    forceDisconnect();
                }
            };
            
            ws.onerror = function(error) {
                clearTimeout(connectionTimeout);
                forceDisconnect();
            };
        }
        
        function updateSensorStatus(data) {
            updateStatusItem('sensor_2x4', data._2x4Present);
            updateStatusItem('sensor_suction', data.woodSuctionConfirm);
            updateStatusItem('sensor_firstcut', data.firstCutOrWoodFwdOne);
            updateStatusItem('sensor_cuthome', data.cutMotorHomeSwitch);
            updateStatusItem('sensor_feedhome', data.feedMotorHomeSensor);
            updateStatusItem('sensor_reload', data.reloadSwitch);
        }
        
        function updateClampStatus(data) {
            updateStatusItem('clamp_feed', data.feedClamp);
            updateStatusItem('clamp_2x4', data._2x4SecureClamp);
            updateStatusItem('clamp_rotation', data.rotationClamp);
        }
        
        function updateLEDStatus(data) {
            updateLEDItem('led_red', data.red, 'led-red');
            updateLEDItem('led_yellow', data.yellow, 'led-yellow');
            updateLEDItem('led_green', data.green, 'led-green');
            updateLEDItem('led_blue', data.blue, 'led-blue');
        }
        
        function updateStatusItem(elementId, isActive) {
            const element = document.getElementById(elementId);
            const valueElement = element.querySelector('.status-value');
            
            element.className = `status-item ${isActive ? 'active' : 'inactive'}`;
            valueElement.textContent = isActive ? 'ON' : 'OFF';
        }
        
        function updateLEDItem(elementId, isActive, ledClass) {
            const element = document.getElementById(elementId);
            const valueElement = element.querySelector('.status-value');
            
            element.className = `status-item ${ledClass} ${isActive ? 'active' : 'inactive'}`;
            valueElement.textContent = isActive ? 'ON' : 'OFF';
        }
        
        function updateEventLog(events) {
            const logContainer = document.getElementById('eventLog');
            logContainer.innerHTML = '';
            
            events.forEach(event => {
                const eventItem = document.createElement('div');
                eventItem.className = 'event-item';
                eventItem.textContent = event;
                logContainer.appendChild(eventItem);
            });
            
            logContainer.scrollTop = logContainer.scrollHeight;
        }
        
        function formatUptime(ms) {
            const seconds = Math.floor(ms / 1000);
            const minutes = Math.floor(seconds / 60);
            const hours = Math.floor(minutes / 60);
            const days = Math.floor(hours / 24);
            
            if (days > 0) return `${days}d ${hours % 24}h`;
            if (hours > 0) return `${hours}h ${minutes % 60}m`;
            if (minutes > 0) return `${minutes}m ${seconds % 60}s`;
            return `${seconds}s`;
        }
        
        // Connect on page load
        connect();
        
        // Handle manual reconnection
        document.getElementById('connectionStatus').addEventListener('click', function() {
            if (!isConnected) {
                reconnectAttempts = 0;
                connect();
            }
        });
        
        // Request initial data
        setInterval(() => {
            if (isConnected && ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({type: 'request_all_data'}));
            }
        }, 5000);
    </script>
</body>
</html>
)rawliteral";

#endif // DASHBOARD_HTML_H
