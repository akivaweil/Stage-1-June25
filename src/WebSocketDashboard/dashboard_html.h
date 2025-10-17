#ifndef DASHBOARD_HTML_H
#define DASHBOARD_HTML_H

#include <Arduino.h>

const char dashboardHTML[] PROGMEM = R"rawliteral(
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
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%) !important;
            min-height: 100vh;
            padding: 24px;
            overflow-x: hidden;
            line-height: 1.6;
            display: block !important;
            visibility: visible !important;
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
        
        @keyframes fadeInUp {
            from {
                opacity: 0;
                transform: translateY(20px);
            }
            to {
                opacity: 1;
                transform: translateY(0);
            }
        }
        
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.7; }
        }
        
        .container {
            max-width: 1400px;
            margin: 0 auto;
            display: grid !important;
            grid-template-columns: 1fr 1fr;
            gap: 24px;
            padding: 0 8px;
            animation: fadeInUp 0.8s ease-out;
            visibility: visible !important;
        }
        
        .card {
            background: rgba(255, 255, 255, 0.12);
            backdrop-filter: blur(24px);
            border: 1px solid rgba(255, 255, 255, 0.25);
            border-radius: 20px;
            padding: 28px;
            box-shadow: 0 12px 40px rgba(0, 0, 0, 0.15), 
                        0 4px 16px rgba(0, 0, 0, 0.1),
                        inset 0 1px 0 rgba(255, 255, 255, 0.1);
            transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
            position: relative;
            overflow: hidden;
        }
        
        .card:hover {
            background: rgba(255, 255, 255, 0.18);
            transform: translateY(-4px);
            box-shadow: 0 16px 48px rgba(0, 0, 0, 0.2), 
                        0 8px 24px rgba(0, 0, 0, 0.15),
                        inset 0 1px 0 rgba(255, 255, 255, 0.15);
        }
        
        .card-header {
            display: flex;
            align-items: center;
            gap: 16px;
            margin-bottom: 24px;
            padding-bottom: 16px;
            border-bottom: 1px solid rgba(255, 255, 255, 0.15);
        }
        
        .card-icon {
            width: 44px;
            height: 44px;
            background: rgba(255, 255, 255, 0.15);
            border-radius: 12px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 22px;
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
        }
        
        .card-title {
            color: #ffffff;
            font-size: 1.3rem;
            font-weight: 600;
            letter-spacing: -0.02em;
        }
        
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(130px, 1fr));
            gap: 16px;
        }
        
        .status-item {
            background: rgba(255, 255, 255, 0.08);
            border-radius: 12px;
            padding: 16px;
            text-align: center;
            transition: all 0.3s ease;
            border: 1px solid rgba(255, 255, 255, 0.1);
            position: relative;
        }
        
        .status-item.active {
            background: rgba(34, 197, 94, 0.25);
            border: 1px solid rgba(34, 197, 94, 0.4);
            box-shadow: 0 4px 16px rgba(34, 197, 94, 0.2);
        }
        
        .status-item.inactive {
            background: rgba(107, 114, 128, 0.15);
            border: 1px solid rgba(107, 114, 128, 0.25);
        }
        
        
        .status-label {
            color: rgba(255, 255, 255, 0.8);
            font-size: 0.85rem;
            margin-bottom: 6px;
            font-weight: 500;
        }
        
        .status-value {
            color: #ffffff;
            font-size: 1.1rem;
            font-weight: 700;
            letter-spacing: -0.01em;
            transition: all 0.3s ease;
        }
        
        .status-value.loading {
            animation: pulse 1.5s ease-in-out infinite;
        }
        
        .metric-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
            gap: 20px;
        }
        
        .metric-item {
            text-align: center;
        }
        
        .metric-value {
            color: #ffffff;
            font-size: 2.2rem;
            font-weight: 700;
            margin-bottom: 6px;
            letter-spacing: -0.02em;
        }
        
        .metric-label {
            color: rgba(255, 255, 255, 0.8);
            font-size: 0.95rem;
            font-weight: 500;
        }
        
        .metric-item.ghosted {
            opacity: 0.3;
            pointer-events: none;
        }
        
        .metric-item.ghosted .metric-value {
            color: rgba(255, 255, 255, 0.4);
        }
        
        .metric-item.ghosted .metric-label {
            color: rgba(255, 255, 255, 0.4);
        }
        
        .connection-status {
            position: fixed;
            top: 24px;
            right: 24px;
            background: rgba(0, 0, 0, 0.85);
            color: white;
            padding: 14px 24px;
            border-radius: 12px;
            font-size: 0.95rem;
            font-weight: 500;
            z-index: 1000;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.1);
            box-shadow: 0 8px 24px rgba(0, 0, 0, 0.3);
            cursor: pointer;
            transition: all 0.3s ease;
        }
        
        .connection-status.disconnected {
            background: rgba(239, 68, 68, 0.9);
            box-shadow: 0 8px 24px rgba(239, 68, 68, 0.3);
            font-size: 1.9rem;
            padding: 28px 48px;
            font-weight: 700;
        }
        
        .connection-status.connected {
            background: rgba(34, 197, 94, 0.9);
            box-shadow: 0 8px 24px rgba(34, 197, 94, 0.3);
        }
        
        
        .connection-status:hover {
            transform: translateY(-2px);
        }
        
        
        .event-log {
            max-height: 320px;
            overflow-y: auto;
            background: rgba(0, 0, 0, 0.25);
            border-radius: 12px;
            padding: 16px;
            border: 1px solid rgba(255, 255, 255, 0.1);
        }
        
        .event-log::-webkit-scrollbar {
            width: 6px;
        }
        
        .event-log::-webkit-scrollbar-track {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 3px;
        }
        
        .event-log::-webkit-scrollbar-thumb {
            background: rgba(255, 255, 255, 0.3);
            border-radius: 3px;
        }
        
        .event-log::-webkit-scrollbar-thumb:hover {
            background: rgba(255, 255, 255, 0.5);
        }
        
        .event-item {
            color: rgba(255, 255, 255, 0.9);
            font-size: 0.9rem;
            margin-bottom: 8px;
            padding: 12px 16px;
            border-radius: 8px;
            background: rgba(255, 255, 255, 0.1);
            border-left: 4px solid rgba(59, 130, 246, 0.6);
            display: flex;
            align-items: center;
            transition: all 0.3s ease;
            font-weight: 500;
        }
        
        .event-item:hover {
            background: rgba(255, 255, 255, 0.15);
            transform: translateX(4px);
        }
        
        .event-item.error {
            border-left-color: rgba(239, 68, 68, 0.8);
            background: rgba(239, 68, 68, 0.1);
        }
        
        .event-item.error:hover {
            background: rgba(239, 68, 68, 0.15);
        }
        
        .event-item.state-change {
            border-left-color: rgba(34, 197, 94, 0.8);
            background: rgba(34, 197, 94, 0.08);
        }
        
        .event-item.state-change:hover {
            background: rgba(34, 197, 94, 0.12);
        }
        
        .event-item.system {
            border-left-color: rgba(168, 85, 247, 0.8);
            background: rgba(168, 85, 247, 0.08);
        }
        
        .event-item.system:hover {
            background: rgba(168, 85, 247, 0.12);
        }
        
        .event-item.performance {
            border-left-color: rgba(245, 158, 11, 0.8);
            background: rgba(245, 158, 11, 0.08);
        }
        
        .event-item.performance:hover {
            background: rgba(245, 158, 11, 0.12);
        }
        
        .event-icon {
            margin-right: 8px;
            font-size: 0.9rem;
            min-width: 16px;
        }
        
        .event-timestamp {
            color: rgba(255, 255, 255, 0.6);
            font-size: 0.75rem;
            margin-right: 8px;
            min-width: 70px;
            font-family: 'Courier New', monospace;
        }
        
        .event-message {
            flex: 1;
        }
        
        .full-width {
            grid-column: 1 / -1;
        }
        
        .chart-container {
            height: 220px;
            background: rgba(0, 0, 0, 0.25);
            border-radius: 12px;
            display: flex;
            align-items: center;
            justify-content: center;
            color: rgba(255, 255, 255, 0.8);
            border: 1px solid rgba(255, 255, 255, 0.1);
            font-size: 1.1rem;
            font-weight: 500;
        }
        
        @media (max-width: 768px) {
            .container {
                grid-template-columns: 1fr;
                gap: 20px;
                padding: 0 4px;
            }
            
            .card {
                padding: 20px;
            }
            
            .connection-status {
                top: 16px;
                right: 16px;
                padding: 12px 20px;
                font-size: 0.9rem;
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
                    <div class="status-label">Uptime</div>
                    <div class="status-value" id="uptime">-</div>
                </div>
            </div>
        </div>
        
        <!-- Performance Metrics Card -->
        <div class="card full-width">
            <div class="card-header">
                <div class="card-icon">📊</div>
                <div class="card-title">Performance Metrics</div>
            </div>
            
            <div style="text-align: center; padding: 24px; background: rgba(255, 255, 255, 0.1); border-radius: 16px; border: 1px solid rgba(255, 255, 255, 0.2);">
                <div style="color: #ffffff; font-size: 3rem; font-weight: 800; margin-bottom: 8px; letter-spacing: -0.02em;" id="reloadTime">-</div>
                <div style="color: rgba(255, 255, 255, 0.9); font-size: 1.1rem; font-weight: 600; letter-spacing: 0.05em;">RELOAD TIME</div>
            </div>
            
            <!-- Time-based Performance Breakdown -->
            <div style="margin-top: 24px;">
                <h3 style="color: rgba(255, 255, 255, 0.9); font-size: 1.1rem; font-weight: 600; margin-bottom: 16px; text-align: center;">Cycles Over Time</h3>
                <div class="metric-grid" style="grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 16px;">
                    <div class="metric-item">
                        <div class="metric-value" id="avgCycles1Min" style="font-size: 2.2rem;">0.0</div>
                        <div class="metric-label">1 Min Avg/min</div>
                    </div>
                    <div class="metric-item">
                        <div class="metric-value" id="avgCycles3Min" style="font-size: 2.2rem;">0.0</div>
                        <div class="metric-label">3 Min Avg/min</div>
                    </div>
                    <div class="metric-item">
                        <div class="metric-value" id="avgCycles5Min" style="font-size: 2.2rem;">0.0</div>
                        <div class="metric-label">5 Min Avg/min</div>
                    </div>
                    <div class="metric-item">
                        <div class="metric-value" id="avgCycles15Min" style="font-size: 2.2rem;">0.0</div>
                        <div class="metric-label">15 Min Avg/min</div>
                    </div>
                    <div class="metric-item">
                        <div class="metric-value" id="avgCycles30Min" style="font-size: 2.2rem;">0.0</div>
                        <div class="metric-label">30 Min Avg/min</div>
                    </div>
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
                    <div class="status-label">Cut Motor Errors</div>
                    <div class="status-value" id="cutMotorErrorCount">0</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Suction Errors</div>
                    <div class="status-value" id="suctionErrorCount">0</div>
                </div>
            </div>
        </div>
        
        <!-- Configuration Card -->
        <div class="card full-width">
            <div class="card-header">
                <div class="card-icon">⚙️</div>
                <div class="card-title">Configuration Settings</div>
            </div>
            <div style="padding: 16px;">
                <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 24px;">
                    
                    <!-- Configuration Section -->
                    <div style="background: rgba(255, 255, 255, 0.05); border-radius: 12px; padding: 20px; border: 1px solid rgba(255, 255, 255, 0.1); margin: 0 auto; max-width: 400px;">
                        <h3 style="color: rgba(255, 255, 255, 0.9); font-size: 1.1rem; font-weight: 600; margin-bottom: 16px; border-bottom: 1px solid rgba(255, 255, 255, 0.1); padding-bottom: 8px;">Configuration Settings</h3>
                        
                        <div style="margin-bottom: 16px;">
                            <label style="display: block; color: rgba(255, 255, 255, 0.8); font-size: 0.9rem; font-weight: 500; margin-bottom: 6px;">Cut Travel Distance (inches)</label>
                            <div style="display: flex; gap: 8px; align-items: center;">
                                <input type="number" id="cutTravelDistance" step="0.01" min="0.1" max="20.0" 
                                       style="flex: 1; padding: 8px; border: 1px solid rgba(255, 255, 255, 0.3); border-radius: 6px; background: rgba(255, 255, 255, 0.1); color: white; font-size: 0.9rem;" 
                                       placeholder="2.2">
                                <button onclick="updateConfig('cut_travel_distance')" style="padding: 8px 12px; background: rgba(34, 197, 94, 0.8); border: 1px solid rgba(34, 197, 94, 0.6); border-radius: 6px; color: white; font-size: 0.8rem; font-weight: 600; cursor: pointer;">Update</button>
                            </div>
                            <div style="color: rgba(255, 255, 255, 0.6); font-size: 0.75rem; margin-top: 2px;">Range: 0.1 - 20.0</div>
                        </div>
                        
                        <div style="margin-bottom: 16px;">
                            <label style="display: block; color: rgba(255, 255, 255, 0.8); font-size: 0.9rem; font-weight: 500; margin-bottom: 6px;">Feed Travel Distance (inches)</label>
                            <div style="display: flex; gap: 8px; align-items: center;">
                                <input type="number" id="feedTravelDistance" step="0.01" min="0.1" max="10.0" 
                                       style="flex: 1; padding: 8px; border: 1px solid rgba(255, 255, 255, 0.3); border-radius: 6px; background: rgba(255, 255, 255, 0.1); color: white; font-size: 0.9rem;" 
                                       placeholder="3.43">
                                <button onclick="updateConfig('feed_travel_distance')" style="padding: 8px 12px; background: rgba(34, 197, 94, 0.8); border: 1px solid rgba(34, 197, 94, 0.6); border-radius: 6px; color: white; font-size: 0.8rem; font-weight: 600; cursor: pointer;">Update</button>
                            </div>
                            <div style="color: rgba(255, 255, 255, 0.6); font-size: 0.75rem; margin-top: 2px;">Range: 0.1 - 10.0</div>
                        </div>
                        
                        <div style="margin-bottom: 16px;">
                            <label style="display: block; color: rgba(255, 255, 255, 0.8); font-size: 0.9rem; font-weight: 500; margin-bottom: 6px;">Cut Motor Normal Speed</label>
                            <div style="display: flex; gap: 8px; align-items: center;">
                                <input type="number" id="cutMotorNormalSpeed" step="10" min="100" max="5000" 
                                       style="flex: 1; padding: 8px; border: 1px solid rgba(255, 255, 255, 0.3); border-radius: 6px; background: rgba(255, 255, 255, 0.1); color: white; font-size: 0.9rem;" 
                                       placeholder="640">
                                <button onclick="updateConfig('cut_motor_normal_speed')" style="padding: 8px 12px; background: rgba(34, 197, 94, 0.8); border: 1px solid rgba(34, 197, 94, 0.6); border-radius: 6px; color: white; font-size: 0.8rem; font-weight: 600; cursor: pointer;">Update</button>
                            </div>
                            <div style="color: rgba(255, 255, 255, 0.6); font-size: 0.75rem; margin-top: 2px;">Range: 100 - 5000</div>
                        </div>
                    </div>
                </div>
                
                <div id="configStatus" style="padding: 8px 12px; border-radius: 6px; font-size: 0.85rem; font-weight: 500; display: none; margin-top: 16px;"></div>
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
        console.log('Dashboard script starting...');
        let ws;
        let reconnectTimeout;
        let heartbeatInterval;
        let heartbeatTimeout;
        let isConnected = false;
        let reconnectAttempts = 0;
        let lastPongReceived = 0;
        let isReconnecting = false;
        const maxReconnectAttempts = 50; // Increased from 20 to 50 for more persistent reconnection
        const heartbeatIntervalMs = 500; // Send ping every 0.5 seconds (more aggressive)
        const heartbeatTimeoutMs = 1500; // Consider connection dead after 1.5 seconds without pong
        
        
        // Cycle timing variables (removed smooth ticking - now shows time since last cycle)
        
        function updateConnectionStatus(connected, message, isReconnecting = false) {
            const statusEl = document.getElementById('connectionStatus');
            const textEl = document.getElementById('connectionText');
            
            if (connected) {
                statusEl.className = 'connection-status connected';
                textEl.textContent = 'Connected';
            } else {
                statusEl.className = 'connection-status disconnected';
                textEl.textContent = 'Disconnected';
            }
        }
        
        function startHeartbeat() {
            if (heartbeatInterval) {
                clearInterval(heartbeatInterval);
            }
            if (heartbeatTimeout) {
                clearTimeout(heartbeatTimeout);
            }
            
            lastPongReceived = Date.now();
            
            heartbeatInterval = setInterval(() => {
                if (ws && ws.readyState === WebSocket.OPEN) {
                    try {
                        ws.send(JSON.stringify({type: 'ping'}));
                        
                        // Set timeout to detect if pong is not received
                        if (heartbeatTimeout) {
                            clearTimeout(heartbeatTimeout);
                        }
                        heartbeatTimeout = setTimeout(() => {
                            console.log('Heartbeat timeout - no pong received');
                            forceDisconnect();
                        }, heartbeatTimeoutMs);
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
            if (heartbeatTimeout) {
                clearTimeout(heartbeatTimeout);
                heartbeatTimeout = null;
            }
            if (ws) {
                ws.close();
                ws = null;
            }
            updateConnectionStatus(false, '', false);
            attemptReconnect();
        }
        
        function attemptReconnect() {
            if (isReconnecting) {
                return; // Prevent multiple simultaneous reconnection attempts
            }
            
            if (reconnectAttempts >= maxReconnectAttempts) {
                // Reset attempts after a longer delay to allow for network recovery
                setTimeout(() => {
                    reconnectAttempts = 0;
                    attemptReconnect();
                }, 10000); // Wait 10 seconds before resetting attempts
                return;
            }
            
            isReconnecting = true;
            reconnectAttempts++;
            const delay = Math.min(500 + (reconnectAttempts * 200), 5000); // Slower, more stable reconnection attempts
            
            updateConnectionStatus(false, '', false); // Show "Disconnected"
            
            reconnectTimeout = setTimeout(() => {
                if (!isConnected) {
                    connect();
                }
                isReconnecting = false;
            }, delay);
        }
        

        function connect() {
            if (reconnectTimeout) {
                clearTimeout(reconnectTimeout);
                reconnectTimeout = null;
            }
            
            updateConnectionStatus(false, '', false); // Show "Disconnected"
            
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.hostname}/ws`;
            
            try {
                ws = new WebSocket(wsUrl);
                
                const connectionTimeout = setTimeout(() => {
                    if (ws && ws.readyState === WebSocket.CONNECTING) {
                        ws.close();
                        forceDisconnect();
                    }
                }, 2000); // Reduced timeout for faster disconnection detection
                
                ws.onopen = function() {
                    clearTimeout(connectionTimeout);
                    isConnected = true;
                    isReconnecting = false;
                    reconnectAttempts = 0;
                    updateConnectionStatus(true, 'Connected');
                    startHeartbeat();
                    console.log('WebSocket connected successfully');
                    
                    
                    // Request all configuration values
                    requestAllConfig();
                };
                
                ws.onmessage = function(event) {
                    try {
                        const data = JSON.parse(event.data);
                        
                        if (data.type === 'pong') {
                            // Clear heartbeat timeout since we received a pong
                            if (heartbeatTimeout) {
                                clearTimeout(heartbeatTimeout);
                                heartbeatTimeout = null;
                            }
                            lastPongReceived = Date.now();
                            return;
                        }
                        
                        
                        if (data.type === 'system_status') {
                            document.getElementById('currentState').textContent = data.currentState;
                            document.getElementById('uptime').textContent = formatUptime(data.uptime);
                        }
                        
                        if (data.type === 'sensor_status') {
                            updateSensorStatus(data);
                        }
                        
                        
                        if (data.type === 'performance_metrics') {
                            
                            // Update reload time (shows time from no 2x4 state end to feed first cut or cutting state)
                            if (data.reloadTime !== undefined && data.reloadTime > 0) {
                                document.getElementById('reloadTime').textContent = formatTimeSinceLastCycle(data.reloadTime);
                            } else {
                                document.getElementById('reloadTime').textContent = '-';
                            }
                            
                            const systemUptime = data.systemUptime || 0;
                            
                            // Update time-based metrics with ghosting logic (only averages now)
                            updateTimeBasedMetricAvg('avgCycles1Min', data.avgCycles1Min, systemUptime, 60000, true);
                            updateTimeBasedMetricAvg('avgCycles3Min', data.avgCycles3Min, systemUptime, 180000, false);
                            updateTimeBasedMetricAvg('avgCycles5Min', data.avgCycles5Min, systemUptime, 300000, false);
                            updateTimeBasedMetricAvg('avgCycles15Min', data.avgCycles15Min, systemUptime, 900000, false);
                            updateTimeBasedMetricAvg('avgCycles30Min', data.avgCycles30Min, systemUptime, 1800000, false);
                        }
                        
                        if (data.type === 'error_status') {
                            document.getElementById('lastError').textContent = data.lastError;
                            document.getElementById('cutMotorErrorCount').textContent = data.cutMotorErrorCount || 0;
                            document.getElementById('suctionErrorCount').textContent = data.suctionErrorCount || 0;
                        }
                        
                        if (data.type === 'event_log') {
                            updateEventLog(data.events);
                        }
                        
                        
                        if (data.type === 'config_value') {
                            updateConfigField(data.key, data.value);
                        }
                        
                        if (data.type === 'config_updated') {
                            if (data.error) {
                                showConfigStatus('Error: ' + data.error, 'error');
                            } else {
                                showConfigStatus('Configuration updated successfully!', 'success');
                            }
                        }
                        
                        if (data.type === 'all_config') {
                            // Populate all configuration fields
                            updateConfigField('cut_travel_distance', data.cut_travel_distance);
                            updateConfigField('feed_travel_distance', data.feed_travel_distance);
                            updateConfigField('cut_motor_normal_speed', data.cut_motor_normal_speed);
                        }
                    } catch (error) {
                        console.error('Error parsing WebSocket message:', error);
                    }
                };
                
                ws.onclose = function(event) {
                    clearTimeout(connectionTimeout);
                    console.log('WebSocket closed:', event.code, event.reason);
                    if (isConnected) {
                        forceDisconnect();
                    }
                };
                
                ws.onerror = function(error) {
                    clearTimeout(connectionTimeout);
                    console.error('WebSocket error:', error);
                    forceDisconnect();
                };
            } catch (error) {
                console.error('Error creating WebSocket:', error);
                forceDisconnect();
            }
        }
        
        function updateSensorStatus(data) {
            updateStatusItem('sensor_2x4', data._2x4Present);
            updateStatusItem('sensor_suction', data.woodSuctionConfirm);
            updateStatusItem('sensor_firstcut', data.firstCutOrWoodFwdOne);
            updateStatusItem('sensor_cuthome', data.cutMotorHomeSwitch);
            updateStatusItem('sensor_feedhome', data.feedMotorHomeSensor);
            updateStatusItem('sensor_reload', data.reloadSwitch);
        }
        
        
        
        function updateStatusItem(elementId, isActive) {
            const element = document.getElementById(elementId);
            const valueElement = element.querySelector('.status-value');
            
            element.className = `status-item ${isActive ? 'active' : 'inactive'}`;
            valueElement.textContent = isActive ? 'ON' : 'OFF';
        }
        
        
        // Cycle timing functions removed - now shows time since last cycle completion
        
        function updateEventLog(events) {
            const logContainer = document.getElementById('eventLog');
            logContainer.innerHTML = '';
            
            events.forEach(event => {
                const eventItem = document.createElement('div');
                eventItem.className = 'event-item';
                
                // Parse the event string to extract timestamp and message
                const match = event.match(/^\[(\d{2}:\d{2}:\d{2})\] (.+)$/);
                if (match) {
                    const timestamp = match[1];
                    const message = match[2];
                    
                    // Create timestamp element
                    const timestampEl = document.createElement('span');
                    timestampEl.className = 'event-timestamp';
                    timestampEl.textContent = timestamp;
                    
                    // Create icon element
                    const iconEl = document.createElement('span');
                    iconEl.className = 'event-icon';
                    
                    // Create message element
                    const messageEl = document.createElement('span');
                    messageEl.className = 'event-message';
                    messageEl.textContent = message;
                    
                    // Determine event type and styling
                    if (message.toLowerCase().includes('error')) {
                        eventItem.classList.add('error');
                        iconEl.textContent = '⚠️';
                    } else if (message.toLowerCase().includes('state changed') || message.toLowerCase().includes('->')) {
                        eventItem.classList.add('state-change');
                        iconEl.textContent = '🔄';
                    } else if (message.toLowerCase().includes('cycle completed') || message.toLowerCase().includes('performance')) {
                        eventItem.classList.add('performance');
                        iconEl.textContent = '📊';
                    } else if (message.toLowerCase().includes('system') || message.toLowerCase().includes('initialized')) {
                        eventItem.classList.add('system');
                        iconEl.textContent = '⚙️';
                    } else {
                        iconEl.textContent = '📝';
                    }
                    
                    // Append elements
                    eventItem.appendChild(timestampEl);
                    eventItem.appendChild(iconEl);
                    eventItem.appendChild(messageEl);
                } else {
                    // Fallback for events that don't match the expected format
                    eventItem.textContent = event;
                }
                
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
        
        function formatTimeSinceLastCycle(seconds) {
            if (seconds === 0) return '0s';
            
            const totalSeconds = Math.floor(seconds);
            const minutes = Math.floor(totalSeconds / 60);
            const remainingSeconds = totalSeconds % 60;
            
            if (minutes > 0) {
                return `${minutes}:${remainingSeconds.toString().padStart(2, '0')}`;
            } else {
                return `${remainingSeconds}s`;
            }
        }
        
        function updateTimeBasedMetricAvg(avgId, avgValue, systemUptime, requiredTime, isWholeNumber) {
            const avgElement = document.getElementById(avgId);
            const avgMetricItem = avgElement.closest('.metric-item');
            
            // Check if enough time has passed
            const isReady = systemUptime >= requiredTime;
            
            if (isReady) {
                // Remove ghosted class
                avgMetricItem.classList.remove('ghosted');
                
                // Update values
                if (avgValue >= 0) {
                    if (isWholeNumber) {
                        avgElement.textContent = Math.round(avgValue);
                    } else {
                        avgElement.textContent = avgValue.toFixed(1);
                    }
                } else {
                    avgElement.textContent = '0';
                }
            } else {
                // Add ghosted class
                avgMetricItem.classList.add('ghosted');
                
                // Show placeholder values
                avgElement.textContent = '-';
            }
        }
        
        // Configuration functions
        function showConfigStatus(message, type) {
            const statusEl = document.getElementById('configStatus');
            statusEl.textContent = message;
            statusEl.style.display = 'block';
            
            if (type === 'success') {
                statusEl.style.background = 'rgba(34, 197, 94, 0.2)';
                statusEl.style.border = '1px solid rgba(34, 197, 94, 0.4)';
                statusEl.style.color = 'rgba(34, 197, 94, 0.9)';
            } else if (type === 'error') {
                statusEl.style.background = 'rgba(239, 68, 68, 0.2)';
                statusEl.style.border = '1px solid rgba(239, 68, 68, 0.4)';
                statusEl.style.color = 'rgba(239, 68, 68, 0.9)';
            }
            
            // Hide status after 3 seconds
            setTimeout(() => {
                statusEl.style.display = 'none';
            }, 3000);
        }
        
        // Configuration field mapping
        const configFieldMap = {
            'cut_travel_distance': 'cutTravelDistance',
            'feed_travel_distance': 'feedTravelDistance',
            'cut_motor_normal_speed': 'cutMotorNormalSpeed'
        };
        
        function updateConfigField(key, value) {
            const fieldId = configFieldMap[key];
            if (fieldId) {
                const element = document.getElementById(fieldId);
                if (element) {
                    element.value = value;
                }
            }
        }
        
        function updateConfig(key) {
            const fieldId = configFieldMap[key];
            if (!fieldId) {
                showConfigStatus('Unknown configuration key', 'error');
                return;
            }
            
            const input = document.getElementById(fieldId);
            if (!input) {
                showConfigStatus('Configuration field not found', 'error');
                return;
            }
            
            const value = key.includes('position') ? parseInt(input.value) : parseFloat(input.value);
            
            if (isNaN(value)) {
                showConfigStatus('Invalid value. Please enter a valid number', 'error');
                return;
            }
            
            // Validate ranges
            let isValid = true;
            let errorMsg = '';
            
            switch (key) {
                case 'cut_travel_distance':
                    isValid = value >= 0.1 && value <= 20.0;
                    errorMsg = 'Range: 0.1 - 20.0 inches';
                    break;
                case 'feed_travel_distance':
                    isValid = value >= 0.1 && value <= 10.0;
                    errorMsg = 'Range: 0.1 - 10.0 inches';
                    break;
                case 'cut_motor_normal_speed':
                    isValid = value >= 100 && value <= 5000;
                    errorMsg = 'Range: 100 - 5000';
                    break;
            }
            
            if (!isValid) {
                showConfigStatus('Invalid value. ' + errorMsg, 'error');
                return;
            }
            
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    type: 'update_config',
                    key: key,
                    value: value
                }));
            } else {
                showConfigStatus('Not connected to server', 'error');
            }
        }
        
        function requestAllConfig() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    type: 'request_all_config'
                }));
            }
        }

        // Connect on page load
        console.log('Initializing dashboard...');
        console.log('Body background:', window.getComputedStyle(document.body).background);
        connect();
        
        console.log('Dashboard initialized successfully');
        
        // Configuration event listeners - using onclick handlers in HTML instead
        
        // Handle manual reconnection
        document.getElementById('connectionStatus').addEventListener('click', function() {
            if (!isConnected) {
                reconnectAttempts = 0;
                isReconnecting = false;
                if (reconnectTimeout) {
                    clearTimeout(reconnectTimeout);
                    reconnectTimeout = null;
                }
                connect();
            }
        });
        
        // Request initial data and monitor connection health
        setInterval(() => {
            if (isConnected && ws && ws.readyState === WebSocket.OPEN) {
                try {
                    ws.send(JSON.stringify({type: 'request_all_data'}));
                } catch (error) {
                    console.error('Error sending data request:', error);
                    forceDisconnect();
                }
                
                // Additional connection health check - if no pong received recently, force disconnect
                const timeSinceLastPong = Date.now() - lastPongReceived;
                if (timeSinceLastPong > heartbeatTimeoutMs) {
                    console.log('Connection health check failed - no recent pong');
                    forceDisconnect();
                }
            }
        }, 5000);
        
        // Connection monitoring - check WebSocket state every 0.5 seconds (more responsive)
        setInterval(() => {
            if (isConnected && ws && ws.readyState !== WebSocket.OPEN) {
                console.log('WebSocket state changed to:', ws.readyState);
                forceDisconnect();
            }
        }, 500);
    </script>
</body>
</html>
)rawliteral";

#endif // DASHBOARD_HTML_H
