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
    <link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='.9em' font-size='90'>⚙️</text></svg>">
    <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;500;600;700&family=Space+Mono:wght@400;700&display=swap" rel="stylesheet">
    <style>
        :root {
            /* Lighter Dark Theme (Slate 800/700) */
            --bg-main: #1e293b;   /* Slate 800 - Lighter than previous #030712 */
            --bg-card: #334155;   /* Slate 700 - Lighter than previous #0f172a */
            --bg-card-hover: #475569; /* Slate 600 */
            
            --accent-primary: #60a5fa;   /* Lighter Blue */
            --accent-secondary: #a78bfa; /* Lighter Purple */
            --accent-success: #34d399;   /* Emerald 400 */
            --accent-warning: #fbbf24;   /* Amber 400 */
            --accent-danger: #f87171;    /* Red 400 */
            
            --text-main: #f8fafc;     /* Slate 50 */
            --text-muted: #cbd5e1;    /* Slate 300 */
            --text-dim: #94a3b8;      /* Slate 400 */
            
            --border-subtle: rgba(255, 255, 255, 0.1);
            --border-active: rgba(255, 255, 255, 0.2);
            
            --shadow-card: 0 4px 6px -1px rgba(0, 0, 0, 0.2), 0 2px 4px -1px rgba(0, 0, 0, 0.1);
            --shadow-glow: 0 0 20px rgba(96, 165, 250, 0.15);
            
            --radius-lg: 24px;
            --radius-md: 16px;
            --radius-sm: 8px;
        }

        * { box-sizing: border-box; margin: 0; padding: 0; }
        
        body {
            font-family: 'Outfit', system-ui, -apple-system, sans-serif;
            background: var(--bg-main);
            color: var(--text-main);
            min-height: 100vh;
            line-height: 1.5;
            overflow-x: hidden;
            padding: 2rem;
        }

        /* Layout */
        .dashboard-grid {
            display: grid;
            grid-template-columns: repeat(12, 1fr);
            gap: 1.5rem;
            max-width: 1600px;
            margin: 0 auto;
        }

        .col-span-4 { grid-column: span 4; }
        .col-span-6 { grid-column: span 6; }
        .col-span-8 { grid-column: span 8; }
        .col-span-12 { grid-column: span 12; }

        /* Header */
        header {
            grid-column: 1 / -1;
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 1rem;
            padding: 0 0.5rem;
        }

        h1 {
            font-size: 1.5rem;
            font-weight: 700;
            letter-spacing: -0.02em;
            color: var(--text-main);
        }

        /* Cards */
        .card {
            background: var(--bg-card);
            border: 1px solid var(--border-subtle);
            border-radius: var(--radius-lg);
            padding: 1.5rem;
            position: relative;
            transition: transform 0.2s, border-color 0.2s, box-shadow 0.2s;
            box-shadow: var(--shadow-card);
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }

        .card:hover {
            transform: translateY(-2px);
            border-color: var(--border-active);
            box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.3);
        }

        .card-header {
            display: flex;
            align-items: center;
            gap: 0.75rem;
            margin-bottom: 1.5rem;
            color: var(--text-muted);
            font-size: 0.9rem;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 0.05em;
        }

        .card-header svg { width: 18px; height: 18px; stroke-width: 2.5; }

        /* Status Badge */
        .status-badge {
            display: flex;
            align-items: center;
            gap: 0.5rem;
            padding: 0.5rem 1rem;
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid var(--border-subtle);
            border-radius: 100px;
            font-size: 0.85rem;
            font-weight: 500;
            cursor: pointer;
            transition: all 0.2s;
        }

        .status-badge:hover { background: rgba(255, 255, 255, 0.1); }
        
        .status-dot {
            width: 8px; height: 8px;
            border-radius: 50%;
            background: var(--text-dim);
            transition: all 0.3s;
        }

        .status-badge.connected .status-dot {
            background: var(--accent-success);
            box-shadow: 0 0 0 3px rgba(52, 211, 153, 0.2);
        }
        
        .status-badge.disconnected .status-dot {
            background: var(--accent-danger);
            box-shadow: 0 0 0 3px rgba(248, 113, 113, 0.2);
        }

        /* Typography Utilities */
        .value-huge {
            font-size: 3rem;
            font-weight: 700;
            line-height: 1;
            letter-spacing: -0.03em;
            color: var(--text-main);
            font-variant-numeric: tabular-nums;
        }

        .value-large {
            font-size: 1.75rem;
            font-weight: 600;
            letter-spacing: -0.02em;
            color: var(--text-main);
        }

        .label-sm {
            font-size: 0.75rem;
            color: var(--text-dim);
            font-weight: 600;
            margin-top: 0.25rem;
            text-transform: uppercase;
        }

        /* Specific Components */
        
        /* System Status */
        .state-display {
            text-align: center;
            padding: 2rem 0;
            background: radial-gradient(circle at center, rgba(96, 165, 250, 0.1) 0%, transparent 70%);
        }
        .state-value {
            font-size: 2rem;
            font-weight: 800;
            color: var(--accent-primary);
            text-shadow: 0 0 20px rgba(96, 165, 250, 0.3);
            margin-bottom: 0.5rem;
        }

        /* Sensor Matrix */
        .sensor-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 1rem;
        }

        .sensor-item {
            background: rgba(0, 0, 0, 0.2);
            padding: 1rem;
            border-radius: var(--radius-md);
            display: flex;
            align-items: center;
            justify-content: space-between;
            border: 1px solid transparent;
            transition: all 0.3s;
        }

        .sensor-item.active {
            background: rgba(52, 211, 153, 0.1);
            border-color: rgba(52, 211, 153, 0.3);
        }

        .sensor-label { font-size: 0.9rem; font-weight: 500; color: var(--text-muted); }
        .sensor-item.active .sensor-label { color: var(--text-main); font-weight: 600; }
        
        .sensor-led {
            width: 12px; height: 6px;
            border-radius: 10px;
            background: var(--bg-card-hover);
            transition: all 0.3s;
        }

        .sensor-item.active .sensor-led {
            background: var(--accent-success);
            box-shadow: 0 0 8px rgba(52, 211, 153, 0.6);
        }

        /* Metrics */
        .metrics-container {
            display: flex;
            gap: 2rem;
            align-items: flex-end;
        }
        
        .metric-block { flex: 1; }
        
        .mini-charts {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 0.5rem;
            margin-top: 1rem;
        }
        
        .mini-chart-card {
            background: rgba(0, 0, 0, 0.2);
            border-radius: var(--radius-sm);
            padding: 0.75rem 0.5rem;
            text-align: center;
        }

        /* Errors */
        .error-stat {
            display: flex;
            align-items: baseline;
            gap: 0.5rem;
            padding: 0.75rem;
            background: rgba(248, 113, 113, 0.1);
            border-radius: var(--radius-md);
            color: var(--accent-danger);
        }

        /* Config Inputs */
        .input-group { margin-bottom: 1.25rem; }
        .input-group label { display: block; font-size: 0.85rem; color: var(--text-muted); margin-bottom: 0.5rem; }
        
        .input-row { display: flex; gap: 0.75rem; }
        
        input[type="number"] {
            background: rgba(0, 0, 0, 0.3);
            border: 1px solid var(--border-subtle);
            color: var(--text-main);
            padding: 0.75rem 1rem;
            border-radius: var(--radius-sm);
            width: 100%;
            font-family: 'Space Mono', monospace;
            font-size: 0.9rem;
            transition: border-color 0.2s;
        }
        
        input:focus { outline: none; border-color: var(--accent-primary); background: rgba(0, 0, 0, 0.5); }

        button.btn {
            background: var(--bg-card-hover);
            border: 1px solid var(--border-subtle);
            color: var(--accent-primary);
            padding: 0 1.25rem;
            border-radius: var(--radius-sm);
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
        }
        
        button.btn:hover {
            background: var(--accent-primary);
            color: white;
            border-color: transparent;
        }

        /* Logs */
        .log-terminal {
            background: #0f172a; /* Darker than card for contrast */
            border-radius: var(--radius-md);
            padding: 1rem;
            font-family: 'Space Mono', monospace;
            font-size: 0.8rem;
            height: 300px;
            overflow-y: auto;
            border: 1px solid var(--border-subtle);
            color: #f1f5f9;
        }

        .log-line {
            display: flex;
            gap: 1rem;
            padding: 2px 0;
            border-bottom: 1px solid rgba(255,255,255,0.05);
        }
        
        .log-time { color: #64748b; min-width: 60px; }
        .log-msg { color: #e2e8f0; word-break: break-all; }
        
        .log-line.error .log-msg { color: var(--accent-danger); }
        .log-line.success .log-msg { color: var(--accent-success); }
        .log-line.warn .log-msg { color: var(--accent-warning); }

        /* Responsive */
        @media (max-width: 1024px) {
            .dashboard-grid { grid-template-columns: 1fr 1fr; }
            .col-span-4, .col-span-6, .col-span-8, .col-span-12 { grid-column: span 2; }
        }
        
        @media (max-width: 768px) {
            body { padding: 1rem; }
            .dashboard-grid { grid-template-columns: 1fr; gap: 1rem; }
            .col-span-4, .col-span-6, .col-span-8, .col-span-12 { grid-column: span 1; }
            .metrics-container { flex-direction: column; gap: 1rem; }
            .sensor-grid { grid-template-columns: 1fr; }
        }
        
        .hidden { display: none; }
        .fade-in { animation: fadeIn 0.5s ease forwards; }
        @keyframes fadeIn { from { opacity: 0; transform: translateY(10px); } to { opacity: 1; transform: translateY(0); } }
    </style>
</head>
<body>
    <div class="dashboard-grid">
        <header>
            <h1>Stage 1 Controller</h1>
            <div class="status-badge disconnected" id="connectionStatus">
                <div class="status-dot"></div>
                <span id="connectionText">Offline</span>
            </div>
        </header>

        <!-- System Status -->
        <div class="card col-span-4 fade-in" style="animation-delay: 0.1s">
            <div class="card-header">
                <svg fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z"/></svg>
                System State
            </div>
            <div class="state-display">
                <div class="state-value" id="currentState">INITIALIZING</div>
                <div class="label-sm">CURRENT PROCESS</div>
            </div>
            <div style="margin-top: auto; display: flex; justify-content: space-between; padding-top: 1rem; border-top: 1px solid var(--border-subtle);">
                <div>
                    <div class="value-large" id="uptime" style="font-size: 1.25rem; font-family: 'Space Mono';">00:00:00</div>
                    <div class="label-sm">UPTIME</div>
                </div>
            </div>
        </div>

        <!-- Performance -->
        <div class="card col-span-8 fade-in" style="animation-delay: 0.2s">
            <div class="card-header">
                <svg fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M13 10V3L4 14h7v7l9-11h-7z"/></svg>
                Performance Metrics
            </div>
            <div class="metrics-container">
                <div>
                    <div class="value-huge" id="reloadTime">-</div>
                    <div class="label-sm">LAST RELOAD TIME</div>
                </div>
                <div class="metric-block">
                    <div class="label-sm" style="margin-bottom: 0.5rem">CYCLES / MIN (AVG)</div>
                    <div class="mini-charts">
                        <div class="mini-chart-card">
                            <div class="value-large" id="avgCycles1Min" style="font-size: 1.5rem">0.0</div>
                            <div class="label-sm">1M</div>
                        </div>
                        <div class="mini-chart-card">
                            <div class="value-large" id="avgCycles3Min" style="font-size: 1.5rem">0.0</div>
                            <div class="label-sm">3M</div>
                        </div>
                        <div class="mini-chart-card">
                            <div class="value-large" id="avgCycles5Min" style="font-size: 1.5rem">0.0</div>
                            <div class="label-sm">5M</div>
                        </div>
                        <div class="mini-chart-card">
                            <div class="value-large" id="avgCycles15Min" style="font-size: 1.5rem">0.0</div>
                            <div class="label-sm">15M</div>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <!-- Sensors -->
        <div class="card col-span-4 fade-in" style="animation-delay: 0.3s">
            <div class="card-header">
                <svg fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M15 12a3 3 0 11-6 0 3 3 0 016 0z"/><path stroke-linecap="round" stroke-linejoin="round" d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z"/></svg>
                Sensor Matrix
            </div>
            <div class="sensor-grid">
                <div class="sensor-item" id="sensor_2x4">
                    <span class="sensor-label">2x4 Present</span>
                    <div class="sensor-led"></div>
                </div>
                <div class="sensor-item" id="sensor_suction">
                    <span class="sensor-label">Suction</span>
                    <div class="sensor-led"></div>
                </div>
                <div class="sensor-item" id="sensor_firstcut">
                    <span class="sensor-label">First Cut</span>
                    <div class="sensor-led"></div>
                </div>
                <div class="sensor-item" id="sensor_cuthome">
                    <span class="sensor-label">Cut Home</span>
                    <div class="sensor-led"></div>
                </div>
                <div class="sensor-item" id="sensor_feedhome">
                    <span class="sensor-label">Feed Home</span>
                    <div class="sensor-led"></div>
                </div>
                <div class="sensor-item" id="sensor_reload">
                    <span class="sensor-label">Reload</span>
                    <div class="sensor-led"></div>
                </div>
            </div>
        </div>

        <!-- Errors -->
        <div class="card col-span-4 fade-in" style="animation-delay: 0.4s">
            <div class="card-header">
                <svg fill="none" stroke="currentColor" viewBox="0 0 24 24" style="color: var(--accent-danger)"><path stroke-linecap="round" stroke-linejoin="round" d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z"/></svg>
                Diagnostics
            </div>
            <div style="display: flex; flex-direction: column; gap: 1rem; height: 100%;">
                <div>
                    <div class="label-sm">LATEST ERROR</div>
                    <div id="lastError" style="color: var(--accent-danger); font-weight: 600; margin-top: 0.25rem;">None</div>
                </div>
                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 1rem; margin-top: auto;">
                    <div class="error-stat">
                        <div style="font-weight: 700; font-size: 1.5rem;" id="cutMotorErrorCount">0</div>
                        <div class="label-sm" style="color: var(--accent-danger)">CUT ERR</div>
                    </div>
                    <div class="error-stat">
                        <div style="font-weight: 700; font-size: 1.5rem;" id="suctionErrorCount">0</div>
                        <div class="label-sm" style="color: var(--accent-danger)">SUC ERR</div>
                    </div>
                </div>
            </div>
        </div>

        <!-- Config -->
        <div class="card col-span-4 fade-in" style="animation-delay: 0.5s">
            <div class="card-header">
                <svg fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z"/><path stroke-linecap="round" stroke-linejoin="round" d="M15 12a3 3 0 11-6 0 3 3 0 016 0z"/></svg>
                Configuration
            </div>
            
            <div class="input-group">
                <label>Cut Distance (in)</label>
                <div class="input-row">
                    <input type="number" id="cutTravelDistance" step="0.1" placeholder="9.2">
                    <button class="btn" onclick="updateConfig('cut_travel_distance')">SAVE</button>
                </div>
            </div>
            
            <div class="input-group">
                <label>Feed Distance (in)</label>
                <div class="input-row">
                    <input type="number" id="feedTravelDistance" step="0.01" placeholder="3.43">
                    <button class="btn" onclick="updateConfig('feed_travel_distance')">SAVE</button>
                </div>
            </div>
            
            <div class="input-group">
                <label>Cut Motor Speed</label>
                <div class="input-row">
                    <input type="number" id="cutMotorNormalSpeed" step="10" placeholder="640">
                    <button class="btn" onclick="updateConfig('cut_motor_normal_speed')">SAVE</button>
                </div>
            </div>

            <div id="configStatus" style="font-size: 0.8rem; text-align: center; min-height: 1.2em; transition: color 0.3s;"></div>
        </div>

        <!-- Logs -->
        <div class="card col-span-12 fade-in" style="animation-delay: 0.6s">
            <div class="card-header clickable" onclick="toggleLogs(this)" style="cursor: pointer; margin-bottom: 0; justify-content: space-between;">
                <div style="display: flex; align-items: center; gap: 0.75rem;">
                    <svg fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M4 6h16M4 12h16M4 18h7"/></svg>
                    System Logs
                </div>
                <svg id="logToggleIcon" fill="none" stroke="currentColor" viewBox="0 0 24 24" style="transform: rotate(0deg); transition: transform 0.3s;"><path stroke-linecap="round" stroke-linejoin="round" d="M19 9l-7 7-7-7"/></svg>
            </div>
            
            <div id="logContent" style="display: none; grid-template-columns: 1fr 1fr; gap: 1rem; margin-top: 1.5rem;">
                <div>
                    <div class="label-sm" style="margin-bottom: 0.5rem;">EVENTS</div>
                    <div class="log-terminal" id="eventLog"></div>
                </div>
                <div>
                    <div class="label-sm" style="margin-bottom: 0.5rem;">SERIAL</div>
                    <div class="log-terminal" id="serialLog"></div>
                </div>
            </div>
        </div>
    </div>

    <script>
        // Logic maintained from original
        let ws;
        let reconnectTimeout;
        let heartbeatInterval;
        let heartbeatTimeout;
        let isConnected = false;
        let reconnectAttempts = 0;
        
        // Uptime logic
        let lastUptimeMs = 0;
        let lastUptimeUpdateTime = 0;
        let uptimeUpdateInterval = null;

        function toggleLogs(header) {
            const content = document.getElementById('logContent');
            const icon = document.getElementById('logToggleIcon');
            
            if (content.style.display === 'none') {
                content.style.display = 'grid';
                icon.style.transform = 'rotate(180deg)';
            } else {
                content.style.display = 'none';
                icon.style.transform = 'rotate(0deg)';
            }
        }

        function updateConnectionStatus(connected) {
            const badge = document.getElementById('connectionStatus');
            const text = document.getElementById('connectionText');
            if (connected) {
                badge.classList.remove('disconnected');
                badge.classList.add('connected');
                text.textContent = 'Online';
            } else {
                badge.classList.remove('connected');
                badge.classList.add('disconnected');
                text.textContent = 'Offline';
            }
        }

        function connect() {
            if (reconnectTimeout) clearTimeout(reconnectTimeout);
            
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.hostname}/ws`;
            
            try {
                ws = new WebSocket(wsUrl);
                
                ws.onopen = () => {
                    isConnected = true;
                    reconnectAttempts = 0;
                    updateConnectionStatus(true);
                    startHeartbeat();
                    startUptimeUpdates();
                    requestAllConfig();
                };
                
                ws.onmessage = (event) => {
                    try {
                        const data = JSON.parse(event.data);
                        handleMessage(data);
                    } catch (e) { console.error(e); }
                };
                
                ws.onclose = () => {
                    isConnected = false;
                    updateConnectionStatus(false);
                    cleanup();
                    attemptReconnect();
                };
                
                ws.onerror = () => { if (ws.readyState !== 1) ws.close(); };
            } catch (e) {
                console.error(e);
                attemptReconnect();
            }
        }
        
        function cleanup() {
            if (heartbeatInterval) clearInterval(heartbeatInterval);
            if (heartbeatTimeout) clearTimeout(heartbeatTimeout);
            stopUptimeUpdates();
        }

        function attemptReconnect() {
            if (reconnectAttempts > 50) {
                setTimeout(() => { reconnectAttempts = 0; attemptReconnect(); }, 10000);
                return;
            }
            reconnectAttempts++;
            reconnectTimeout = setTimeout(connect, Math.min(1000 + (reconnectAttempts * 500), 5000));
        }

        function startHeartbeat() {
            heartbeatInterval = setInterval(() => {
                if (ws && ws.readyState === 1) {
                    ws.send(JSON.stringify({type: 'ping'}));
                    heartbeatTimeout = setTimeout(() => { ws.close(); }, 2000);
                }
            }, 1000);
        }

        function handleMessage(data) {
            if (data.type === 'pong') {
                if (heartbeatTimeout) clearTimeout(heartbeatTimeout);
            }
            else if (data.type === 'system_status') {
                document.getElementById('currentState').textContent = data.currentState;
                
                const now = Date.now();
                const estimated = lastUptimeMs + (now - lastUptimeUpdateTime);
                if (lastUptimeMs === 0 || Math.abs(estimated - data.uptime) > 2000) {
                    lastUptimeMs = data.uptime;
                    lastUptimeUpdateTime = now;
                    document.getElementById('uptime').textContent = formatUptime(data.uptime);
                }
            }
            else if (data.type === 'sensor_status') {
                updateSensor('sensor_2x4', data._2x4Present);
                updateSensor('sensor_suction', data.woodSuctionConfirm);
                updateSensor('sensor_firstcut', data.firstCutOrWoodFwdOne);
                updateSensor('sensor_cuthome', data.cutMotorHomeSwitch);
                updateSensor('sensor_feedhome', data.feedMotorHomeSensor);
                updateSensor('sensor_reload', data.reloadSwitch);
            }
            else if (data.type === 'performance_metrics') {
                if (data.reloadTime) document.getElementById('reloadTime').textContent = formatTime(data.reloadTime);
                updateMetric('avgCycles1Min', data.avgCycles1Min);
                updateMetric('avgCycles3Min', data.avgCycles3Min);
                updateMetric('avgCycles5Min', data.avgCycles5Min);
                updateMetric('avgCycles15Min', data.avgCycles15Min);
            }
            else if (data.type === 'error_status') {
                document.getElementById('lastError').textContent = data.lastError;
                document.getElementById('cutMotorErrorCount').textContent = data.cutMotorErrorCount || 0;
                document.getElementById('suctionErrorCount').textContent = data.suctionErrorCount || 0;
            }
            else if (data.type === 'event_log') {
                updateLog('eventLog', data.events);
            }
            else if (data.type === 'serial_log') {
                updateLog('serialLog', data.logs, true);
            }
            else if (data.type === 'all_config') {
                if(data.cut_travel_distance) document.getElementById('cutTravelDistance').value = data.cut_travel_distance;
                if(data.feed_travel_distance) document.getElementById('feedTravelDistance').value = data.feed_travel_distance;
                if(data.cut_motor_normal_speed) document.getElementById('cutMotorNormalSpeed').value = data.cut_motor_normal_speed;
            }
            else if (data.type === 'config_updated') {
                showConfigStatus(data.error ? data.error : 'Configuration saved successfully', data.error ? 'error' : 'success');
            }
        }

        function updateSensor(id, active) {
            const el = document.getElementById(id);
            if (active) el.classList.add('active');
            else el.classList.remove('active');
        }
        
        function updateMetric(id, val) {
            const el = document.getElementById(id);
            el.textContent = typeof val === 'number' ? (val % 1 === 0 ? val : val.toFixed(1)) : '0.0';
        }

        function updateLog(id, items, isSerial = false) {
            const container = document.getElementById(id);
            container.innerHTML = '';
            items.forEach(item => {
                const div = document.createElement('div');
                div.className = 'log-line';
                
                let time = '', msg = item;
                if (isSerial) {
                    const match = item.match(/^\[(\d+)ms\] (.+)$/);
                    if (match) { time = `+${match[1]}ms`; msg = match[2]; }
                } else {
                    const match = item.match(/^\[(\d{2}:\d{2}:\d{2})\] (.+)$/);
                    if (match) { time = match[1]; msg = match[2]; }
                }
                
                if (msg.toLowerCase().includes('error')) div.classList.add('error');
                else if (msg.toLowerCase().includes('success')) div.classList.add('success');
                else if (msg.toLowerCase().includes('warning')) div.classList.add('warn');

                div.innerHTML = `<span class="log-time">${time}</span><span class="log-msg">${msg}</span>`;
                container.appendChild(div);
            });
            container.scrollTop = container.scrollHeight;
        }

        function formatUptime(ms) {
            const s = Math.floor(ms / 1000);
            const d = Math.floor(s / 86400);
            const h = Math.floor((s % 86400) / 3600);
            const m = Math.floor((s % 3600) / 60);
            const sec = s % 60;
            
            if (d > 0) return `${d}d ${h}h ${m}m`;
            return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(sec).padStart(2,'0')}`;
        }
        
        function formatTime(s) {
            const m = Math.floor(s / 60);
            const sec = Math.floor(s % 60);
            if (m > 0) return `${m}m ${sec}s`;
            return `${sec.toFixed(1)}s`;
        }

        function startUptimeUpdates() {
            if (uptimeUpdateInterval) clearInterval(uptimeUpdateInterval);
            uptimeUpdateInterval = setInterval(() => {
                if (lastUptimeMs > 0) {
                    const now = Date.now();
                    const diff = now - lastUptimeUpdateTime;
                    document.getElementById('uptime').textContent = formatUptime(lastUptimeMs + diff);
                }
            }, 1000);
        }
        
        function stopUptimeUpdates() {
            if (uptimeUpdateInterval) clearInterval(uptimeUpdateInterval);
        }

        const configMap = {
            'cut_travel_distance': 'cutTravelDistance',
            'feed_travel_distance': 'feedTravelDistance',
            'cut_motor_normal_speed': 'cutMotorNormalSpeed'
        };

        function updateConfig(key) {
            const id = configMap[key];
            const val = document.getElementById(id).value;
            if (!ws || ws.readyState !== 1) return showConfigStatus('Not connected', 'error');
            
            ws.send(JSON.stringify({
                type: 'update_config',
                key: key,
                value: parseFloat(val)
            }));
        }
        
        function requestAllConfig() {
            if (ws && ws.readyState === 1) ws.send(JSON.stringify({type: 'request_all_config'}));
        }
        
        function showConfigStatus(msg, type) {
            const el = document.getElementById('configStatus');
            el.textContent = msg;
            el.style.color = type === 'error' ? 'var(--accent-danger)' : 'var(--accent-success)';
            setTimeout(() => el.textContent = '', 3000);
        }

        // Init
        connect();
        document.getElementById('connectionStatus').addEventListener('click', connect);
        setInterval(() => {
            if (isConnected && ws.readyState === 1) ws.send(JSON.stringify({type: 'request_all_data'}));
        }, 2000);
    </script>
</body>
</html>
)rawliteral";

#endif // DASHBOARD_HTML_H
