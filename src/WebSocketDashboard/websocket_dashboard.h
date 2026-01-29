// WebSocket Dashboard interface
// Exposes functions and helpers used by other modules

#ifndef WEBSOCKET_DASHBOARD_H
#define WEBSOCKET_DASHBOARD_H

// Setup the web server and websocket dashboard
void setupWebSocketDashboard();

// Periodic dashboard status update (called from main loop)
void updateDashboardStatus();

// Returns the current configuration mode for the dashboard:
// 0 = 3 Inch, 1 = Minis
int getCurrentConfigMode();

#endif // WEBSOCKET_DASHBOARD_H


