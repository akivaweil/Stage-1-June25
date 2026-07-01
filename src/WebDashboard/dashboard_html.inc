#pragma once

#include <Arduino.h>

const char dashboardHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Table Saw Dashboard</title>
    <link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='.9em' font-size='90'>⚙️</text></svg>">
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
    <link href="https://fonts.googleapis.com/css2?family=Geist:wght@300;400;500;600;700&family=Geist+Mono:wght@400;500;600;700&display=swap" rel="stylesheet">
    <style>
        :root {
            /* Tuesday — Navy Forced-Dark Palette (HSL) */
            /* Hue 222–225 = the saturated navy from Tuesday's sidebar (hsl(225 64% 14%)).
               Saturation lifted from ~10% to ~28% so the surfaces actually read blue. */
            --bg-main: hsl(222 32% 8%);            /* deep navy page bg (#0d1320-ish) */
            --bg-card: hsl(222 28% 12%);           /* clearly lifted card */
            --bg-elevated: hsl(222 24% 17%);       /* chips / inputs / hovers */
            --bg-input: hsl(222 32% 9%);           /* recessed input */
            --bg-deepest: hsl(222 40% 4%);         /* logs / terminal */

            --border-default: hsl(217 25% 22%);    /* bluish hairline */
            --border-strong: hsl(217 30% 32%);     /* hover state */
            --border-glass: hsl(0 0% 100% / 0.08);

            --text-main: hsl(0 0% 98%);            /* foreground */
            --text-muted: hsl(217 16% 70%);        /* slate-300-ish */
            --text-dim: hsl(217 14% 52%);          /* slate-400 */

            /* Tuesday status palette */
            --accent-primary: hsl(217 91% 60%);    /* blue-500 */
            --accent-primary-soft: hsl(213 94% 78%);/* blue-300 — gradient endpoint */
            --accent-primary-deep: hsl(225 64% 14%);/* sidebar navy */
            --accent-secondary: hsl(258 90% 66%);  /* violet-500 */
            --accent-success: hsl(142 71% 45%);    /* green-500 */
            --accent-warning: hsl(38 92% 50%);     /* amber-500 */
            --accent-danger: hsl(0 72% 51%);       /* red-500 */

            --shadow-card: 0 1px 2px rgba(0, 0, 0, 0.3);
            --shadow-elevated: 0 8px 24px rgba(0, 0, 0, 0.45),
                               0 2px 4px rgba(0, 0, 0, 0.3);
            --shadow-focus: 0 0 0 3px hsl(217 91% 60% / 0.25);
            --shadow-blue-glow: 0 0 24px hsl(217 91% 60% / 0.25);

            /* Tuesday radius scale — tight & purposeful */
            --radius-lg: 0.75rem;   /* 12px — cards */
            --radius-md: 0.5rem;    /* 8px  — chips, dropdowns */
            --radius-sm: 0.375rem;  /* 6px  — buttons, inputs */
        }

        * { box-sizing: border-box; margin: 0; padding: 0; }

        /* Scales every rem-based size across the dashboard. */
        html { font-size: 22px; }

        body {
            font-family: 'Geist', system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
            background: var(--bg-main);
            color: var(--text-main);
            min-height: 100vh;
            line-height: 1.45;
            overflow-x: hidden;
            padding: 0.875rem;
            -webkit-font-smoothing: antialiased;
            -moz-osx-font-smoothing: grayscale;
            font-feature-settings: "ss01", "cv11";
        }

        /* Background ambience — strong blue bloom, like the Tuesday sidebar bleeding into the page */
        body::before {
            content: '';
            position: fixed;
            inset: 0;
            background:
                radial-gradient(ellipse 70% 55% at 0% 0%,    hsl(225 64% 22% / 0.55), transparent 60%),
                radial-gradient(ellipse 60% 50% at 100% 0%,  hsl(217 91% 50% / 0.18), transparent 65%),
                radial-gradient(ellipse 90% 60% at 50% 110%, hsl(225 60% 18% / 0.4),  transparent 70%);
            pointer-events: none;
            z-index: 0;
        }

        /* Layout */
        .dashboard-grid {
            display: grid;
            grid-template-columns: repeat(12, 1fr);
            gap: 0.625rem;
            max-width: 1600px;
            margin: 0 auto;
            position: relative;
            z-index: 1;
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
            padding: 0.5rem 0.875rem;
            position: sticky;
            top: 0.5rem;
            z-index: 50;
            background: linear-gradient(135deg,
                hsl(225 50% 14% / 0.75) 0%,
                hsl(222 35% 10% / 0.75) 100%);
            backdrop-filter: blur(16px) saturate(160%);
            -webkit-backdrop-filter: blur(16px) saturate(160%);
            border: 1px solid hsl(217 35% 30% / 0.55);
            border-radius: var(--radius-lg);
            box-shadow: var(--shadow-card),
                        inset 0 1px 0 hsl(0 0% 100% / 0.06);
        }

        .header-title {
            display: flex;
            align-items: center;
            gap: 0.625rem;
        }

        .accent-bar {
            width: 3px;
            height: 22px;
            border-radius: 999px;
            background: linear-gradient(to bottom, var(--accent-primary), hsl(217 91% 45%));
            box-shadow: 0 0 14px hsl(217 91% 60% / 0.55);
        }

        h1 {
            font-size: 1.125rem;
            font-weight: 700;
            letter-spacing: -0.025em;
            background: linear-gradient(135deg, hsl(0 0% 100%) 0%, hsl(213 94% 78%) 100%);
            -webkit-background-clip: text;
            background-clip: text;
            -webkit-text-fill-color: transparent;
            color: transparent;
        }

        /* Status Badge */
        .status-badge {
            display: inline-flex;
            align-items: center;
            gap: 0.4rem;
            padding: 0.3rem 0.7rem;
            background: hsl(240 5% 10% / 0.6);
            border: 1px solid var(--border-default);
            border-radius: 999px;
            font-size: 0.72rem;
            font-weight: 500;
            color: var(--text-muted);
            cursor: pointer;
            transition: border-color 0.18s ease, background 0.18s ease;
            backdrop-filter: blur(8px);
        }

        .status-badge:hover {
            border-color: var(--border-strong);
            background: hsl(240 5% 12% / 0.8);
        }

        .status-dot {
            width: 6px; height: 6px;
            border-radius: 50%;
            background: var(--text-dim);
            transition: all 0.25s;
        }

        .status-badge.connected {
            color: var(--text-main);
        }
        .status-badge.connected .status-dot {
            background: var(--accent-success);
            box-shadow: 0 0 0 3px hsl(142 71% 45% / 0.18),
                        0 0 8px hsl(142 71% 45% / 0.5);
        }

        .status-badge.disconnected .status-dot {
            background: var(--accent-danger);
            box-shadow: 0 0 0 3px hsl(0 72% 51% / 0.18);
        }

        /* Cards */
        .card {
            background: linear-gradient(180deg,
                hsl(222 28% 13%) 0%,
                hsl(222 30% 11%) 100%);
            border: 1px solid var(--border-default);
            border-radius: var(--radius-lg);
            padding: 0.875rem;
            position: relative;
            transition: border-color 0.2s ease, box-shadow 0.2s ease;
            box-shadow: var(--shadow-card),
                        inset 0 1px 0 hsl(0 0% 100% / 0.04);
            display: flex;
            flex-direction: column;
            overflow: visible;
        }

        .card:hover {
            border-color: hsl(217 50% 38%);
            box-shadow: var(--shadow-card),
                        inset 0 1px 0 hsl(0 0% 100% / 0.05),
                        0 0 0 1px hsl(217 91% 60% / 0.06);
        }

        @keyframes systemStateBootFlash {
            0%, 100% { box-shadow: var(--shadow-card); border-color: var(--border-default); }
            50%      { box-shadow: 0 0 0 1px var(--accent-danger), 0 0 24px hsl(0 72% 51% / 0.4);
                       border-color: var(--accent-danger); }
        }
        .system-state-card-flash { animation: systemStateBootFlash 0.5s ease-out; }

        .card-header {
            display: flex;
            align-items: center;
            gap: 0.5rem;
            margin-bottom: 0.75rem;
            color: var(--text-muted);
            font-size: 0.68rem;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 0.08em;
        }

        .card-header svg {
            width: 13px;
            height: 13px;
            stroke-width: 2;
            color: var(--text-dim);
        }

        /* Typography */
        .value-huge {
            font-size: 2rem;
            font-weight: 700;
            line-height: 1;
            letter-spacing: -0.04em;
            color: var(--text-main);
            font-variant-numeric: tabular-nums;
        }

        .value-large {
            font-size: 1.25rem;
            font-weight: 600;
            letter-spacing: -0.02em;
            color: var(--text-main);
            font-variant-numeric: tabular-nums;
        }

        .label-sm {
            font-size: 0.62rem;
            color: var(--text-dim);
            font-weight: 600;
            margin-top: 0.25rem;
            text-transform: uppercase;
            letter-spacing: 0.06em;
        }

        .dimmed {
            color: var(--text-dim) !important;
            opacity: 0.55;
        }

        /* System State */
        .state-display {
            text-align: center;
            padding: 0.75rem 0 0.5rem;
            border-radius: var(--radius-md);
            position: relative;
        }
        .state-value {
            font-size: 0.875rem;
            font-weight: 600;
            color: var(--accent-primary-soft);
            letter-spacing: 0.04em;
            margin-bottom: 0.125rem;
        }

        /* Sensor Matrix */
        .sensor-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 0.375rem;
        }

        .sensor-item {
            background: var(--bg-elevated);
            padding: 0.45rem 0.625rem;
            border-radius: var(--radius-sm);
            display: flex;
            align-items: center;
            justify-content: space-between;
            border: 1px solid var(--border-default);
            transition: border-color 0.18s ease, background 0.18s ease;
        }

        .sensor-item:hover {
            border-color: var(--border-strong);
        }

        .sensor-item.active {
            background: hsl(142 71% 45% / 0.08);
            border-color: hsl(142 71% 45% / 0.45);
        }

        .sensor-label {
            font-size: 0.75rem;
            font-weight: 500;
            color: var(--text-muted);
        }
        .sensor-item.active .sensor-label {
            color: var(--text-main);
            font-weight: 500;
        }

        .sensor-led {
            width: 9px; height: 4px;
            border-radius: 999px;
            background: hsl(240 4% 22%);
            transition: all 0.25s;
        }

        .sensor-item.active .sensor-led {
            background: var(--accent-success);
            box-shadow: 0 0 8px hsl(142 71% 45% / 0.7);
        }

        /* Amber flash while the rotation clamp is retracting */
        .sensor-item.retracting {
            background: hsl(38 92% 50% / 0.10);
            border-color: hsl(38 92% 50% / 0.5);
        }
        .sensor-item.retracting .sensor-label {
            color: var(--text-main);
        }
        .sensor-item.retracting .sensor-led {
            background: var(--accent-warning);
            box-shadow: 0 0 8px hsl(38 92% 50% / 0.7);
        }

        /* Metrics */
        .metrics-container {
            display: flex;
            gap: 1.25rem;
            align-items: flex-end;
        }

        .metric-block { flex: 1; }

        .mini-charts {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 0.375rem;
            margin-top: 0.5rem;
        }

        .mini-chart-card {
            background: var(--bg-elevated);
            border: 1px solid var(--border-default);
            border-radius: var(--radius-sm);
            padding: 0.45rem 0.375rem;
            text-align: center;
            transition: border-color 0.15s ease;
        }

        .mini-chart-card:hover {
            border-color: var(--border-strong);
        }

        .mini-chart-card.status-red {
            background: hsl(0 72% 51% / 0.08);
            border-color: hsl(0 72% 51% / 0.45);
        }

        .mini-chart-card.status-yellow {
            background: hsl(38 92% 50% / 0.08);
            border-color: hsl(38 92% 50% / 0.45);
        }

        .mini-chart-card.status-green {
            background: hsl(142 71% 45% / 0.08);
            border-color: hsl(142 71% 45% / 0.45);
        }

        /* Errors */
        .error-stat {
            display: flex;
            align-items: baseline;
            gap: 0.4rem;
            padding: 0.45rem 0.625rem;
            background: hsl(0 72% 51% / 0.06);
            border-radius: var(--radius-sm);
            color: var(--accent-danger);
            border: 1px solid hsl(0 72% 51% / 0.25);
            transition: border-color 0.18s ease, background 0.18s ease;
        }

        .error-stat:hover {
            background: hsl(0 72% 51% / 0.1);
            border-color: hsl(0 72% 51% / 0.45);
        }

        /* Inputs / Buttons */
        .input-group { margin-bottom: 0.5rem; }
        .input-group label {
            display: block;
            font-size: 0.72rem;
            color: var(--text-muted);
            margin-bottom: 0.25rem;
            font-weight: 500;
        }

        .input-row { display: flex; gap: 0.375rem; }

        /* Strip the native number-input spinner arrows in config rows */
        .input-group input[type="number"]::-webkit-outer-spin-button,
        .input-group input[type="number"]::-webkit-inner-spin-button {
            -webkit-appearance: none;
            margin: 0;
        }
        .input-group input[type="number"] {
            -moz-appearance: textfield;
            appearance: textfield;
        }

        input[type="number"] {
            background: var(--bg-input);
            border: 1px solid var(--border-default);
            color: var(--text-main);
            padding: 0.35rem 0.625rem;
            border-radius: var(--radius-sm);
            width: 100%;
            font-family: 'Geist Mono', ui-monospace, 'SF Mono', Menlo, monospace;
            font-size: 0.78rem;
            font-variant-numeric: tabular-nums;
            transition: border-color 0.15s ease, box-shadow 0.15s ease;
        }

        input[type="number"]::placeholder { color: var(--text-dim); }

        input:focus {
            outline: none;
            border-color: var(--accent-primary);
            box-shadow: var(--shadow-focus);
        }

        button.btn {
            background: var(--bg-elevated);
            border: 1px solid var(--border-default);
            color: var(--text-main);
            padding: 0.35rem 0.7rem;
            border-radius: var(--radius-sm);
            font-weight: 500;
            font-size: 0.74rem;
            font-family: inherit;
            letter-spacing: 0.02em;
            cursor: pointer;
            transition: border-color 0.15s ease, background 0.15s ease, color 0.15s ease;
        }

        button.btn:hover {
            border-color: var(--border-strong);
            background: hsl(240 5% 13%);
        }

        button.btn:active { transform: scale(0.98); }

        /* Primary action variant — Tuesday's "New Order" blue glass button */
        button.btn-primary {
            background: hsl(217 91% 60% / 0.32);
            color: hsl(0 0% 100%);
            border: 1px solid hsl(217 91% 65% / 0.45);
            box-shadow: 0 2px 6px hsl(217 91% 50% / 0.25),
                        inset 0 1px 0 hsl(0 0% 100% / 0.15);
            backdrop-filter: blur(8px);
            -webkit-backdrop-filter: blur(8px);
            font-weight: 600;
            border-radius: 999px;
            transition: background 0.18s ease, border-color 0.18s ease, transform 0.18s ease, box-shadow 0.18s ease;
        }
        button.btn-primary:hover {
            background: hsl(217 91% 60% / 0.5);
            border-color: hsl(217 91% 65% / 0.65);
            box-shadow: 0 4px 14px hsl(217 91% 50% / 0.4),
                        inset 0 1px 0 hsl(0 0% 100% / 0.2);
            transform: translateY(-1px);
        }
        button.btn-primary:active { transform: translateY(0); }

        /* Logs */
        .log-terminal {
            background: var(--bg-deepest);
            border-radius: var(--radius-md);
            padding: 0.625rem;
            font-family: 'Geist Mono', ui-monospace, 'SF Mono', Menlo, monospace;
            font-size: 0.7rem;
            height: 220px;
            overflow-y: auto;
            border: 1px solid var(--border-default);
            color: var(--text-muted);
            line-height: 1.45;
        }

        .log-terminal::-webkit-scrollbar { width: 6px; }
        .log-terminal::-webkit-scrollbar-track { background: transparent; }
        .log-terminal::-webkit-scrollbar-thumb {
            background: hsl(240 4% 22%);
            border-radius: 999px;
        }
        .log-terminal::-webkit-scrollbar-thumb:hover { background: hsl(240 4% 30%); }

        .log-line {
            display: flex;
            gap: 0.875rem;
            padding: 1px 0;
        }

        .log-time { color: var(--text-dim); min-width: 56px; }
        .log-msg  { color: var(--text-muted); word-break: break-all; }

        .log-line.error   .log-msg { color: var(--accent-danger); }
        .log-line.success .log-msg { color: var(--accent-success); }
        .log-line.warn    .log-msg { color: var(--accent-warning); }

        /* Mode Toggle — Tuesday PageToggle (gray pill + pill-squish) */
        .mode-toggle-container {
            display: flex;
            justify-content: center;
            margin-bottom: 0.75rem;
        }

        /* Container — Tuesday: rounded-full bg-gray-800/60 p-1 ring-1 ring-inset ring-gray-700/60 */
        .mode-toggle {
            position: relative;
            display: inline-grid;
            grid-template-columns: repeat(2, 1fr);
            padding: 4px;
            border-radius: 999px;
            background: hsl(220 13% 18% / 0.6);                  /* gray-800/60 */
            box-shadow: inset 0 0 0 1px hsl(220 9% 36% / 0.6);   /* ring-gray-700/60 */
        }

        /* Track holds the bubble's translateX — separated so squish (scale) can compose */
        .mode-bubble-track {
            position: absolute;
            top: 4px;
            bottom: 4px;
            left: 4px;
            width: calc(50% - 4px);
            transition: transform 216ms cubic-bezier(0.32, 1.2, 0.55, 1);
            z-index: 1;
            pointer-events: none;
        }

        .mode-toggle.mode-1 .mode-bubble-track {
            transform: translateX(100%);
        }

        /* Bubble — blue glass, matches .state-pill and .btn-primary */
        .mode-bubble {
            width: 100%;
            height: 100%;
            border-radius: 999px;
            background: hsl(217 91% 60% / 0.32);                  /* same alpha as btn-primary */
            border: 1px solid hsl(217 91% 65% / 0.45);
            box-shadow: 0 2px 6px hsl(217 91% 50% / 0.28),
                        inset 0 1px 0 hsl(0 0% 100% / 0.15),
                        0 0 14px hsl(217 91% 60% / 0.22);
            transform-origin: center;
        }

        .mode-bubble.squish {
            animation: pill-squish 420ms cubic-bezier(0.32, 1.2, 0.55, 1);
        }

        /* Tuesday's exact pill-squish keyframes */
        @keyframes pill-squish {
            0%   { transform: scaleX(1)    scaleY(1); }
            30%  { transform: scaleX(0.78) scaleY(1.12); }
            60%  { transform: scaleX(1.06) scaleY(0.95); }
            82%  { transform: scaleX(0.98) scaleY(1.02); }
            100% { transform: scaleX(1)    scaleY(1); }
        }

        /* Pill items — h-8 px-4 rounded-full text-sm font-medium */
        .mode-option {
            position: relative;
            z-index: 2;
            height: 32px;                                          /* h-8 */
            padding: 0 16px;                                       /* px-4 */
            border-radius: 999px;
            cursor: pointer;
            display: inline-flex;
            align-items: center;
            justify-content: center;
            font-size: 0.875rem;                                   /* text-sm */
            font-weight: 500;                                      /* font-medium */
            color: hsl(220 9% 65%);                                /* gray-400 */
            transition: color 150ms ease;
            user-select: none;
        }

        .mode-option.active {
            color: hsl(0 0% 100%);                                 /* white */
        }

        .mode-option:hover:not(.active) {
            color: hsl(220 14% 83%);                               /* gray-200 */
        }

        /* Dropdown */
        .dropdown-content {
            display: none;
            position: absolute;
            top: 100%;
            left: 0;
            background: hsl(240 6% 8% / 0.92);
            min-width: 220px;
            box-shadow: var(--shadow-elevated);
            border-radius: var(--radius-md);
            border: 1px solid var(--border-default);
            z-index: 100;
            padding: 0.375rem;
            margin-top: 0.5rem;
            backdrop-filter: blur(14px) saturate(150%);
            -webkit-backdrop-filter: blur(14px) saturate(150%);
            animation: fadeIn 0.18s ease;
        }

        .history-header {
            font-size: 0.66rem;
            color: var(--text-dim);
            padding: 0.5rem 0.625rem 0.4rem;
            border-bottom: 1px solid var(--border-default);
            margin-bottom: 0.25rem;
            font-weight: 600;
            letter-spacing: 0.08em;
            text-transform: uppercase;
        }

        .history-item {
            padding: 0.45rem 0.625rem;
            color: var(--text-main);
            font-family: 'Geist Mono', ui-monospace, monospace;
            font-size: 0.8rem;
            font-variant-numeric: tabular-nums;
            display: flex;
            justify-content: space-between;
            border-radius: var(--radius-sm);
            transition: background 0.15s;
        }

        .history-item:hover {
            background: var(--bg-elevated);
        }

        .history-index {
            color: var(--text-dim);
            font-size: 0.72rem;
        }

        /* Hero Status Card */
        .hero-card {
            background:
                radial-gradient(ellipse 90% 100% at 0% 0%, hsl(217 91% 60% / 0.07), transparent 60%),
                linear-gradient(180deg, hsl(222 28% 13%) 0%, hsl(222 30% 11%) 100%);
        }

        .hero-stats {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 0;
            padding: 0.625rem 0 0.5rem;
        }

        .hero-stat {
            text-align: center;
            padding: 0.25rem 0.75rem;
            position: relative;
        }

        .hero-stat + .hero-stat {
            border-left: 1px solid var(--border-default);
        }

        .hero-stat-clickable {
            cursor: pointer;
            border-radius: var(--radius-sm);
            transition: background 0.18s ease;
        }
        .hero-stat-clickable:hover {
            background: hsl(217 91% 60% / 0.05);
        }

        .hero-divider {
            border-top: 1px solid var(--border-default);
            margin: 0.5rem 0 0.625rem;
        }

        .state-pill {
            display: inline-flex;
            align-items: center;
            padding: 0.28rem 0.7rem;
            background: hsl(217 91% 60% / 0.14);
            border: 1px solid hsl(217 91% 60% / 0.4);
            color: var(--accent-primary-soft);
            border-radius: 999px;
            font-size: 0.68rem;
            font-weight: 700;
            letter-spacing: 0.06em;
            text-transform: uppercase;
            font-family: 'Geist Mono', ui-monospace, monospace;
            box-shadow: inset 0 1px 0 hsl(0 0% 100% / 0.08),
                        0 0 14px hsl(217 91% 60% / 0.18);
        }

        /* Pin the tall Config card to the right column, spanning 2 rows so
           Sensors + Diagnostics nest neatly below the Hero (header is row 1,
           Config takes rows 2–3, Logs lands on row 4). */
        .config-card {
            grid-column: 9 / -1;
            grid-row: 2 / span 2;
        }

        /* Section Heading inside cards (e.g. "Motion") */
        .section-heading {
            font-size: 0.62rem;
            color: var(--text-dim);
            font-weight: 600;
            letter-spacing: 0.08em;
            text-transform: uppercase;
            margin: 0.625rem 0 0.4rem;
            padding-top: 0.5rem;
            border-top: 1px solid var(--border-default);
        }
        .section-heading:first-child { border-top: 0; padding-top: 0; margin-top: 0; }

        /* Misc */
        .hidden { display: none; }
        .fade-in { animation: fadeIn 0.45s ease forwards; }
        @keyframes fadeIn {
            from { opacity: 0; transform: translateY(6px); }
            to   { opacity: 1; transform: translateY(0); }
        }

        /* Responsive */
        @media (max-width: 1024px) {
            .dashboard-grid { grid-template-columns: 1fr 1fr; }
            .col-span-4, .col-span-6, .col-span-8, .col-span-12 { grid-column: span 2; }
            .config-card { grid-row: auto; }
        }

        @media (max-width: 768px) {
            body { padding: 1rem; }
            .dashboard-grid { grid-template-columns: 1fr; gap: 0.75rem; }
            .col-span-4, .col-span-6, .col-span-8, .col-span-12 { grid-column: span 1; }
            .metrics-container { flex-direction: column; gap: 1rem; align-items: stretch; }
            .sensor-grid { grid-template-columns: 1fr; }
            header { top: 0; border-radius: var(--radius-md); }
            h1 { font-size: 1.125rem; }
            .hero-stats { grid-template-columns: 1fr; }

            /* Mobile card order: System Overview → Configuration → everything else */
            #systemStateCard { order: 1; }
            .config-card     { order: 2; }
            .card            { order: 3; }
            .hero-stat + .hero-stat {
                border-left: 0;
                border-top: 1px solid var(--border-default);
                margin-top: 0.5rem;
                padding-top: 0.75rem;
            }
        }
    </style>
</head>
<body>
    <div class="dashboard-grid">
        <header>
            <div class="header-title">
                <div class="accent-bar"></div>
                <h1>Stage 1 Controller</h1>
            </div>
            <div class="status-badge disconnected" id="connectionStatus">
                <div class="status-dot"></div>
                <span id="connectionText">Offline</span>
            </div>
        </header>

        <!-- Hero Status -->
        <div class="card hero-card col-span-8 fade-in" id="systemStateCard" style="animation-delay: 0.1s">
            <div class="card-header" style="justify-content: space-between;">
                <div style="display: flex; align-items: center; gap: 0.5rem;">
                    <svg fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M13 10V3L4 14h7v7l9-11h-7z"/></svg>
                    System Overview
                </div>
                <div style="display: flex; align-items: center; gap: 0.5rem;">
                    <div class="state-pill" id="currentState">INITIALIZING</div>
                    <button class="btn btn-primary" onclick="triggerStartCycle()" style="font-size: 0.66rem; padding: 0.3rem 0.7rem; letter-spacing: 0.05em;">START CYCLE</button>
                </div>
            </div>

            <div class="hero-stats">
                <div class="hero-stat">
                    <div class="value-huge" id="uptime" style="font-family: 'Geist Mono', ui-monospace, monospace; font-feature-settings: 'tnum';">00:00:00</div>
                    <div class="label-sm">UPTIME</div>
                </div>
                <div class="hero-stat hero-stat-clickable" id="reloadTimeContainer" onclick="toggleReloadHistory()">
                    <div class="value-huge" id="reloadTime" style="font-family: 'Geist Mono', ui-monospace, monospace;">-</div>
                    <div class="label-sm">LAST RELOAD <span style="font-size: 0.7em; color: var(--text-dim); margin-left: 0.25rem;">▼</span></div>
                    <div id="reloadHistoryDropdown" class="dropdown-content" onclick="event.stopPropagation()">
                        <div class="history-header">HISTORY (LAST 10)</div>
                        <div id="reloadHistoryList"></div>
                    </div>
                </div>
            </div>

            <div class="hero-divider"></div>

            <div>
                <div class="label-sm" style="margin: 0 0 0.5rem;">CYCLES / MIN (AVG)</div>
                <div class="mini-charts">
                    <div class="mini-chart-card">
                        <div class="value-large dimmed" id="avgCycles1Min" style="font-size: 1.25rem;">0.0</div>
                        <div class="label-sm" style="margin-top: 0.125rem;">1M</div>
                    </div>
                    <div class="mini-chart-card" id="card-avgCycles3Min">
                        <div class="value-large dimmed" id="avgCycles3Min" style="font-size: 1.25rem;">0.0</div>
                        <div class="label-sm" style="margin-top: 0.125rem;">3M</div>
                    </div>
                    <div class="mini-chart-card" id="card-avgCycles5Min">
                        <div class="value-large dimmed" id="avgCycles5Min" style="font-size: 1.25rem;">0.0</div>
                        <div class="label-sm" style="margin-top: 0.125rem;">5M</div>
                    </div>
                    <div class="mini-chart-card" id="card-avgCycles15Min">
                        <div class="value-large dimmed" id="avgCycles15Min" style="font-size: 1.25rem;">0.0</div>
                        <div class="label-sm" style="margin-top: 0.125rem;">15M</div>
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
                <div class="sensor-item" id="clamp_rotation">
                    <span class="sensor-label" id="clamp_rotation_label">Rotation Clamp</span>
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
            <div style="display: flex; flex-direction: column; gap: 0.625rem; height: 100%;">
                <div>
                    <div class="label-sm" style="margin-top: 0;">LATEST ERROR</div>
                    <div id="lastError" style="color: var(--accent-danger); font-weight: 500; margin-top: 0.25rem; font-size: 0.82rem;">None</div>
                </div>
                <div style="border-top: 1px solid var(--border-default); padding-top: 0.5rem;">
                    <div class="label-sm" style="margin-top: 0;">LAST RESET CAUSE <span id="crashCountBadge" style="font-size: 0.66rem; color: var(--text-dim); margin-left: 0.25rem;"></span></div>
                    <div id="resetReason" style="font-weight: 600; margin-top: 0.25rem; font-family: 'Geist Mono', ui-monospace, monospace; font-size: 0.78rem;">—</div>
                    <div id="crashContext" style="font-size: 0.66rem; color: var(--text-dim); margin-top: 0.25rem; font-family: 'Geist Mono', ui-monospace, monospace; line-height: 1.45;"></div>
                </div>
                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 0.45rem; margin-top: auto;">
                    <div class="error-stat">
                        <div style="font-weight: 600; font-size: 1.05rem; font-variant-numeric: tabular-nums;" id="cutMotorErrorCount">0</div>
                        <div class="label-sm" style="color: var(--accent-danger); margin-top: 0;">CUT ERR</div>
                    </div>
                    <div class="error-stat">
                        <div style="font-weight: 600; font-size: 1.05rem; font-variant-numeric: tabular-nums;" id="suctionErrorCount">0</div>
                        <div class="label-sm" style="color: var(--accent-danger); margin-top: 0;">SUC ERR</div>
                    </div>
                </div>
            </div>
        </div>

        <!-- Config -->
        <div class="card col-span-4 config-card fade-in" style="animation-delay: 0.5s">
            <div class="card-header">
                <svg fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z"/><path stroke-linecap="round" stroke-linejoin="round" d="M15 12a3 3 0 11-6 0 3 3 0 016 0z"/></svg>
                Configuration
            </div>
            
            <div class="mode-toggle-container">
                <div class="mode-toggle mode-0" id="modeToggle">
                    <div class="mode-bubble-track">
                        <div class="mode-bubble" id="modeBubble"></div>
                    </div>
                    <div class="mode-option active" id="mode-3inch" onclick="setMode(0)">3 Inch</div>
                    <div class="mode-option" id="mode-minis" onclick="setMode(1)">Minis</div>
                </div>
            </div>

            <!-- Motion -->
            <div class="section-heading">Motion</div>

            <div class="input-group">
                <label>Cut Distance (in)</label>
                <input type="number" id="cutTravelDistance" step="0.1" placeholder="9.2">
            </div>

            <div class="input-group">
                <label>Cut Motor Speed (in/s)</label>
                <input type="number" id="cutMotorNormalSpeed" step="0.1" placeholder="1.28">
            </div>

            <div class="input-group">
                <label>Feed Distance (in) <span style="opacity:0.7;font-weight:normal;">— controls 2nd square size</span></label>
                <input type="number" id="feedTravelDistance" step="0.01" placeholder="3.43">
            </div>

            <div class="input-group">
                <label>Feed Offset from Sensor (in)</label>
                <input type="number" id="feedMotorOffsetFromSensor" step="0.01" placeholder="0.15">
            </div>

            <!-- Rotation -->
            <div class="section-heading">Rotation</div>

            <div class="input-group">
                <label>Servo Home (deg) <span style="opacity:0.7;font-weight:normal;">— active = Home + 100&deg;</span></label>
                <input type="number" id="rotationServoHomePosition" min="0" max="180" step="1" placeholder="14">
            </div>

            <div class="input-group">
                <label>Clamp Activation (in from cut start)</label>
                <input type="number" id="rotationClampActivationDistance" step="0.1" placeholder="6.5">
            </div>

            <div class="input-group">
                <label>Servo Activation (in from cut start)</label>
                <input type="number" id="rotationServoActivationDistance" step="0.1" placeholder="8.2">
            </div>

            <div class="input-group">
                <label>Clamp Retract Delay After Servo Rotation (ms) <span style="opacity:0.7;font-weight:normal;">&mdash; per mode</span></label>
                <input type="number" id="rotationClampExtendMs" step="50" placeholder="2200">
            </div>

            <!-- Transfer Arm -->
            <div class="section-heading">Transfer Arm</div>

            <div class="input-group">
                <label>TA Signal Offset (in before cut end)</label>
                <input type="number" id="taSignalOffsetFromEnd" step="0.01" placeholder="0.2">
            </div>

            <!-- Pullback (YESWOOD-only) -->
            <div class="section-heading">Pullback</div>

            <div class="input-group">
                <label>Feed Pullback Distance (in)</label>
                <input type="number" id="feedPullbackDistance" step="0.05" placeholder="0.3">
            </div>

            <div class="input-group">
                <label>Feed Forward Compensation (in)</label>
                <input type="number" id="feedPullbackFeedCompensation" step="0.05" placeholder="0.3">
            </div>

            <div id="configStatus" style="font-size: 0.72rem; text-align: center; min-height: 1.2em; transition: color 0.25s; margin: 0.25rem 0;"></div>

            <div style="margin-top: 0.75rem; padding-top: 0.625rem; border-top: 1px solid var(--border-default); display: flex; gap: 0.375rem;">
                <button class="btn" style="flex: 1;" onclick="downloadAllConfigs()">↓ Download</button>
                <button class="btn" style="flex: 1;" onclick="document.getElementById('configUploadInput').click()">↑ Upload</button>
                <input type="file" id="configUploadInput" style="display: none;" accept=".json" onchange="uploadConfigData(event)">
            </div>
        </div>

        <!-- Logs -->
        <div class="card col-span-12 fade-in" style="animation-delay: 0.6s">
            <div class="card-header clickable" onclick="toggleLogs(this)" style="cursor: pointer; margin-bottom: 0; justify-content: space-between;">
                <div style="display: flex; align-items: center; gap: 0.625rem;">
                    <svg fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M4 6h16M4 12h16M4 18h7"/></svg>
                    System Logs
                </div>
                <svg id="logToggleIcon" fill="none" stroke="currentColor" viewBox="0 0 24 24" style="width: 14px; height: 14px; transform: rotate(0deg); transition: transform 0.25s; color: var(--text-dim);"><path stroke-linecap="round" stroke-linejoin="round" d="M19 9l-7 7-7-7"/></svg>
            </div>

            <div id="logContent" style="display: none; grid-template-columns: 1fr 1fr; gap: 0.625rem; margin-top: 0.75rem;">
                <div>
                    <div class="label-sm" style="margin: 0 0 0.4rem;">EVENTS</div>
                    <div class="log-terminal" id="eventLog"></div>
                </div>
                <div>
                    <div class="label-sm" style="margin: 0 0 0.4rem;">SERIAL</div>
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
        
        // Reload time logic for smooth updates
        let lastReloadTimeSeconds = 0;
        let lastReloadTimeUpdateTime = 0;
        let reloadTimeUpdateInterval = null;
        let reloadTimeActive = false;

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

        function toggleReloadHistory() {
            const dd = document.getElementById('reloadHistoryDropdown');
            if (dd.style.display === 'block') {
                dd.style.display = 'none';
            } else {
                dd.style.display = 'block';
            }
        }
        
        // Close dropdown when clicking outside
        document.addEventListener('click', function(event) {
            const dd = document.getElementById('reloadHistoryDropdown');
            const trigger = document.getElementById('reloadTimeContainer');
            
            if (dd.style.display === 'block' && !trigger.contains(event.target)) {
                dd.style.display = 'none';
            }
        });

        function flashSystemStateCard() {
            const card = document.getElementById('systemStateCard');
            if (card) {
                card.classList.remove('system-state-card-flash');
                card.offsetHeight;
                card.classList.add('system-state-card-flash');
                setTimeout(() => card.classList.remove('system-state-card-flash'), 500);
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
                flashSystemStateCard();
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
            stopReloadTimeUpdates();
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

        function formatConfigValue(val) {
            if (typeof val === 'number') {
                // Round to max 3 decimal places to avoid floating point artifacts (e.g. 8.900001)
                // parseFloat removes trailing zeros
                return parseFloat(val.toFixed(3));
            }
            return val;
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
            else if (data.type === 'clamp_status') {
                updateRotationClampIndicator(data.rotationClamp);
            }
            else if (data.type === 'performance_metrics') {
                if (data.reloadTime !== undefined) {
                    const newReloadTime = data.reloadTime;
                    const now = Date.now();
                    
                    // Check if reload timer is active (counting up)
                    // If new time is greater than previous (with small tolerance for float comparison), timer is active
                    if (newReloadTime > (lastReloadTimeSeconds + 0.1)) {
                        // Timer is active - start smooth updates
                        if (!reloadTimeActive) {
                            reloadTimeActive = true;
                            lastReloadTimeSeconds = newReloadTime;
                            lastReloadTimeUpdateTime = now;
                            startReloadTimeUpdates();
                        } else {
                            // Update base time if server sent a new value (resync)
                            lastReloadTimeSeconds = newReloadTime;
                            lastReloadTimeUpdateTime = now;
                        }
                    } else if (reloadTimeActive && newReloadTime <= lastReloadTimeSeconds) {
                        // Timer stopped - stop smooth updates and show final value
                        stopReloadTimeUpdates();
                        lastReloadTimeSeconds = newReloadTime;
                        document.getElementById('reloadTime').textContent = formatTime(newReloadTime);
                    } else if (!reloadTimeActive) {
                        // Timer not active, just display the value
                        lastReloadTimeSeconds = newReloadTime;
                        document.getElementById('reloadTime').textContent = formatTime(newReloadTime);
                    }
                }
                
                if (data.reloadHistory) {
                    updateReloadHistoryList(data.reloadHistory);
                }

                updateMetric('avgCycles1Min', data.avgCycles1Min);
                updateMetric('avgCycles3Min', data.avgCycles3Min);
                updateMetric('avgCycles5Min', data.avgCycles5Min);
                updateMetric('avgCycles15Min', data.avgCycles15Min);
                
                // Apply conditional styling to 3M, 5M, and 15M cards
                updateMetricCardStatus('avgCycles3Min', data.avgCycles3Min);
                updateMetricCardStatus('avgCycles5Min', data.avgCycles5Min);
                updateMetricCardStatus('avgCycles15Min', data.avgCycles15Min);
            }
            else if (data.type === 'error_status') {
                document.getElementById('lastError').textContent = data.lastError;
                document.getElementById('cutMotorErrorCount').textContent = data.cutMotorErrorCount || 0;
                document.getElementById('suctionErrorCount').textContent = data.suctionErrorCount || 0;
            }
            else if (data.type === 'crash_info') {
                const reasonEl = document.getElementById('resetReason');
                const ctxEl = document.getElementById('crashContext');
                const badgeEl = document.getElementById('crashCountBadge');
                if (reasonEl) {
                    reasonEl.textContent = data.resetReason || '—';
                    reasonEl.style.color = data.abnormal ? 'var(--accent-danger)' : 'var(--text-main)';
                }
                if (ctxEl) {
                    if (data.abnormal) {
                        const stepStr = (data.lastCuttingStep >= 0 && data.lastState === 'CUTTING')
                            ? ' step ' + data.lastCuttingStep : '';
                        const upMs = data.lastUptimeMs || 0;
                        const upSec = (upMs / 1000).toFixed(1);
                        ctxEl.textContent = 'Died in ' + (data.lastState || '?') + stepStr +
                                            ' @ ' + upSec + 's uptime';
                    } else {
                        ctxEl.textContent = 'Clean boot';
                    }
                }
                if (badgeEl) {
                    const c = data.crashCount || 0;
                    badgeEl.textContent = c > 0 ? '(' + c + ' since power-on)' : '';
                }
            }
            else if (data.type === 'event_log') {
                updateLog('eventLog', data.events);
            }
            else if (data.type === 'serial_log') {
                updateLog('serialLog', data.logs, true);
            }
            else if (data.type === 'config_mode_changed') {
                updateModeUI(data.mode);
                showConfigStatus('Switched to ' + (data.mode === 0 ? '3 Inch' : 'Minis'), 'success');
            }
            else if (data.type === 'all_config') {
                if (data.config_mode !== undefined) {
                    updateModeUI(data.config_mode);
                }
                if(data.cut_travel_distance) document.getElementById('cutTravelDistance').value = formatConfigValue(data.cut_travel_distance);
                if(data.feed_travel_distance) document.getElementById('feedTravelDistance').value = formatConfigValue(data.feed_travel_distance);
                if(data.feed_motor_offset_from_sensor) document.getElementById('feedMotorOffsetFromSensor').value = formatConfigValue(data.feed_motor_offset_from_sensor);
                if(data.cut_motor_normal_speed) document.getElementById('cutMotorNormalSpeed').value = formatConfigValue(data.cut_motor_normal_speed);
                if(data.rotation_clamp_extend_ms) document.getElementById('rotationClampExtendMs').value = formatConfigValue(data.rotation_clamp_extend_ms);
                if(data.rotation_clamp_activation_distance !== undefined) document.getElementById('rotationClampActivationDistance').value = formatConfigValue(data.rotation_clamp_activation_distance);
                if(data.rotation_servo_activation_distance !== undefined) document.getElementById('rotationServoActivationDistance').value = formatConfigValue(data.rotation_servo_activation_distance);
                if(data.rotation_servo_home_position !== undefined) document.getElementById('rotationServoHomePosition').value = formatConfigValue(data.rotation_servo_home_position);
                if(data.ta_signal_offset_from_end !== undefined) document.getElementById('taSignalOffsetFromEnd').value = formatConfigValue(data.ta_signal_offset_from_end);
                if(data.feed_pullback_distance !== undefined) document.getElementById('feedPullbackDistance').value = formatConfigValue(data.feed_pullback_distance);
                if(data.feed_pullback_feed_compensation !== undefined) document.getElementById('feedPullbackFeedCompensation').value = formatConfigValue(data.feed_pullback_feed_compensation);
            }
            else if (data.type === 'config_updated') {
                showConfigStatus(data.error ? data.error : 'Configuration saved successfully', data.error ? 'error' : 'success');
            }
            else if (data.type === 'all_configs_data') {
                const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = 'tablesaw_config.json';
                document.body.appendChild(a);
                a.click();
                document.body.removeChild(a);
                URL.revokeObjectURL(url);
                showConfigStatus('Config downloaded successfully', 'success');
            }
            else if (data.type === 'error') {
                showConfigStatus(data.message, 'error');
            }
        }

        function updateReloadHistoryList(history) {
            const list = document.getElementById('reloadHistoryList');
            if (!list) return;
            list.innerHTML = '';
            
            if (!history || history.length === 0) {
                list.innerHTML = '<div class="history-item" style="justify-content: center; color: var(--text-dim);">No history</div>';
                return;
            }
            
            history.forEach((time, index) => {
                const div = document.createElement('div');
                div.className = 'history-item';
                div.innerHTML = `<span class="history-index">#${index + 1}</span> <span class="history-value">${formatTime(time)}</span>`;
                list.appendChild(div);
            });
        }

        function updateSensor(id, active) {
            const el = document.getElementById(id);
            if (active) el.classList.add('active');
            else el.classList.remove('active');
        }

        // Rotation clamp live indicator: green while extended, amber
        // "Retracting" flash on the extended->retracted transition.
        const ROTATION_CLAMP_RETRACT_FLASH_MS = 1500;
        let rotationClampWasExtended = null;
        let rotationClampFlashTimer = null;
        function updateRotationClampIndicator(extended) {
            const item = document.getElementById('clamp_rotation');
            const label = document.getElementById('clamp_rotation_label');
            if (!item || !label) return;
            if (extended) {
                if (rotationClampFlashTimer) { clearTimeout(rotationClampFlashTimer); rotationClampFlashTimer = null; }
                item.classList.remove('retracting');
                item.classList.add('active');
                label.textContent = 'Rotation Clamp — Extended';
            } else {
                item.classList.remove('active');
                if (rotationClampWasExtended === true) {
                    item.classList.add('retracting');
                    label.textContent = 'Rotation Clamp — Retracting';
                    if (rotationClampFlashTimer) clearTimeout(rotationClampFlashTimer);
                    rotationClampFlashTimer = setTimeout(() => {
                        item.classList.remove('retracting');
                        label.textContent = 'Rotation Clamp';
                        rotationClampFlashTimer = null;
                    }, ROTATION_CLAMP_RETRACT_FLASH_MS);
                } else if (!rotationClampFlashTimer) {
                    label.textContent = 'Rotation Clamp';
                }
            }
            rotationClampWasExtended = extended;
        }
        
        function updateMetric(id, val) {
            const el = document.getElementById(id);
            if (typeof val === 'number' && val >= 0) {
                el.textContent = (val % 1 === 0 ? val : val.toFixed(1));
                el.classList.remove('dimmed');
            } else {
                el.textContent = '0.0';
                el.classList.add('dimmed');
            }
        }
        
        function updateMetricCardStatus(id, val) {
            const cardId = 'card-' + id;
            const card = document.getElementById(cardId);
            if (!card) return;
            
            // Remove all status classes
            card.classList.remove('status-red', 'status-yellow', 'status-green');
            
            // Apply status based on value
            if (typeof val === 'number' && val >= 0) {
                if (val < 4.5) {
                    card.classList.add('status-red');
                } else if (val < 5) {
                    card.classList.add('status-yellow');
                } else {
                    card.classList.add('status-green');
                }
            }
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
            const val = Math.round(s);
            const m = Math.floor(val / 60);
            if (m > 0) {
                const sec = val % 60;
                return `${m}m ${sec}s`;
            }
            return `${val}s`;
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
        
        function startReloadTimeUpdates() {
            if (reloadTimeUpdateInterval) clearInterval(reloadTimeUpdateInterval);
            reloadTimeUpdateInterval = setInterval(() => {
                if (reloadTimeActive && lastReloadTimeSeconds >= 0) {
                    const now = Date.now();
                    const diffSeconds = (now - lastReloadTimeUpdateTime) / 1000;
                    const currentTime = lastReloadTimeSeconds + diffSeconds;
                    document.getElementById('reloadTime').textContent = formatTime(currentTime);
                }
            }, 100); // Update every 100ms for smooth counting
        }
        
        function stopReloadTimeUpdates() {
            if (reloadTimeUpdateInterval) clearInterval(reloadTimeUpdateInterval);
            reloadTimeActive = false;
        }

        const configMap = {
            'cut_travel_distance': 'cutTravelDistance',
            'feed_travel_distance': 'feedTravelDistance',
            'feed_motor_offset_from_sensor': 'feedMotorOffsetFromSensor',
            'cut_motor_normal_speed': 'cutMotorNormalSpeed',
            'rotation_clamp_extend_ms': 'rotationClampExtendMs',
            'rotation_clamp_activation_distance': 'rotationClampActivationDistance',
            'rotation_servo_activation_distance': 'rotationServoActivationDistance',
            'rotation_servo_home_position': 'rotationServoHomePosition',
            'ta_signal_offset_from_end': 'taSignalOffsetFromEnd',
            'feed_pullback_distance': 'feedPullbackDistance',
            'feed_pullback_feed_compensation': 'feedPullbackFeedCompensation'
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

        function setMode(mode) {
            if (!ws || ws.readyState !== 1) return showConfigStatus('Not connected', 'error');
            
            // Optimistic update
            updateModeUI(mode);
            
            ws.send(JSON.stringify({
                type: 'set_config_mode',
                mode: mode
            }));
        }

        function updateModeUI(mode) {
            const btn3Inch = document.getElementById('mode-3inch');
            const btnMinis = document.getElementById('mode-minis');
            const toggle  = document.getElementById('modeToggle');

            if (mode === 0) {
                btn3Inch.classList.add('active');
                btnMinis.classList.remove('active');
            } else {
                btn3Inch.classList.remove('active');
                btnMinis.classList.add('active');
            }

            if (toggle) {
                const wasMode1 = toggle.classList.contains('mode-1');
                const wasMode0 = toggle.classList.contains('mode-0');
                const prevMode = wasMode1 ? 1 : (wasMode0 ? 0 : null);

                toggle.classList.remove('mode-0', 'mode-1');
                toggle.classList.add('mode-' + mode);

                // Fire pill-squish only on an actual change (not on initial sync)
                if (prevMode !== null && prevMode !== mode) {
                    const bubble = toggle.querySelector('.mode-bubble');
                    if (bubble) {
                        bubble.classList.remove('squish');
                        void bubble.offsetWidth;  /* reflow → restart animation */
                        bubble.classList.add('squish');
                        setTimeout(() => bubble.classList.remove('squish'), 440);
                    }
                }
            }
        }

        function triggerStartCycle() {
            if (!ws || ws.readyState !== 1) return showConfigStatus('Not connected', 'error');
            ws.send(JSON.stringify({type: 'trigger_start_cycle'}));
        }

        function downloadAllConfigs() {
            if (!ws || ws.readyState !== 1) return showConfigStatus('Not connected', 'error');
            ws.send(JSON.stringify({type: 'download_all_configs'}));
            showConfigStatus('Requesting config data...', 'success');
        }

        function uploadConfigData(event) {
            const file = event.target.files[0];
            if (!file) return;

            const reader = new FileReader();
            reader.onload = function(e) {
                try {
                    const data = JSON.parse(e.target.result);
                    if (data.type !== 'all_configs_data') {
                        throw new Error('Invalid config file format');
                    }
                    
                    if (!ws || ws.readyState !== 1) return showConfigStatus('Not connected', 'error');
                    
                    data.type = 'upload_all_configs';
                    ws.send(JSON.stringify(data));
                    showConfigStatus('Uploading configuration...', 'success');
                } catch (error) {
                    showConfigStatus('Error reading file: ' + error.message, 'error');
                }
                
                // Clear the input so the same file can be selected again
                event.target.value = '';
            };
            reader.readAsText(file);
        }

        // Save each config field automatically when it loses focus with a changed
        // value (the native 'change' event also fires on Enter, which is what we want).
        Object.entries(configMap).forEach(([key, id]) => {
            const el = document.getElementById(id);
            if (el) el.addEventListener('change', () => updateConfig(key));
        });

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


