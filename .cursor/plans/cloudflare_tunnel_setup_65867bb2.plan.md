---
name: Cloudflare Tunnel Setup
overview: Set up Cloudflare Tunnel on your warehouse Windows computer to access the ESP32 dashboard remotely from home, with no changes to ESP32 code.
todos:
  - id: find-ip
    content: Find ESP32's local IP address
    status: pending
  - id: cloudflare-account
    content: Create Cloudflare account if needed
    status: pending
  - id: install-cloudflared
    content: Download and install cloudflared on Windows
    status: pending
  - id: authenticate
    content: Authenticate cloudflared with Cloudflare
    status: pending
    dependencies:
      - cloudflare-account
      - install-cloudflared
  - id: create-tunnel
    content: Create tunnel and note credentials
    status: pending
    dependencies:
      - authenticate
  - id: config-file
    content: Create config.yml with ESP32 IP
    status: pending
    dependencies:
      - find-ip
      - create-tunnel
  - id: test-tunnel
    content: Test tunnel manually
    status: pending
    dependencies:
      - config-file
  - id: install-service
    content: Install as Windows service for auto-start
    status: pending
    dependencies:
      - test-tunnel
  - id: verify-remote
    content: Verify remote access from home/cellular
    status: pending
---

# Remote Dashboard Access via Cloudflare Tunnel

## Overview

This plan sets up Cloudflare Tunnel on your Windows computer to provide secure remote access to your ESP32 dashboard. **Zero ESP32 code changes required** - everything happens on the Windows machine.

## Prerequisites

- Windows computer on Everwood WiFi network (you have this)
- ESP32 running and accessible on local network
- Free Cloudflare account (we'll create if needed)

## Step 1: Find Your ESP32's IP Address

You need to know the ESP32's local IP address. Options:

- Check your router's connected devices list
- Look at the Serial Monitor output when ESP32 boots (it prints the IP)
- Check your phone/computer's recent connections when you accessed the dashboard

The IP will look like `192.168.1.xxx` or similar.

## Step 2: Create Cloudflare Account

1. Go to https://dash.cloudflare.com/sign-up
2. Create free account (no credit card required)
3. Verify email

## Step 3: Install Cloudflare Tunnel Software on Windows

**If Chrome blocks the download, use one of these methods:**

### Method 1: Download via PowerShell (Recommended)

Open PowerShell **as Administrator** and run:

```powershell
# Create folder
New-Item -ItemType Directory -Force -Path C:\cloudflared

# Download directly (bypasses Chrome)
Invoke-WebRequest -Uri "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-windows-amd64.exe" -OutFile "C:\cloudflared\cloudflared.exe"
```

### Method 2: Use Different Browser

- Try Microsoft Edge or Firefox
- They typically don't block GitHub releases as aggressively

### Method 3: Disable Chrome Security Temporarily

1. In Chrome, go to `chrome://settings/security`
2. Turn off "Enhanced protection" temporarily
3. Download the file
4. Re-enable protection

**After downloading:**

1. **If Windows Defender blocks it**: Add an exception:

                                                                                                - Open Windows Security (search "Windows Security" in Start menu)
                                                                                                - Go to "Virus & threat protection"
                                                                                                - Click "Manage settings" under "Virus & threat protection settings"
                                                                                                - Scroll down to "Exclusions" and click "Add or remove exclusions"
                                                                                                - Click "Add an exclusion" → "Folder"
                                                                                                - Add `C:\cloudflared\` (or wherever you put the file)
                                                                                                - Alternatively, add the specific file: "Add an exclusion" → "File" → select `cloudflared.exe`

2. Move the downloaded file to `C:\cloudflared\` (create this folder if needed)
3. Rename to `cloudflared.exe` (if not already)
4. Add `C:\cloudflared\` to Windows PATH (optional but recommended)

## Step 4: Authenticate Cloudflare Tunnel

Open Command Prompt or PowerShell **as Administrator** and run:

```bash
C:\cloudflared\cloudflared.exe tunnel login
```

This opens a browser where you authorize the tunnel.

## Step 5: Create the Tunnel

```bash
C:\cloudflared\cloudflared.exe tunnel create tablesaw-dashboard
```

This creates a tunnel and generates a credentials file. Note the tunnel ID shown.

## Step 6: Create Configuration File

Create file: `C:\cloudflared\config.yml` with:

```yaml
tunnel: tablesaw-dashboard
credentials-file: C:\Users\[YourUsername]\.cloudflared\[tunnel-id].json

ingress:
 - hostname: tablesaw.yourdomain.com
    service: http://192.168.x.x:80
 - service: http_status:404
```

Replace:

- `[YourUsername]` with your Windows username
- `[tunnel-id]` with the ID from step 5
- `192.168.x.x` with your ESP32's IP address
- `tablesaw.yourdomain.com` with your desired subdomain (or use Cloudflare's auto-generated one)

## Step 7: Set Up DNS (Optional - for custom domain)

If you own a domain managed by Cloudflare:

```bash
C:\cloudflared\cloudflared.exe tunnel route dns tablesaw-dashboard tablesaw.yourdomain.com
```

If not, Cloudflare provides a free `trycloudflare.com` subdomain.

## Step 8: Run the Tunnel

Test it first:

```bash
C:\cloudflared\cloudflared.exe tunnel --config C:\cloudflared\config.yml run
```

Visit your URL - you should see your dashboard!

## Step 9: Set Up as Windows Service (Auto-start)

Install as service so it runs automatically:

```bash
C:\cloudflared\cloudflared.exe service install
```

Start the service:

```bash
net start cloudflared
```

## Step 10: Test Remote Access

1. Disconnect from Everwood WiFi (use cellular/home network)
2. Visit your Cloudflare URL
3. Dashboard should load exactly as it does locally

## Troubleshooting

**Chrome blocks the download and won't let you keep it:**

- This is a **false positive** - Chrome aggressively blocks unsigned executables
- **Solution**: Use PowerShell to download directly (see Step 3, Method 1)
- Alternative: Use Edge or Firefox browser instead

**Windows Defender blocks the file after download:**

- This is also a **false positive** - Defender flags unsigned executables
- **Solution**: Add Windows Defender exception (see Step 3, "After downloading" section)
- Go to Windows Security → Virus & threat protection → Manage settings → Exclusions
- Add `C:\cloudflared\` folder as an exclusion

**Can't reach ESP32:**

- Verify ESP32 IP address is correct
- Check Windows firewall isn't blocking cloudflared
- Ensure ESP32 is powered on and connected to Everwood WiFi

**Tunnel won't start:**

- Check config.yml syntax (YAML is space-sensitive)
- Verify credentials file path is correct
- Run Command Prompt as Administrator

**Dashboard loads but doesn't update:**

- WebSocket connections should work through tunnel automatically
- Check browser console for errors

## Security Notes

- Cloudflare Tunnel is encrypted end-to-end
- No open ports on your router required
- No port forwarding needed
- Your ESP32 is protected by Cloudflare's network
- Consider adding Cloudflare Access for password protection (optional)

## Result

After completion, you'll access your dashboard from anywhere via:

`https://tablesaw.yourdomain.com` (or your chosen URL)

The ESP32 requires **zero modifications** and continues operating exactly as it does now.