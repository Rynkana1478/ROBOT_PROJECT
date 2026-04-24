#ifndef SECRETS_H
#define SECRETS_H

// Copy this file to secrets.h and fill in your real values.
// secrets.h is gitignored; this template is tracked.

#define WIFI_SSID     "YourSSID"
#define WIFI_PASSWORD "YourPassword"

// Where the Flask server lives
//   Local LAN:        "192.168.x.x"   port 25565  HTTPS=false
//   DDNS:             "yourdomain.thddns.net" port 5570  HTTPS=false
//   Cloudflared:      "*.trycloudflare.com"  port 443  HTTPS=true
//   Render/Railway:   "*.onrender.com"  port 443  HTTPS=true
#define SERVER_HOST   "192.168.1.100"
#define SERVER_PORT   25565
#define SERVER_HTTPS  false

// A name for this robot (shown on the dashboard, not used for auth).
#define CHASSIS_ID    "my-robot"

#endif
