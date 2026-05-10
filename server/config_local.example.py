"""
Local server configuration.

Copy this file to `config_local.py` and fill in your values.
config_local.py is gitignored — it never gets committed, so your
DDNS host / personal endpoints can't accidentally leak to the
public mirror.

If config_local.py doesn't exist, app.py falls back to environment
variables (PORT, DDNS_HOST, DDNS_PORT) so cloud deployments that
inject env vars (Render, Railway, etc.) still work.
"""

# Server listening port. 25565 is just a default — pick anything free.
PORT = 25565

# DDNS info — used only to print the public URL in the startup banner.
# Leave as empty strings to hide the line.
DDNS_HOST = ""        # e.g. "myrobot.example.com"
DDNS_PORT = ""        # e.g. "12345"
