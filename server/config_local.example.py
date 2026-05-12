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

# Google AI Studio key for the chat translator.
# Get one free at https://aistudio.google.com/apikey.
# Leave empty to disable the LLM and use rule-based parsing only.
GOOGLE_AI_API_KEY = ""
# Any model the key has access to. Defaults to gemini-2.5-flash
# (fast, generous free tier). Swap to "gemma-3-27b-it" if you prefer Gemma.
GOOGLE_AI_MODEL = "gemini-2.5-flash"
