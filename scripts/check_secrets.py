#!/usr/bin/env python3
"""
Pre-push secret scanner.

Scans every TRACKED file (whatever git would push) for things that look like
they shouldn't leave your machine: WiFi creds, API tokens, private IPs in
firmware code, cloudflared URLs, personal absolute paths, email addresses.

Exit codes:
  0  clean
  1  hits found, push should be blocked

Skips: docs (*.md), the example secrets template, this file, and anything
matching SKIP_FILES below.
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

# --- Patterns. Keep names tight; the user will see them in the report. ---

PATTERNS: list[tuple[str, re.Pattern[str], str]] = [
    (
        "wifi_password_literal",
        re.compile(r'#\s*define\s+WIFI_PASSWORD\s+"(?!YourPassword"|change-me")[^"]+"'),
        "Real WIFI_PASSWORD baked into a header. Move it into src/secrets.h.",
    ),
    (
        "wifi_ssid_literal",
        re.compile(r'#\s*define\s+WIFI_SSID\s+"(?!YourSSID"|YourHotspot")[^"]+"'),
        "Real WIFI_SSID baked into a header. Move it into src/secrets.h.",
    ),
    (
        "private_ip_in_code",
        re.compile(r'"\s*(?:10|192\.168|172\.(?:1[6-9]|2\d|3[01]))\.\d{1,3}\.\d{1,3}\s*"'),
        "Hard-coded private IP. Belongs in secrets.h, not source.",
    ),
    (
        "cloudflared_tunnel",
        # Real cloudflared URLs are <word>-<word>-<word>-<word>.trycloudflare.com.
        # Skip placeholders like "your-tunnel" or "xyz" that appear in docs.
        re.compile(
            r'(?!your-|xyz\.|example[.-])'
            r'[a-z0-9]+(?:-[a-z0-9]+){3,}\.trycloudflare\.com',
            re.IGNORECASE,
        ),
        "Cloudflared tunnel URL. Put it in secrets.h / env vars.",
    ),
    (
        "personal_path",
        re.compile(r'C:\\Users\\[A-Za-z0-9._-]+\\|/Users/[A-Za-z0-9._-]+/|/home/[A-Za-z0-9._-]+/'),
        "Absolute personal path leaks your machine layout. Use a relative path.",
    ),
    (
        "email_address",
        re.compile(r'[A-Za-z0-9._%+-]+@(?!example\.com|noreply\.|users\.noreply\.)[A-Za-z0-9.-]+\.[A-Za-z]{2,}'),
        "Looks like a real email address.",
    ),
    (
        "aws_key",
        re.compile(r'AKIA[0-9A-Z]{16}'),
        "AWS access key.",
    ),
    (
        "anthropic_key",
        re.compile(r'sk-ant-[A-Za-z0-9_-]{20,}'),
        "Anthropic API key.",
    ),
    (
        "openai_key",
        re.compile(r'sk-[A-Za-z0-9]{32,}'),
        "OpenAI-style key.",
    ),
    (
        "github_token",
        re.compile(r'gh[pousr]_[A-Za-z0-9]{30,}'),
        "GitHub token.",
    ),
]

SKIP_FILES = {
    Path("scripts/check_secrets.py"),
    Path("src/secrets.example.h"),
    Path(".gitignore"),
}

SKIP_EXTS = {".md"}


def tracked_files() -> list[Path]:
    """Files git considers tracked (what would be pushed)."""
    out = subprocess.run(
        ["git", "ls-files", "-z"],
        capture_output=True, check=True, text=False,
    )
    return [Path(p.decode()) for p in out.stdout.split(b"\x00") if p]


def scan(path: Path) -> list[tuple[int, str, str, str]]:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except (OSError, UnicodeError):
        return []

    hits = []
    for lineno, line in enumerate(text.splitlines(), start=1):
        for name, pat, why in PATTERNS:
            if pat.search(line):
                snippet = line.strip()
                if len(snippet) > 120:
                    snippet = snippet[:117] + "..."
                hits.append((lineno, name, why, snippet))
    return hits


def main() -> int:
    try:
        files = tracked_files()
    except subprocess.CalledProcessError:
        print("check_secrets: not in a git repo", file=sys.stderr)
        return 0  # don't block non-git contexts

    findings: list[tuple[Path, int, str, str, str]] = []
    for f in files:
        if f in SKIP_FILES or f.suffix in SKIP_EXTS:
            continue
        if not f.exists():
            continue
        for lineno, name, why, snippet in scan(f):
            findings.append((f, lineno, name, why, snippet))

    if not findings:
        print("check_secrets: clean")
        return 0

    print("check_secrets: BLOCKED — possible secrets in tracked files\n", file=sys.stderr)
    for f, lineno, name, why, snippet in findings:
        print(f"  {f}:{lineno}  [{name}]", file=sys.stderr)
        print(f"      {why}", file=sys.stderr)
        print(f"      > {snippet}", file=sys.stderr)
        print(file=sys.stderr)
    print(
        "If you're sure this is fine, push with:  git push --no-verify\n"
        "Or remove the offender, commit the fix, and push again.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
