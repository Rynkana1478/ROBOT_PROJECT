#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include "config.h"

// ============================================
// WiFi-based debug logger
// Header-only: just the log buffer.
// flush() and hardware tests live in robot_main.cpp
// to avoid include-order issues.
// ============================================

class Debug {
public:
    static const int MAX_LOGS = 20;
    static const int MAX_MSG_LEN = 80;
    static char logs[MAX_LOGS][MAX_MSG_LEN];
    static int logHead;
    static int logCount;
    static bool enabled;
    static portMUX_TYPE mux;

    static void begin() {
        logHead = 0;
        logCount = 0;
        enabled = true;
    }

    static void log(const char* msg) {
        if (!enabled) return;
        portENTER_CRITICAL(&mux);
        strncpy(logs[logHead], msg, MAX_MSG_LEN - 1);
        logs[logHead][MAX_MSG_LEN - 1] = '\0';
        logHead = (logHead + 1) % MAX_LOGS;
        if (logCount < MAX_LOGS) logCount++;
        portEXIT_CRITICAL(&mux);
    }

    static void logf(const char* fmt, ...) {
        if (!enabled) return;
        char buf[MAX_MSG_LEN];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, MAX_MSG_LEN, fmt, args);
        va_end(args);
        log(buf);
    }
};

// Static member definitions
char Debug::logs[Debug::MAX_LOGS][Debug::MAX_MSG_LEN];
int Debug::logHead = 0;
int Debug::logCount = 0;
bool Debug::enabled = true;
portMUX_TYPE Debug::mux = portMUX_INITIALIZER_UNLOCKED;

#endif // DEBUG_H
