#ifndef COMMAND_QUEUE_H
#define COMMAND_QUEUE_H

#include <Arduino.h>
#include "config.h"

// Auto-mode command queue.
// Manual mode does NOT use this — manual_state is a hard override applied
// directly by the brain loop with no queueing/acks.

enum CmdType {
    CMD_NONE,
    CMD_SET_TARGET,
    CMD_MOVE_RELATIVE,
    CMD_TURN_RELATIVE,   // pure rotation: turn N degrees from current heading
    CMD_BACKTRACK,
    CMD_RESET,
    CMD_CLEAR_MAP,
    CMD_CALIBRATE,
    CMD_SET_HEADING,
};

struct QueuedCmd {
    uint32_t id;
    CmdType  type;
    float    x, y;       // for set_target / move_relative
    float    heading;    // for set_heading and turn_relative (delta degrees)
};

class CommandQueue {
public:
    void begin() {
        _head = 0; _tail = 0; _size = 0;
        _ingestedThrough = 0;
        _currentTargetId = 0;
        _lastCompletedId = 0;
        _lastCompletionStatus = "NONE";
    }

    bool isEmpty() const { return _size == 0; }
    bool isFull()  const { return _size >= CMD_QUEUE_DEPTH; }
    int  size()    const { return _size; }

    // Push if not duplicate; returns true if accepted.
    bool push(const QueuedCmd &cmd) {
        // Drop already-ingested ids (idempotent).
        if (cmd.id != 0 && cmd.id <= _ingestedThrough) return false;
        if (isFull()) return false;
        _ring[_tail] = cmd;
        _tail = (_tail + 1) % CMD_QUEUE_DEPTH;
        _size++;
        if (cmd.id > _ingestedThrough) _ingestedThrough = cmd.id;
        return true;
    }

    // Peek at the head without removing.
    const QueuedCmd* peek() const {
        if (isEmpty()) return nullptr;
        return &_ring[_head];
    }

    // Pop head and record as in-flight target id.
    bool pop(QueuedCmd &out) {
        if (isEmpty()) return false;
        out = _ring[_head];
        _head = (_head + 1) % CMD_QUEUE_DEPTH;
        _size--;
        _currentTargetId = out.id;
        return true;
    }

    void clear() {
        _head = 0; _tail = 0; _size = 0;
    }

    // Mark current in-flight command as completed.
    void completeCurrent(const char* status) {
        if (_currentTargetId != 0) {
            _lastCompletedId = _currentTargetId;
            _lastCompletionStatus = status;
            _currentTargetId = 0;
        }
    }

    uint32_t ingestedThrough()      const { return _ingestedThrough; }
    uint32_t currentTargetId()      const { return _currentTargetId; }
    uint32_t lastCompletedId()      const { return _lastCompletedId; }
    const char* lastCompletionStatus() const { return _lastCompletionStatus; }

    void setCurrentTargetId(uint32_t id) { _currentTargetId = id; }

private:
    QueuedCmd _ring[CMD_QUEUE_DEPTH];
    int _head = 0;
    int _tail = 0;
    int _size = 0;

    uint32_t _ingestedThrough = 0;
    uint32_t _currentTargetId = 0;
    uint32_t _lastCompletedId = 0;
    const char* _lastCompletionStatus = "NONE";
};

#endif
