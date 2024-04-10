#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <sstream>
#include <iomanip>


#include "../WiringPi/wiringPi/wiringPi.h"

#include "../State/state.h"

enum class LogType
{
    STATE,
    MSG
};

struct LogEntry
{
    LogType type;
    std::string message;
};

void log_thread();

void log_state(state_t state, std::chrono::time_point<std::chrono::high_resolution_clock> start, float P0, float T0, float loop_time, float velo_windowx, float velo_counterx, float theta_0);

void log_message(const std::string& message, std::chrono::time_point<std::chrono::high_resolution_clock>  start);

void log_stop();