#pragma once
#include <string>
#include <mutex>

extern std::string last_manual_data;
extern std::string ai_motor_data;
extern bool autonomous_mode;
extern std::mutex data_mutex;

