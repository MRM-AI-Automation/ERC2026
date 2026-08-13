#include "stack/shared_state.hpp"

std::string last_manual_data = "M0X0Y0P0Q0A0S0J0DE";
std::string ai_motor_data = "L0R0E";
bool autonomous_mode = false;
std::mutex data_mutex;

