#pragma once

#include <map>
#include <string>
#include <vector>

extern std::map<int, int> elm327_pid_status;

void init_elm327_pid_status();
void handle_eml_reponse(const std::string &raw);
void handle_eml_reponse(const std::vector<uint8_t> &raw);
