#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <sstream>
#include <print>
#include <iostream>
#include "toml.hpp"

void updateTableValue(const char* SETTINGS_FILEPATH, const char* table_name, const char* setting_name, double value);

float getValueFromPacket(std::vector<std::string> token, int index);

uint64_t getValueFromPacket_uint64(std::vector<std::string> token, int index);

std::string getValueFromPacket_string(std::vector<std::string> token, int index);

std::vector<std::string> split(const std::string& input, char delimiter);

void commAddValue(std::string* string, double value, int precision);

#endif