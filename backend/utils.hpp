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

double getValueFromPacket_double(std::vector<std::string> token, int index);

uint64_t getValueFromPacket_uint64(std::vector<std::string> token, int index);

std::string getValueFromPacket_string(std::vector<std::string> token, int index);

std::vector<std::string> split(const std::string& input, char delimiter);

void commAddValue(std::string* string, double value, int precision);

void commAddValue_string(std::string* string, std::string value);

float map_f(float x, float in_min, float in_max, float out_min, float out_max);

float map_f_nochecks(float x, float in_min, float in_max, float out_min, float out_max);

double map_d_nochecks(double x, double in_min, double in_max, double out_min, double out_max);

#endif