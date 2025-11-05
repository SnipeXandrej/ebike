#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <sstream>

std::vector<std::string> split(const std::string& input, char delimiter);

void commAddValue(std::string* string, double value, int precision);

#endif