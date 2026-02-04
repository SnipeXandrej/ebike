#include "messagingUtils.hpp"
#include <print>
#include <sstream>
#include <stdio.h>

namespace msg {
    std::string stringStart = "@!@!";
    std::string stringEnd = "!@!@";
    std::string messageEnd = "@@!!\n";

    std::mutex mtx;

    void start(std::string& message, int command) {
        mtx.lock();
        std::string string = std::format("{};", command);
        message.append(string);
    }

    void addValue(std::string& message, double value) {
        char buffer[128];
        int len = snprintf(
            buffer,
            sizeof(buffer),
            "%.0f",
            value
        );

        message.append(buffer, len);
        message.append(";");
    }

    void addValue(std::string& message, double value, int precision) {
        char buffer[128];
        int len = snprintf(
            buffer,
            sizeof(buffer),
            "%.*f",
            precision,
            value
        );

        message.append(buffer, len);
        message.append(";");
    }

    void end(std::string& message) {
        message.append(messageEnd);
        mtx.unlock();
    }

    std::vector<std::string> split(const std::string& input, const std::string& delimiter) {
        std::vector<std::string> result;

        if (input.find(delimiter) == std::string::npos) {
            return result;
        }

        size_t pos = 0;
        while (pos < input.size()) {
            // string handling
            // check if the input starts with the start marker
            if (input.substr(pos, stringStart.size()) == stringStart) {
                size_t endPos = input.find(stringEnd, pos);

                if (endPos == std::string::npos)
                    break;

                // get string from inside the markers
                size_t innerStart = pos + stringStart.size();
                size_t innerLength = endPos - innerStart;
                result.push_back(input.substr(innerStart, innerLength));

                // move past the end marker
                pos = endPos + stringEnd.size();

                // skip delimiter after the end marker
                if (pos + delimiter.size() <= input.size() && input.substr(pos, delimiter.size()) == delimiter) {
                    pos += delimiter.size();
                }
            } else {
                // non-string handling
                size_t delimPos = input.find(delimiter, pos);

                if (delimPos == std::string::npos)
                    break;
                else {
                    result.push_back(input.substr(pos, delimPos - pos));
                    pos = delimPos + delimiter.size();
                }
            }
        }

        return result;
    }

    float getValueFromSplit(std::vector<std::string> token, int &index) {
        if (index < (int)token.size()) {
            std::stringstream ss(token[index]);
            float number;
            ss >> number;

            float ret = number;
            index++;

            return ret;
        }

        std::print("[getValueFromSplit] Index out of bounds: {}\n", index);
        return -1;
    }

    double getValueFromSplit_double(std::vector<std::string> token, int &index) {
        if (index < (int)token.size()) {
            std::stringstream ss(token[index]);
            double result;
            ss >> result;

            index++;
            return result;
        }

        std::print("[getValueFromSplit_double] Index out of bounds: {}\n", index);
        return -1;
    }

    std::string getValueFromSplit_string(std::vector<std::string> token, int &index) {
        if (index < (int)token.size()) {
            std::string ret = token[index];
            index++;

            return ret;
        }

        std::print("[getValueFromSplit_string] Index out of bounds: {}\n", index);
        return "Error";
    }

    uint64_t getValueFromSplit_uint64(std::vector<std::string> token, int &index) {
        if (index < (int)token.size()) {
            std::stringstream stream(token[index]);
            uint64_t result;
            stream >> result;
            return result;
        }

        std::print("[getValueFromSplit_uint64] Index out of bounds: {}\n", index);
        return -1;
    }
}