#include "utils.hpp"

float getValueFromPacket(std::vector<std::string> token, int index) {
    if (index < (int)token.size()) {
        return std::stof(token[index]);
    }

    std::println("Index out of bounds");
    return -1;
}

uint64_t getValueFromPacket_uint64(std::vector<std::string> token, int index) {
    if (index < (int)token.size()) {
        std::stringstream stream(token[index]);
        uint64_t result;
        stream >> result;
        return result;
    }

    std::println("Index out of bounds");
    return -1;
}

std::string getValueFromPacket_string(std::vector<std::string> token, int index) {
    if (index < (int)token.size()) {
        return token[index];
    }

    std::println("Index out of bounds");
    return "-1";
}

std::vector<std::string> split(const std::string& input, char delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(input);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        result.push_back(token);
    }

    return result;
}

void commAddValue(std::string* string, double value, int precision) {
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << value;

    string->append(out.str());
    string->append(";");
}