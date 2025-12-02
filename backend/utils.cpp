#include "utils.hpp"

void updateTableValue(const char* SETTINGS_FILEPATH, const char* table_name, const char* setting_name, double value) {
    toml::table tbl = toml::parse_file(SETTINGS_FILEPATH);
    toml::table* settings = tbl[table_name].as_table();

    // if the table doesn't exist - create it
    if (!settings) {
        tbl.insert_or_assign(table_name, toml::table{});
        settings = tbl[table_name].as_table();
    }

    settings->insert_or_assign(setting_name, value);
    std::cout << "saved: " << table_name << "." << setting_name << "=" << value << "\n";


    // Write back to file
    std::ofstream file(SETTINGS_FILEPATH);
    file << tbl;
    file.close();
}

float getValueFromPacket(std::vector<std::string> token, int index) {
    if (index < (int)token.size()) {
        return std::stof(token[index]);
    }

    std::print("Index out of bounds: {}\n", token[index-1]);
    return -1;
}

double getValueFromPacket_double(std::vector<std::string> token, int index) {
    if (index < (int)token.size()) {
        return std::stod(token[index]);
    }

    std::print("Index out of bounds: {}\n", token[index-1]);
    return -1;
}

uint64_t getValueFromPacket_uint64(std::vector<std::string> token, int index) {
    if (index < (int)token.size()) {
        std::stringstream stream(token[index]);
        uint64_t result;
        stream >> result;
        return result;
    }

    std::print("Index out of bounds: {}\n", token[index-1]);
    return -1;
}

std::string getValueFromPacket_string(std::vector<std::string> token, int index) {
    if (index < (int)token.size()) {
        return token[index];
    }

    std::print("Index out of bounds: {}\n", token[index-1]);
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

float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
    float temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    if (temp < out_min)
        temp = out_min;

    if (temp > out_max)
        temp = out_max;

    return temp;
}

float map_f_nochecks(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double map_d_nochecks(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void commAddValue(std::string* string, double value, int precision) {
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << value;

    string->append(out.str());
    string->append(";");
}

void commAddValue_string(std::string* string, std::string value) {
    string->append(value);
    string->append(";");
}