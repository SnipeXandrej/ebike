#include "utils.hpp"

void updateTableValue(const char* SETTINGS_FILEPATH, const char* table_name, const char* setting_name, double value) {
    // // Update the value
    // if (auto settings = tbl[table_name].as_table()) {
    //     (*settings)[setting_name] = value;  // Set to your desired value
    // } else {
    //     std::cerr << "No [settings] table found.\n";
    // }

    toml::table tbl = toml::parse_file(SETTINGS_FILEPATH);

    // Access the [settings] table
    if (toml::table* settings = tbl[table_name].as_table()) {
        // Assign or update the value
        settings->insert_or_assign(setting_name, value); // or any desired value
        std::cout << "saved: " << table_name << "." << setting_name << "=" << value << "\n";
    }

    // Write back to file
    std::ofstream file(SETTINGS_FILEPATH);
    file << tbl;
    file.close();
}

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