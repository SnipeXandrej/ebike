#include "commonUtils.hpp"
#include "toml.hpp"
#include <print>

float MovingAverage::smoothValue(float newValue, float previousValue, float smoothingFactor) {
    return previousValue + smoothingFactor * (newValue - previousValue);
}

void MovingAverage::initInput(float input) {
    if (!done) {
        previousOutput = input;
        done = 1;
    }
}

void MovingAverage::setInput(float input) {
    previousOutput = input;
}

float MovingAverage::moveAverage(float input) {
    output = smoothValue(input, previousOutput, smoothingFactor);
    previousOutput = output;

    return output;
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


void updateTableValue(const char* SETTINGS_FILEPATH, const char* table_name, const char* setting_name, double value) {
    toml::table tbl = toml::parse_file(SETTINGS_FILEPATH);
    toml::table* settings = tbl[table_name].as_table();

    // if the table doesn't exist - create it
    if (!settings) {
        tbl.insert_or_assign(table_name, toml::table{});
        settings = tbl[table_name].as_table();
    }

    settings->insert_or_assign(setting_name, value);
    std::cout << "updated: " << table_name << "." << setting_name << "=" << value << "\n";
}

void updateTableValue(toml::table &tbl, const char* table_name, const char* setting_name, double value) {
    toml::table* settings = tbl[table_name].as_table();

    // if the table doesn't exist - create it
    if (!settings) {
        tbl.insert_or_assign(table_name, toml::table{});
        settings = tbl[table_name].as_table();
    }

    settings->insert_or_assign(setting_name, value);
    std::cout << "updated: " << table_name << "." << setting_name << "=" << value << "\n";
}

void updateTableValue(const char* SETTINGS_FILEPATH, const char* table_name, const char* setting_name, std::string string) {
    toml::table tbl = toml::parse_file(SETTINGS_FILEPATH);
    toml::table* settings = tbl[table_name].as_table();

    // if the table doesn't exist - create it
    if (!settings) {
        tbl.insert_or_assign(table_name, toml::table{});
        settings = tbl[table_name].as_table();
    }

    settings->insert_or_assign(setting_name, string);
    std::cout << "updated: " << table_name << "." << setting_name << "=" << string << "\n";
}

void updateTableValue(toml::table &tbl, const char* table_name, const char* setting_name, std::string string) {
        toml::table* settings = tbl[table_name].as_table();

    // if the table doesn't exist - create it
    if (!settings) {
        tbl.insert_or_assign(table_name, toml::table{});
        settings = tbl[table_name].as_table();
    }

    settings->insert_or_assign(setting_name, string);
    std::cout << "updated: " << table_name << "." << setting_name << "=" << string << "\n";
}

void saveTableToFile(toml::table &tbl, const char* SETTINGS_FILEPATH) {
    std::ofstream file(SETTINGS_FILEPATH);
    file << tbl;
    file.close();
    std::print("table was saved to {}\n", SETTINGS_FILEPATH);
}