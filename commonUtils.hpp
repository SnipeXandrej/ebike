#ifndef COMMONUTILS_H
#define COMMONUTILS_H

#include <iostream>
#include "toml.hpp"
#include <mutex>
#include <thread>

class AtomicString {
    mutable std::mutex mutex_;
    std::string value_;

public:
    void set(std::string value);
    std::string get();
};

struct profile_t {
    std::string name;
    float l_current_min_scale;
    float l_current_max_scale;
    float l_min_erpm;
    float l_max_erpm;
    float l_min_duty;
    float l_max_duty;
    float l_watt_min;
    float l_watt_max;
    float l_in_current_min;
    float l_in_current_max;
    float  c_phase_current_max;
    bool   c_force_single_motor_acceleration;
};

class MovingAverage {
private:
    float previousOutput;
    bool done = 0;

    float smoothValue(float newValue, float previousValue, float smoothingFactor);

public:
    float smoothingFactor = 1;
    float output;

    void initInput(float input);
    void setInput(float input);
    float moveAverage(float input);
};

float map_f(float x, float in_min, float in_max, float out_min, float out_max);

float map_f_nochecks(float x, float in_min, float in_max, float out_min, float out_max);

double map_d_nochecks(double x, double in_min, double in_max, double out_min, double out_max);

void updateTableValue(const char* SETTINGS_FILEPATH, const char* table_name, const char* setting_name, double value);
void updateTableValue(toml::table &tbl, const char* table_name, const char* setting_name, double value);
void updateTableValue(const char* SETTINGS_FILEPATH, const char* table_name, const char* setting_name, std::string string);
void updateTableValue(toml::table &tbl, const char* table_name, const char* setting_name, std::string string);
void saveTableToFile(toml::table &tbl, const char* SETTINGS_FILEPATH);

#endif