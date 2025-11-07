#include "toml.hpp"
#include "imgui.h"
#include <fstream>
#include <iostream>
#include <print>

class MovingAverage {
private:
    float previousInput;
    float Input;
    bool done;

    float smoothValue(float newValue, float previousValue, float smoothingFactor);

public:
    float smoothingFactor;
    float output;

    void initInput(float input);
    float moveAverage(float input);
};

std::vector<std::string> split(const std::string& input, char delimiter);

float map_f(float x, float in_min, float in_max, float out_min, float out_max);

void addVUMeter(float input, float input_min, float input_max, const char *label, int precision, int LED_COUNT);

void addValueToArray(int SIZE, float arr[], float newVal);

std::string removeStringWithEqualSignAtTheEnd(const std::string toRemove, std::string str);

float getValueFromString(const std::string toRemove, std::string str);

int findInArray_int(const char* items[], int item_count, int target);

void updateTableValue(const char* SETTINGS_FILEPATH, const char* table_name, const char* setting_name, double value);

void commAddValue(std::string* string, double value, int precision);

void TextCenteredOnLine(const char* label, float alignment, bool contentRegionFromWindow);

void drawRotatedRect(ImDrawList* draw_list, ImVec2 center, ImVec2 size, float angle_deg, ImU32 color, float thickness);

void powerVerticalDiagonalHorizontal(float input);

float getValueFromPacket(std::vector<std::string> token, int *index);

std::string getValueFromPacket_string(std::vector<std::string> token, int *index);

uint64_t getValueFromPacket_uint64(std::vector<std::string> token, int *index);

void StyleColorsDarkBreeze(ImGuiStyle* dst);