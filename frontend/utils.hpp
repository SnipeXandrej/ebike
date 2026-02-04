#ifndef UTILS_HPP
#define UTILS_HPP

#include "imgui.h"
#include <iostream>

void addVUMeter(float input, float input_min, float input_max, const char *label, int precision, int LED_COUNT);

void addValueToArray(int SIZE, float arr[], float newVal);

std::string removeStringWithEqualSignAtTheEnd(const std::string toRemove, std::string str);

float getValueFromString(const std::string toRemove, std::string str);

int findInArray_int(const char* items[], int item_count, int target);

void TextCenteredOnLine(const char* label, float alignment, bool contentRegionFromWindow);

void drawRotatedRect(ImDrawList* draw_list, ImVec2 center, ImVec2 size, float angle_deg, ImU32 color, float thickness);

void powerWidget(int numOfBars, float maxWatts, float indicatorEveryWatts, float input);

void StyleColorsDarkBreeze(ImGuiStyle* dst);

#endif