#include "other.hpp"


float MovingAverage::smoothValue(float newValue, float previousValue, float smoothingFactor) {
    return previousValue + smoothingFactor * (newValue - previousValue);
}

void MovingAverage::initInput(float input) {
    if (!done) {
        previousInput = input;
        done = 1;
    }
}

float MovingAverage::moveAverage(float input) {
    Input = smoothValue(input, previousInput, smoothingFactor);
    previousInput = Input;

    output = Input;

    return Input;
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

void addVUMeter(float input, float input_min, float input_max, const char *label, int precision, int LED_COUNT) {
    // int LED_COUNT = 18;
    ImGui::PushID(label);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
    ImGui::BeginGroup();
    for (int i = 1; i <= LED_COUNT; i++) {
        ImGui::PushID(i);
        float mapLevel = map_f(input, input_min, input_max, 0.0, LED_COUNT);
        ImVec4 currentColor;

        bool howDoINameThisVariableQuestionMark = (LED_COUNT - i < mapLevel) ? true : false;

        if (i <= 2) { // RED
            if (howDoINameThisVariableQuestionMark) {
                currentColor = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
            } else {
                currentColor = ImVec4(0.15f, 0.0f, 0.0f, 1.0f);
            }
        } else if (i <= 4) { // YELLOW
            if (howDoINameThisVariableQuestionMark) {
                currentColor = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
            } else {
                currentColor = ImVec4(0.15f, 0.15f, 0.0f, 1.0f);
            }
        } else { // GREEN
            if (howDoINameThisVariableQuestionMark) {
                currentColor = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
            } else {
                currentColor = ImVec4(0.0f, 0.15f, 0.0f, 1.0f);
            }
        }

        ImGui::PushStyleColor(ImGuiCol_Button, currentColor);

        ImGui::Button("##", ImVec2(50, 15));
        ImGui::PopStyleColor();
        ImGui::PopID();
    }
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.3f, 0.3f, 1.0f));
    ImGui::Button(label, ImVec2(50, 30));
    if (precision == 0) {
        ImGui::Text("%0.0f", input);
    } else {
        ImGui::Text("%0.1f", input);
    }

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
    ImGui::EndGroup();
    ImGui::PopID();
}

void addValueToArray(int SIZE, float arr[], float newVal) {
    // Shift all values to the left
    for (int i = 0; i < SIZE - 1; ++i) {
        arr[i] = arr[i + 1];
    }
    // Add new value to the end
    arr[SIZE - 1] = newVal;
}

std::string removeStringWithEqualSignAtTheEnd(const std::string toRemove, std::string str)
{
    size_t pos = str.find(toRemove);
    str.erase(pos, toRemove.length() + 1);

    // cout << str << "\n";
    return str;
}

float getValueFromString(const std::string toRemove, std::string str) {
    float value = 0;

    try {
        value = stof(removeStringWithEqualSignAtTheEnd(toRemove, str));
    }
    catch (std::invalid_argument const& ex) {
        std::cout << "[f:getValueFromString] this did an oopsie: " << ex.what() << '\n';
    }

    return value;
}

int findInArray_int(const char* items[], int item_count, int target) {
    char targetStr[256];

    snprintf(targetStr, sizeof(targetStr), "%d", target);
    // std::cout << targetStr << " targetStr\n";

    for (int i = 0; i < item_count; ++i) {
        if (strcmp(items[i], targetStr) == 0) {
            return i;
        }
    }
    return -1;
}

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

void commAddValue(std::string* string, double value, int precision) {
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << value;

    string->append(out.str());
    string->append(";");
}

void TextCenteredOnLine(const char* label, float alignment, bool contentRegionFromWindow) {
    ImGuiStyle& style = ImGui::GetStyle();

    float size = ImGui::CalcTextSize(label).x + style.FramePadding.x * 2.0f;
    float avail = contentRegionFromWindow ? ImGui::GetWindowContentRegionMax().x : ImGui::GetContentRegionAvail().x;

    float off = (avail - size) * alignment;
    if (off > 0.0f)
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + off);

    ImGui::Text(label);
}

void drawRotatedRect(ImDrawList* draw_list, ImVec2 center, ImVec2 size, float angle_deg, ImU32 color, float thickness) {
    float angle_rad = angle_deg * M_PI / 180.0f;
    float cos_a = cosf(angle_rad);
    float sin_a = sinf(angle_rad);

    // Half extents
    float hx = size.x * 0.5f;
    float hy = size.y * 0.5f;

    // Corners relative to center
    ImVec2 corners[4] = {
        ImVec2(-hx, -hy),
        ImVec2(hx, -hy),
        ImVec2(hx, hy),
        ImVec2(-hx, hy)
    };

    // Rotate and translate
    for (int i = 0; i < 4; ++i) {
        float x = corners[i].x;
        float y = corners[i].y;
        corners[i].x = center.x + x * cos_a - y * sin_a;
        corners[i].y = center.y + x * sin_a + y * cos_a;
    }

    // Draw the rotated rectangle
    // draw_list->AddPolyline(corners, 4, color, true, thickness);
    // draw_list->AddQuadFilled(corners, 4, color, true, thickness);
    draw_list->AddConvexPolyFilled(corners, 4, color);
}

void powerVerticalDiagonalHorizontal(float input) {
    ImVec2 pos = ImGui::GetCursorScreenPos();  // Reference point
    // ImVec2 pos = ImVec2(ImGui::GetWindowSize().x/2.0, ImGui::GetWindowSize().y/2.0);
    ImU32 color = IM_COL32(0, 255, 0, 255); // Green
    ImVec2 size = ImVec2(100, 10); // Width x Height

    ImU32 greenDark     = IM_COL32(0, 45, 0, 255); // Green
    ImU32 greenBright   = IM_COL32(0, 255, 0, 255); // Green

    float MAX_WATTAGE = 7500.0f;
    float mapped = map_f(input, 0.0, MAX_WATTAGE, 1.0, 75.0);

    int BAR_COUNT_DIAGONAL = 25;
    int BAR_COUNT_HORIZONTAL = BAR_COUNT_DIAGONAL + 50;

    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    int kw_point = 0;

    for (float i = 1; i <= BAR_COUNT_DIAGONAL; i += 1) {
        pos.y -= 13.0f;
        pos.x += 5.0f;
        ImVec2 center = ImVec2(pos.x + 100, pos.y + 100);

        if (mapped < i) {
            color = greenDark;
        } else {
            if ((mapped - i) < 1.0) {
                int color_brightness = (int)map_f((mapped - i), 0.0, 1.0, 45, 255);
                color = IM_COL32(0, color_brightness, 0 , 255);
            } else {
                color = greenBright;
            }
        }

        drawRotatedRect(draw_list, center, size, 65.0f, color, 2.0f);
        if (((int)i+1) % 10 == 1) {
            ImU32 colorPoints = IM_COL32(255, 0, 0 , 100);
            drawRotatedRect(draw_list, center, size, 65.0f, colorPoints, 2.0f);

            ImGui::BeginGroup();
                ImGui::SetCursorPos(ImVec2(center.x - 35.0, center.y - 70.0));
                kw_point++;
                ImGui::Text("%d", kw_point);
            ImGui::EndGroup();
        }
    }

    size = ImVec2(80, 10); // Width x Height
    for (int i = BAR_COUNT_DIAGONAL+1; i < BAR_COUNT_HORIZONTAL; i++) {
        // pos.y -= 8.0f;
        pos.x += 11.0f;

        ImVec2 center = ImVec2(pos.x + 95, pos.y + 90.0);

        if (mapped < i) {
            color = greenDark;
        } else {
            if ((mapped - i) < 1.0) {
                int color_brightness = (int)map_f((mapped - i), 0.0, 1.0, 45, 255);
                color = IM_COL32(0, color_brightness, 0 , 255);
            } else {
                color = greenBright;
            }
        }

        drawRotatedRect(draw_list, center, size, 65.0f, color, 2.0f);
        if (((int)i+1) % 10 == 1) {
            ImU32 colorPoints = IM_COL32(255, 0, 0 , 100);
            drawRotatedRect(draw_list, center, size, 65.0f, colorPoints, 2.0f);

            ImGui::BeginGroup();
                ImGui::SetCursorPos(ImVec2(center.x - 30.0, center.y - 65.0));
                kw_point++;
                ImGui::Text("%d", kw_point);
            ImGui::EndGroup();
        }
    }
}

float getValueFromPacket(std::vector<std::string> token, int *index) {
    if (*index < (int)token.size()) {
        float ret = std::stof(token[*index]);
        *index = *index+1;

        return ret;
    }

    std::println("Index out of bounds");
    return -1;
}

std::string getValueFromPacket_string(std::vector<std::string> token, int *index) {
    if (*index < (int)token.size()) {
        std::string ret = token[*index];
        *index = *index+1;

        return ret;
    }

    std::println("Index out of bounds");
    return "Error";
}

uint64_t getValueFromPacket_uint64(std::vector<std::string> token, int *index) {
    if (*index < (int)token.size()) {
        std::stringstream stream(token[*index]);
        uint64_t result;
        stream >> result;
        return result;
    }

    std::println("Index out of bounds");
    return -1;
}

void StyleColorsDarkBreeze(ImGuiStyle* dst) {
    ImGuiStyle* style = dst ? dst : &ImGui::GetStyle();

    ImVec4* colors = style->Colors;

    colors[ImGuiCol_Text]                   = ImVec4(0.90f, 0.90f, 0.92f, 1.00f);
    colors[ImGuiCol_TextDisabled]           = ImVec4(0.60f, 0.60f, 0.65f, 1.00f);
    colors[ImGuiCol_WindowBg]               = ImVec4(0.0f, 0.0f, 0.0f, 1.00f); //ImVec4(0.13f, 0.15f, 0.18f, 1.00f);
    colors[ImGuiCol_ChildBg]                = ImVec4(0.0f, 0.0f, 0.0f, 1.00f); //ImVec4(0.12f, 0.14f, 0.17f, 1.00f);
    colors[ImGuiCol_PopupBg]                = ImVec4(0.15f, 0.17f, 0.20f, 1.00f);
    colors[ImGuiCol_Border]                 = ImVec4(0.25f, 0.28f, 0.32f, 0.60f);
    colors[ImGuiCol_BorderShadow]           = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_FrameBg]                = ImVec4(0.20f, 0.23f, 0.26f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]         = ImVec4(0.25f, 0.28f, 0.32f, 1.00f);
    colors[ImGuiCol_FrameBgActive]          = ImVec4(0.28f, 0.31f, 0.36f, 1.00f);
    colors[ImGuiCol_TitleBg]                = ImVec4(0.10f, 0.11f, 0.13f, 1.00f);
    colors[ImGuiCol_TitleBgActive]          = ImVec4(0.15f, 0.16f, 0.19f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed]       = ImVec4(0.08f, 0.09f, 0.10f, 1.00f);
    colors[ImGuiCol_MenuBarBg]              = ImVec4(0.13f, 0.15f, 0.18f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]            = ImVec4(0.10f, 0.10f, 0.12f, 1.00f);
    colors[ImGuiCol_ScrollbarGrab]          = ImVec4(0.25f, 0.28f, 0.32f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabHovered]   = ImVec4(0.28f, 0.31f, 0.36f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabActive]    = ImVec4(0.31f, 0.34f, 0.40f, 1.00f);
    colors[ImGuiCol_CheckMark]              = ImVec4(0.36f, 0.74f, 0.97f, 1.00f);
    colors[ImGuiCol_SliderGrab]             = ImVec4(0.36f, 0.74f, 0.97f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]       = ImVec4(0.46f, 0.84f, 1.00f, 1.00f);
    colors[ImGuiCol_Button]                 = ImVec4(0.20f, 0.23f, 0.26f, 1.00f);
    colors[ImGuiCol_ButtonHovered]          = ImVec4(0.25f, 0.28f, 0.32f, 1.00f);
    colors[ImGuiCol_ButtonActive]           = ImVec4(0.28f, 0.31f, 0.36f, 1.00f);
    colors[ImGuiCol_Header]                 = ImVec4(0.20f, 0.23f, 0.26f, 1.00f);
    colors[ImGuiCol_HeaderHovered]          = ImVec4(0.25f, 0.28f, 0.32f, 1.00f);
    colors[ImGuiCol_HeaderActive]           = ImVec4(0.28f, 0.31f, 0.36f, 1.00f);
    colors[ImGuiCol_Separator]              = ImVec4(0.25f, 0.28f, 0.32f, 1.00f);
    colors[ImGuiCol_SeparatorHovered]       = ImVec4(0.28f, 0.31f, 0.36f, 1.00f);
    colors[ImGuiCol_SeparatorActive]        = ImVec4(0.31f, 0.34f, 0.40f, 1.00f);
    colors[ImGuiCol_ResizeGrip]             = ImVec4(0.28f, 0.31f, 0.36f, 0.70f);
    colors[ImGuiCol_ResizeGripHovered]      = ImVec4(0.36f, 0.74f, 0.97f, 0.70f);
    colors[ImGuiCol_ResizeGripActive]       = ImVec4(0.46f, 0.84f, 1.00f, 0.95f);
    colors[ImGuiCol_Tab]                    = ImVec4(0.16f, 0.18f, 0.22f, 1.00f);
    colors[ImGuiCol_TabHovered]             = ImVec4(0.26f, 0.28f, 0.32f, 1.00f);
    colors[ImGuiCol_TabActive]              = ImVec4(0.20f, 0.22f, 0.26f, 1.00f);
    colors[ImGuiCol_TabUnfocused]           = ImVec4(0.12f, 0.14f, 0.17f, 1.00f);
    colors[ImGuiCol_TabUnfocusedActive]     = ImVec4(0.16f, 0.18f, 0.22f, 1.00f);
    colors[ImGuiCol_DockingPreview]         = ImVec4(0.36f, 0.74f, 0.97f, 0.70f);
    colors[ImGuiCol_DockingEmptyBg]         = ImVec4(0.12f, 0.14f, 0.17f, 1.00f);
    colors[ImGuiCol_PlotLines]              = ImVec4(0.61f, 0.61f, 0.61f, 1.00f);
    colors[ImGuiCol_PlotLinesHovered]       = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
    colors[ImGuiCol_PlotHistogram]          = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
    colors[ImGuiCol_PlotHistogramHovered]   = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
    colors[ImGuiCol_TextSelectedBg]         = ImVec4(0.36f, 0.74f, 0.97f, 0.35f);
    colors[ImGuiCol_DragDropTarget]         = ImVec4(0.36f, 0.74f, 0.97f, 1.00f);
    colors[ImGuiCol_NavHighlight]           = ImVec4(0.36f, 0.74f, 0.97f, 1.00f);
    colors[ImGuiCol_NavWindowingHighlight]  = ImVec4(0.36f, 0.74f, 0.97f, 0.70f);
    colors[ImGuiCol_NavWindowingDimBg]      = ImVec4(0.20f, 0.23f, 0.26f, 0.20f);
    colors[ImGuiCol_ModalWindowDimBg]       = ImVec4(0.20f, 0.23f, 0.26f, 0.35f);

    style->WindowRounding    = 5.0f;
    style->FrameRounding     = 4.0f;
    style->GrabRounding      = 3.0f;
    style->ScrollbarRounding = 5.0f;
    style->TabRounding       = 4.0f;
    style->ScrollbarSize     = 70.0f;
    style->TouchExtraPadding = ImVec2(5.0, 5.0);
}