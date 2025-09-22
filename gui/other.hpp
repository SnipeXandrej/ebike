#include "cpuUsage.h"
#include "serial.hpp"



class MovingAverage {
private:
    float previousInput;
    float Input;
    bool done = 0;

    float smoothValue(float newValue, float previousValue, float smoothingFactor) {
        return previousValue + smoothingFactor * (newValue - previousValue);
    }

public:
    float smoothingFactor = 1;
    float output;

    void initInput(float input) {
        if (!done) {
            previousInput = input;
            done = 1;
        }
    }

    float moveAverage(float input) {
        Input = smoothValue(input, previousInput, smoothingFactor);
        previousInput = Input;

        output = Input;

        return Input;
    }
};

std::vector<std::string> split(const std::string& input, char delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(input);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        result.push_back(token);
    }

    return result;
}


// #define _CRT_SECURE_NO_WARNINGS
// #define STB_IMAGE_IMPLEMENTATION
// #include "stb_image.h"

// // Simple helper function to load an image into a OpenGL texture with common settings
// bool LoadTextureFromMemory(const void* data, size_t data_size, GLuint* out_texture, int* out_width, int* out_height)
// {
//     // Load from file
//     int image_width = 0;
//     int image_height = 0;
//     unsigned char* image_data = stbi_load_from_memory((const unsigned char*)data, (int)data_size, &image_width, &image_height, NULL, 4);
//     if (image_data == NULL)
//         return false;

//     // Create a OpenGL texture identifier
//     GLuint image_texture;
//     glGenTextures(1, &image_texture);
//     glBindTexture(GL_TEXTURE_2D, image_texture);

//     // Setup filtering parameters for display
//     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

//     // Upload pixels into texture
//     glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
//     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
//     stbi_image_free(image_data);

//     *out_texture = image_texture;
//     *out_width = image_width;
//     *out_height = image_height;

//     return true;
// }

// // Open and read a file, then forward to LoadTextureFromMemory()
// bool LoadTextureFromFile(const char* file_name, GLuint* out_texture, int* out_width, int* out_height)
// {
//     FILE* f = fopen(file_name, "rb");
//     if (f == NULL)
//         return false;
//     fseek(f, 0, SEEK_END);
//     size_t file_size = (size_t)ftell(f);
//     if (file_size == -1)
//         return false;
//     fseek(f, 0, SEEK_SET);
//     void* file_data = IM_ALLOC(file_size);
//     fread(file_data, 1, file_size, f);
//     fclose(f);
//     bool ret = LoadTextureFromMemory(file_data, file_size, out_texture, out_width, out_height);
//     IM_FREE(file_data);
//     return ret;
// }


    // int my_image_width = 0;
    // int my_image_height = 0;
    // GLuint my_image_texture = 0;
    // bool ret = LoadTextureFromFile("/home/snipex/ototototto/648A7623.jpg", &my_image_texture, &my_image_width, &my_image_height);

                        // ImGui::Text("pointer = %x", my_image_texture);
                    // ImGui::Text("size = %d x %d", my_image_width, my_image_height);
                    // // ImGui::Image((ImTextureID)(intptr_t)my_image_texture, ImVec2(my_image_width, my_image_height));
                    // // Half size, same contents
                    // ImGui::Image((ImTextureID)(intptr_t)my_image_texture, ImVec2(my_image_width/5, my_image_height/5), ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f));


float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
    float temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    if (temp < out_min)
        temp = out_min;

    if (temp > out_max)
        temp = out_max;

    return temp;
}

void addVUMeter(float input, float input_min, float input_max, const char *label, int precision) {
    int LED_COUNT = 18;
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
        std::cout << "saved: " << setting_name << "=" << value << "\n";
    }

    // Write back to file
    std::ofstream file(SETTINGS_FILEPATH);
    file << tbl;
    file.close();
}

void StyleColorsDarkBreeze(ImGuiStyle* dst = nullptr) {
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
}