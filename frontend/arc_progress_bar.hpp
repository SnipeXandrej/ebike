/**
 * @file arc_progress_bar.hpp
 * @brief Provide a arc progress bar widget for ImGui. Though it has been modified a little :)
 *
 * @author AnClark Liu
 * @author Andrej Halveland
 * @date 2025-10-07
 * @license MIT
 */


#include <imgui.h>
#include <iostream>

class ArcProgressBar {
public:
    void init(float _size, float _max_angle_factor, float _thickness, float _min_input, float _max_input, bool _drawArcOutsideInputBoundary, std::string _name) {
        size = _size;
        max_angle_factor = _max_angle_factor;
        thickness = _thickness;
        min_input = _min_input;
        max_input = _max_input;
        drawArcOutsideInputBoundary = _drawArcOutsideInputBoundary;
        name = _name;
    }

    void setTextScaling(float input);

    void ProgressBarArc(float input, ImVec2 pos);

    void ProgressBarArc(float input);

    inline ImColor _GetStyleColor(const ImGuiCol_ color_id) {
        IM_ASSERT(color_id >= 0 && color_id < ImGuiCol_COUNT);

        if (color_id < 0 || color_id >= ImGuiCol_COUNT)
            return ImColor(DEFAULT_FOREGROUND_COLOR);    // Fallback

        return ImColor(ImGui::GetStyle().Colors[color_id]);
    }

    float size;
    float max_angle_factor;
    float thickness;
    float min_input;
    float max_input;
    bool drawArcOutsideInputBoundary;
    std::string name;

private:
    float textScale = 0.8;
    void _DrawArc(float size, float max_angle_factor, float input, float thickness, ImVec2 pos, float min_input, float max_input);

    const ImColor DEFAULT_FOREGROUND_COLOR = ImColor(ImVec4(1.0f, 1.0f, 0.4f, 1.0f));
    const ImColor ColorInside = ImColor(ImVec4(0.7f, 0.0f, 0.0f, 1.0f));
    const ImColor ColorInsideNegative = ImColor(ImVec4(1.0f, 0.64f, 0.027f, 1.0f));
};