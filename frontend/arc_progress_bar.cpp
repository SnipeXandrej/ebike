#include "arc_progress_bar.hpp"
#include <format>

void ArcProgressBar::_DrawArc(float size, float max_angle_factor, float input, float thickness, ImVec2 pos, float min_input, float max_input)
{
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    float inputMapped = map_f(input, min_input, max_input, 0, 100);

    float x = pos.x, y = pos.y;    // Position

    constexpr float ONE_DIV_360f = 1.0f / 360.0f;   // Performance tweak

    float a_min_factor, a_max_factor, a_max_factor_100percentage;
    a_min_factor = -1.5f + ((360 - max_angle_factor) * ONE_DIV_360f);    // Angle PI*-1.5f resides in the bottom point of circle
    a_max_factor_100percentage = (a_min_factor + 1.0f) * -1.0f;       // Arc max factor on 100% percentage state

    float a_factor_delta = (a_max_factor_100percentage - a_min_factor) * (inputMapped * 0.01f);
    a_max_factor = a_min_factor + a_factor_delta;

    ImGui::SetCursorPos(ImVec2(x - 7.0, y + size * 0.5f));
    ImGui::Text("%0.0f", min_input);

    ImGui::SetCursorPos(ImVec2(x + size - 15.0, y + size * 0.5f));
    ImGui::Text("%0.0f", max_input);

    {
        ImGui::PushFont(ImGui::GetFont(), ImGui::GetFontSize() * 0.75);
        std::string text = std::format("{:.0f}", input);
        ImVec2 textSize = ImGui::CalcTextSize(text.data());
        ImGui::SetCursorPos(ImVec2((x + (size * 0.5f)) - (textSize.x * 0.5), y + (size * 0.20)));
        ImGui::Text("%s", text.data());
        ImGui::PopFont();
    }

    {
        // std::string text = std::format("{:.0f}", input);
        ImVec2 textSize = ImGui::CalcTextSize(name.data());
        ImGui::SetCursorPos(ImVec2((x + (size * 0.5f)) - (textSize.x * 0.5), y + (size * 0.5)));
        ImGui::Text("%s", name.data());
    }

    // Path for background arc (dimmed arc)
    draw_list->PathArcTo(ImVec2(x + size * 0.5f, y + size * 0.5f), size * 0.5f, 3.141592f * a_min_factor, 3.141592f * a_max_factor_100percentage);
    draw_list->PathStroke(_GetStyleColor(ImGuiCol_Button), ImDrawFlags_None, thickness);

    // Path for progress filling (highlighted arc)
    draw_list->PathArcTo(ImVec2(x + size * 0.5f, y + size * 0.5f), size * 0.5f, 3.141592f * a_min_factor, 3.141592f * a_max_factor);
    draw_list->PathStroke(ColorInside, ImDrawFlags_None, thickness);
}

void ArcProgressBar::ProgressBarArc(float input, ImVec2 pos)
{
    // By default, ImDrawList draws in absolute position relaitve to the canvas, NOT the window.
    // So we need to consider the window position if we want to draw something inside the window.

    ImVec2 window_pos = ImGui::GetWindowPos();    // Do not forget to get window's position.

    // And do not forget to add the window position to our own position.
    _DrawArc(size, max_angle_factor, input, thickness, ImVec2(pos.x + window_pos.x, pos.y + window_pos.y), min_input, max_input);
}

void ArcProgressBar::ProgressBarArc(float input)
{
    ImVec2 pos = ImGui::GetCursorScreenPos(); // Get current insert point
    ImGui::Dummy(ImVec2(size, size));   // Placeholder

    ProgressBarArc(input, pos);
}