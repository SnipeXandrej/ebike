#include "arc_progress_bar.hpp"
#include "commonUtils.hpp"
#include <format>

void ArcProgressBar::_DrawArc(float size, float max_angle_factor, float input, float thickness, ImVec2 pos, float min_input, float max_input)
{
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    bool isInputNegative = false;
    float inputMapped;
    if (drawArcOutsideInputBoundary) {
        if (input >= min_input) {
            inputMapped = map_f(input, min_input, max_input, 0, 100);
        } else {
            isInputNegative = true;
            inputMapped = map_f(input, min_input, min_input-(max_input+min_input), 0, 100);
        }
    } else {
        inputMapped = map_f(input, min_input, max_input, 0, 100);
    }

    float x = pos.x, y = pos.y;    // Position

    constexpr float ONE_DIV_360f = 1.0f / 360.0f;   // Performance tweak

    float a_min_factor = -1.505f + ((360 - max_angle_factor) * ONE_DIV_360f);    // Angle PI*-1.5f resides in the bottom point of circle
    float a_max_factor_100percentage = (a_min_factor + 1.0f) * -1.0f;       // Arc max factor on 100% percentage state

    float a_factor_delta = (a_max_factor_100percentage - a_min_factor) * ((inputMapped) * 0.01f);
    float a_max_factor = a_min_factor + a_factor_delta;

    float a_factor_delta_green = (a_max_factor_100percentage - a_min_factor) * ((inputMapped) * 0.01f);
    float a_max_factor_green = a_min_factor + a_factor_delta_green + 0.02f;

    ImGui::SetCursorScreenPos(ImVec2(x - 3.5 * scale_ui, y + size * 0.25f * scale_ui));
    ImGui::Text("%0.0f", min_input);

    ImGui::SetCursorScreenPos(ImVec2(x + (size - 15.0) * 0.5 * scale_ui, y + size * 0.25f * scale_ui));
    ImGui::Text("%0.0f", max_input);

    {
        ImGui::PushFont(ImGui::GetFont(), ImGui::GetStyle().FontSizeBase * 1.9 * textScale);
        std::string text = std::format("{:.0f}", input);
        ImVec2 textSize = ImGui::CalcTextSize(text.data());
        ImGui::SetCursorScreenPos(ImVec2(x + (size * 0.25 * scale_ui) - (textSize.x / 2.0), y + (size * 0.25f * scale_ui) - textSize.y));
        ImGui::Text("%s", text.data());
        ImGui::PopFont();
    }

    {
        // std::string text = std::format("{:.0f}", input);
        ImVec2 textSize = ImGui::CalcTextSize(name.data());
        ImGui::SetCursorScreenPos(ImVec2(x + (size * 0.25 * scale_ui) - (textSize.x / 2.0), y + (size * 0.25f * scale_ui)));
        ImGui::Text("%s", name.data());
    }

    ImColor green = ImVec4(0.0f, 0.85f, 0.0f, 1.0f);
    ImColor red = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
    ImColor highlightArcColor;
    ImColor arcInsideColor;

    if (!isInputNegative) {
        highlightArcColor = green;
        arcInsideColor = ColorInside;
    } else {
        highlightArcColor = red;
        arcInsideColor = ColorInsideNegative;
    }

    draw_list->PathArcTo(ImVec2(x + size * 0.25f * scale_ui, y + size * 0.25f * scale_ui), size * 0.25f * scale_ui, 3.141592f * a_min_factor, 3.141592f * a_max_factor_100percentage);
    draw_list->PathStroke(_GetStyleColor(ImGuiCol_Button), ImDrawFlags_None, thickness * 0.5 * scale_ui);

    // Path for progress filling (highlighted arc)
    draw_list->PathArcTo(ImVec2(x + size * 0.25f * scale_ui, y + size * 0.25f * scale_ui), size * 0.25f * scale_ui, 3.141592f * a_min_factor, 3.141592f * a_max_factor_green);
    draw_list->PathStroke(highlightArcColor, ImDrawFlags_None, thickness * 0.5f * scale_ui);

    // inside arc
    draw_list->PathArcTo(ImVec2(x + size * 0.25f * scale_ui, y + size * 0.25f * scale_ui), size * 0.25f * scale_ui, 3.141592f * a_min_factor, 3.141592f * a_max_factor);
    draw_list->PathStroke(arcInsideColor, ImDrawFlags_None, thickness * 0.5f * scale_ui);
}

void ArcProgressBar::ProgressBarArc(float input, ImVec2 pos)
{
    ImGui::SetCursorPos(pos);
    ProgressBarArc(input);
}

void ArcProgressBar::ProgressBarArc(float input)
{
    scale_ui = ImGui::GetFontSize() / ImGui::GetStyle().FontSizeBase;
    const ImVec2 origin = ImGui::GetCursorScreenPos();

    _DrawArc(size, max_angle_factor, input, thickness, ImVec2(origin.x, origin.y), min_input, max_input);

    ImGui::SetCursorPos(origin);
    ImGui::Dummy(ImVec2(
            (size * 0.5f + thickness) * scale_ui,
            (size * 0.25f + thickness) * scale_ui + ImGui::GetTextLineHeight()));
}

void ArcProgressBar::setTextScaling(float input) {
    textScale = input;
}