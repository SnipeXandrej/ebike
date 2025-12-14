#include <imgui.h>

class ImGuiGesture {
public:
    ImGuiIO& io = ImGui::GetIO();

    void start();
    void end();
};