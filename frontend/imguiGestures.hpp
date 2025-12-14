#include <imgui.h>

class ImGuiGesture {
public:
    ImGuiIO& io = ImGui::GetIO();

    void closeGesture();

    void start();
    void end();
};