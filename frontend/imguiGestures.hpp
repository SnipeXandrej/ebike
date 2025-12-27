#include <imgui.h>

class ImGuiGesture {
public:
    ImGuiIO& io = ImGui::GetIO();

    void closeGesture();
    void openGesture();

    void start();
    void end();
};