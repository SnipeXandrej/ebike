#include <imgui.h>

class ImGuiGesture {
public:
    ImGuiIO& io = ImGui::GetIO();

    void closeGesture();
    void openGesture();

    bool start();
    void beforeEnd();
    void end();
};