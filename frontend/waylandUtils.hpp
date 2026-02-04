struct SDL_Window;
#include <cstdint>
#include "xdg-shell-client-protocol.h"

namespace wayland_utils {
    bool init(SDL_Window* window);

    bool drag_window(SDL_Window* window);

    bool resize_edge(SDL_Window* window, uint32_t edge);

}
