// Wayland xdg-shell window move using xdg_toplevel_move.

#include "waylandUtils.hpp"
#include <SDL3/SDL.h>
#include <SDL3/SDL_video.h>
#include <wayland-client.h>

#include <cstring>

    struct wl_display* display = nullptr;
    struct wl_registry* registry = nullptr;
    struct wl_seat* seat = nullptr;
    struct wl_pointer* pointer = nullptr;

    uint32_t last_pointer_serial = 0;
    bool initialized = false;

    void pointer_enter(void*, struct wl_pointer*, uint32_t serial, struct wl_surface*, wl_fixed_t, wl_fixed_t) {
        last_pointer_serial = serial;
    }

    void pointer_leave(void*, struct wl_pointer*, uint32_t, struct wl_surface*) {}
    void pointer_motion(void*, struct wl_pointer*, uint32_t, wl_fixed_t, wl_fixed_t) {}
    void pointer_button(void*, struct wl_pointer*, uint32_t serial, uint32_t, uint32_t, uint32_t) {
        last_pointer_serial = serial;
    }

    void pointer_axis(void*, struct wl_pointer*, uint32_t, uint32_t, wl_fixed_t) {}
    void pointer_frame(void*, struct wl_pointer*) {}
    void pointer_axis_source(void*, struct wl_pointer*, uint32_t) {}
    void pointer_axis_stop(void*, struct wl_pointer*, uint32_t, uint32_t) {}
    void pointer_axis_discrete(void*, struct wl_pointer*, uint32_t, int32_t) {}
    void pointer_axis_value120(void*, struct wl_pointer*, uint32_t, int32_t) {}
    void pointer_axis_relative_direction(void*, struct wl_pointer*, uint32_t, uint32_t) {}

    static const struct wl_pointer_listener pointer_listener = {
        pointer_enter,
        pointer_leave,
        pointer_motion,
        pointer_button,
        pointer_axis,
        pointer_frame,
        pointer_axis_source,
        pointer_axis_stop,
        pointer_axis_discrete,
        pointer_axis_value120,
        pointer_axis_relative_direction,
    };

    void seat_capabilities(void*, struct wl_seat* seat, uint32_t capabilities) {
        if ((capabilities & WL_SEAT_CAPABILITY_POINTER) && pointer == nullptr) {
            pointer = wl_seat_get_pointer(seat);
            if (pointer)
                wl_pointer_add_listener(pointer, &pointer_listener, nullptr);
        }
    }

    void seat_name(void*, struct wl_seat*, const char*) {}

    static const struct wl_seat_listener seat_listener = {
        seat_capabilities,
        seat_name,
    };

    void registry_global(void*, struct wl_registry* registry, uint32_t name, const char* interface, uint32_t version) {
        if (std::strcmp(interface, wl_seat_interface.name) == 0 && seat == nullptr) {
            uint32_t bind_version = (version < 7u) ? version : 7u;
            seat = static_cast<wl_seat*>(wl_registry_bind(registry, name, &wl_seat_interface, bind_version));
            if (seat)
                wl_seat_add_listener(seat, &seat_listener, nullptr);
        }
    }

    void registry_global_remove(void*, struct wl_registry*, uint32_t) {}

    static const struct wl_registry_listener registry_listener = {
        registry_global,
        registry_global_remove,
    };

namespace wayland_utils {
    bool init(SDL_Window* window)
    {
        if (initialized)
            return (display != nullptr);

        const char* driver = SDL_GetCurrentVideoDriver();
        if (!driver || std::strcmp(driver, "wayland") != 0)
            return false;

        SDL_PropertiesID props = SDL_GetWindowProperties(window);
        if (!props)
            return false;

        display = static_cast<wl_display*>(SDL_GetPointerProperty(props, SDL_PROP_WINDOW_WAYLAND_DISPLAY_POINTER, nullptr));
        if (!display)
            return false;

        registry = wl_display_get_registry(display);
        if (!registry)
            return false;

        wl_registry_add_listener(registry, &registry_listener, nullptr);
        wl_display_roundtrip(display);  // get globals (seat)
        wl_display_roundtrip(display);  // get seat capabilities (pointer)

        wl_registry_destroy(registry);
        registry = nullptr;

        initialized = true;
        return (seat != nullptr && pointer != nullptr);
    }

    bool drag_window(SDL_Window* window)
    {
        if (!initialized || !seat || last_pointer_serial == 0)
            return false;

        SDL_PropertiesID props = SDL_GetWindowProperties(window);
        if (!props)
            return false;

        struct xdg_toplevel* toplevel = static_cast<struct xdg_toplevel*>(SDL_GetPointerProperty(props, SDL_PROP_WINDOW_WAYLAND_XDG_TOPLEVEL_POINTER, nullptr));
        if (!toplevel)
            return false;

        xdg_toplevel_move(toplevel, seat, last_pointer_serial);
        return true;
    }

    bool resize_edge(SDL_Window* window, uint32_t edge) {
        if (!initialized || !seat || last_pointer_serial == 0)
            return false;

        SDL_PropertiesID props = SDL_GetWindowProperties(window);
        if (!props)
            return false;

        struct xdg_toplevel* toplevel = static_cast<struct xdg_toplevel*>(SDL_GetPointerProperty(props, SDL_PROP_WINDOW_WAYLAND_XDG_TOPLEVEL_POINTER, nullptr));
        if (!toplevel)
            return false;

        xdg_toplevel_resize(toplevel, seat, last_pointer_serial, edge);
        return true;
    }
}
