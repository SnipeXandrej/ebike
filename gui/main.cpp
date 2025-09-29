// Dear ImGui: standalone example application for SDL3 + OpenGL
// (SDL is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/ folder).
// - Introduction, links and more at the top of imgui.cpp


#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include <SDL3/SDL.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL3/SDL_opengles2.h>
#else
#include <SDL3/SDL_opengl.h>
#endif

#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include "toml.hpp"
#include <functional>
#include <print>
#include <format>

#include "serial.hpp"
#include "other.hpp"

float MAX_WATTAGE = 7500.0;


enum COMMAND_ID {
    GET_BATTERY = 0,
    ARE_YOU_ALIVE = 1,
    GET_STATS = 2,
    RESET_ESTIMATED_RANGE = 3,
    RESET_TRIP = 4,
    READY_FOR_MESSAGE = 5,
    SET_ODOMETER = 6,
    SAVE_PREFERENCES = 7,
    READY_TO_WRITE = 8,
    GET_FW = 9,
    PING = 10,
    TOGGLE_FRONT_LIGHT = 11,
    ESP32_SERIAL_LENGTH = 12,
    SET_AMPHOURS_USED_LIFETIME = 13,
    GET_VESC_MCCONF = 14,
    SET_VESC_MCCONF = 15,
};

enum POWER_PROFILE {
    LEGAL = 0,
    ECO = 1,
    BALANCED = 2,
    PERFORMANCE = 3
};

int POWER_PROFILE_CURRENT;

// 125.48 RPM per km/h

struct VESC_MCCONF {
    float l_current_min_scale;
    float l_current_max_scale;
    float l_min_erpm;
    float l_max_erpm;
    float l_min_duty;
    float l_max_duty;
    float l_watt_min;
    float l_watt_max;
    float l_in_current_min;
    float l_in_current_max;
    std::string name = "";
    int id;
    // int motor_poles;
    // float gear_ratio;
    // float wheel_diameter;
};

VESC_MCCONF esp32_vesc_mcconf, mcconf_current, mcconf_legal, mcconf_eco, mcconf_balanced, mcconf_performance;

struct {
    float totalSecondsSinceBoot = 0;
    uint64_t clockSecondsSinceBoot = 0;
    uint64_t clockMinutesSinceBoot = 0;
    uint64_t clockHoursSinceBoot = 0;
    uint64_t clockDaysSinceBoot = 0;

    float speed_kmh;
    float motor_rpm;
    float odometer_distance;
    float trip_distance;
    float gear_level;
    float gear_maxCurrent;
    float power_level;
    float phase_current;
    float wh_over_km_average;
    float ah_per_km;
    float range_left;
    float temperature_motor;
    float timeCore0_us;
    float timeCore1_us;
    float acceleration;
    bool power_on = false;

    std::string fw_name;
    std::string fw_version;
    std::string fw_compile_date_time;
} esp32;

// Limit FPS
struct {
    float TARGET_FPS;
    bool LIMIT_FRAMERATE;
    std::string serialPortName;
    int serialWriteWaitMs;
    int powerProfile;
} settings;

struct {
    float voltage;
    float voltage_min;
    float voltage_max;
    float current;
    float ampHoursUsed;
    float ampHoursUsedLifetime;
    float watts;
    float wattHoursUsed;
    float percentage;
    float ampHoursRated;
    float ampHoursRated_tmp;
    float amphours_min_voltage;
    float amphours_max_voltage;
} battery;

struct {
    MovingAverage current;
    MovingAverage acceleration;
    MovingAverage wattage;
    MovingAverage wattageMoreSmooth;
    MovingAverage whOverKm;
} movingAverages;

bool done = false;
bool ready_to_write = true;

// TODO: do not hardcode filepaths :trol:
const char* SETTINGS_FILEPATH = "/home/snipex/.config/ebikegui/settings.toml";
char hostname[1024];

char *desktopEnvironment;

auto timeDrawStart = std::chrono::steady_clock::now();
auto timeDrawEnd = std::chrono::steady_clock::now();
auto timeDrawDiff = std::chrono::duration<double, std::milli>(timeDrawEnd - timeDrawStart).count();

auto timeRenderStart = std::chrono::steady_clock::now();
auto timeRenderEnd = std::chrono::steady_clock::now();
auto timeRenderDiff = std::chrono::duration<double, std::milli>(timeRenderEnd - timeRenderStart).count();

auto timePingStart = std::chrono::steady_clock::now();
auto timePingEnd = std::chrono::steady_clock::now();
auto timePingDiff = std::chrono::duration<double, std::milli>(timeRenderEnd - timeRenderStart).count();

float voltage_last_values_array[140];
float current_last_values_array[140];
float wattage_last_values_array[2000];
float temperature_last_values_array[2000];

std::string to_send = "";
std::string to_send_extra = "";

CPUUsage cpuUsage_ImGui;
CPUUsage cpuUsage_SerialThread;
CPUUsage cpuUsage_Everything;
SerialProcessor SerialP;

void setBrightnessLow() {
    // std::system("brightnessctl set 0%");
    if (strcmp(desktopEnvironment, "KDE") == 0) {
        std::system("kscreen-doctor --dpms off");
    }
}

void setBrightnessHigh() {
    // std::system("brightnessctl set 100%");
    if (strcmp(desktopEnvironment, "KDE") == 0) {
        std::system("kscreen-doctor --dpms on");
    }
}

void setMcconfValues(VESC_MCCONF mcconf) {
    std::string append = std::format("{};{};{};{};{};{};{};{};{};{};{};{};\n"
                                        ,static_cast<int>(COMMAND_ID::SET_VESC_MCCONF)
                                        ,mcconf.l_current_min_scale
                                        ,mcconf.l_current_max_scale
                                        ,mcconf.l_min_erpm
                                        ,mcconf.l_max_erpm
                                        ,mcconf.l_min_duty
                                        ,mcconf.l_max_duty
                                        ,mcconf.l_watt_min
                                        ,mcconf.l_watt_max
                                        ,mcconf.l_in_current_min
                                        ,mcconf.l_in_current_max
                                        ,mcconf.name
    );
    to_send_extra.append(append);
}

void setPowerProfile(int PROFILE) {
    POWER_PROFILE_CURRENT = PROFILE;
    settings.powerProfile = PROFILE;

    switch (PROFILE) {
        case POWER_PROFILE::LEGAL:
            mcconf_current = mcconf_legal;
            break;
        case POWER_PROFILE::ECO:
            mcconf_current = mcconf_eco;
            break;
        case POWER_PROFILE::BALANCED:
            mcconf_current = mcconf_balanced;
            break;
        case POWER_PROFILE::PERFORMANCE:
            mcconf_current = mcconf_performance;
            break;
    }
    setMcconfValues(mcconf_current);
}

void processSerialRead(std::string line) {
        if (!line.empty()) {
            // std::cout << "Received: " << line << "\n";
            int index = 1;
            int command_id;
            auto packet = split(line, ';');
            try {
                command_id = std::stoi(packet[0]);
            } catch(...) {
                std::println("Failed to convert command_id stoi()");
                command_id = -1;
            }

            if (command_id == COMMAND_ID::ARE_YOU_ALIVE) {
                    std::cout << "[SERIAL] Succesful communication with Atmega8!" << "\n";
                    SerialP.succesfulCommunication = true;
            }

            if (SerialP.succesfulCommunication) {
                switch (command_id) {
                    case COMMAND_ID::GET_BATTERY:
                        battery.voltage = getValueFromPacket(packet, &index);
                        battery.current = getValueFromPacket(packet, &index);
                        battery.watts = getValueFromPacket(packet, &index);
                        battery.wattHoursUsed = getValueFromPacket(packet, &index);
                        battery.ampHoursUsed = getValueFromPacket(packet, &index);
                        battery.ampHoursUsedLifetime = getValueFromPacket(packet, &index);
                        battery.ampHoursRated = getValueFromPacket(packet, &index);
                        battery.percentage = getValueFromPacket(packet, &index);
                        battery.voltage_min = getValueFromPacket(packet, &index);
                        battery.voltage_max = getValueFromPacket(packet, &index);
                        battery.amphours_min_voltage = getValueFromPacket(packet, &index);
                        battery.amphours_max_voltage = getValueFromPacket(packet, &index);

                        addValueToArray(2000, wattage_last_values_array, battery.watts);
                        break;

                    case COMMAND_ID::GET_STATS:
                        esp32.speed_kmh = getValueFromPacket(packet, &index);
                        esp32.motor_rpm = getValueFromPacket(packet, &index);
                        esp32.odometer_distance = getValueFromPacket(packet, &index);
                        esp32.trip_distance = getValueFromPacket(packet, &index);
                        esp32.gear_level = getValueFromPacket(packet, &index);
                        esp32.gear_maxCurrent = getValueFromPacket(packet, &index);
                        esp32.power_level = getValueFromPacket(packet, &index);
                        esp32.phase_current = getValueFromPacket(packet, &index);
                        esp32.wh_over_km_average = getValueFromPacket(packet, &index);
                        esp32.ah_per_km = getValueFromPacket(packet, &index);
                        esp32.range_left = getValueFromPacket(packet, &index);
                        esp32.temperature_motor = getValueFromPacket(packet, &index);
                        esp32.totalSecondsSinceBoot = getValueFromPacket(packet, &index);
                        esp32.timeCore0_us = getValueFromPacket(packet, &index);
                        esp32.timeCore1_us = getValueFromPacket(packet, &index);
                        esp32.acceleration = getValueFromPacket(packet, &index);
                        esp32.power_on = (bool)getValueFromPacket(packet, &index);

                        esp32.clockSecondsSinceBoot = (uint64_t)(esp32.totalSecondsSinceBoot) % 60;
                        esp32.clockMinutesSinceBoot = (uint64_t)(esp32.totalSecondsSinceBoot / 60.0) % 60;
                        esp32.clockHoursSinceBoot   = (uint64_t)(esp32.totalSecondsSinceBoot / 60.0 / 60.0) % 24;
                        esp32.clockDaysSinceBoot    = esp32.totalSecondsSinceBoot / 60.0 / 60.0 / 24;

                        addValueToArray(2000, temperature_last_values_array, esp32.temperature_motor);
                        break;

                    case COMMAND_ID::GET_FW:
                        esp32.fw_name = getValueFromPacket_string(packet, &index);
                        esp32.fw_version = getValueFromPacket_string(packet, &index);
                        esp32.fw_compile_date_time = getValueFromPacket_string(packet, &index);
                        break;

                    case COMMAND_ID::READY_TO_WRITE:
                        ready_to_write = true;
                        break;

                    case COMMAND_ID::GET_VESC_MCCONF:
                        esp32_vesc_mcconf.l_current_min_scale = getValueFromPacket(packet, &index);
                        esp32_vesc_mcconf.l_current_max_scale = getValueFromPacket(packet, &index);
                        esp32_vesc_mcconf.l_min_erpm = getValueFromPacket(packet, &index);
                        esp32_vesc_mcconf.l_max_erpm = getValueFromPacket(packet, &index);
                        esp32_vesc_mcconf.l_min_duty = getValueFromPacket(packet, &index);
                        esp32_vesc_mcconf.l_max_duty = getValueFromPacket(packet, &index);
                        esp32_vesc_mcconf.l_watt_min = getValueFromPacket(packet, &index);
                        esp32_vesc_mcconf.l_watt_max = getValueFromPacket(packet, &index);
                        esp32_vesc_mcconf.l_in_current_min = getValueFromPacket(packet, &index);
                        esp32_vesc_mcconf.l_in_current_max = getValueFromPacket(packet, &index);
                        esp32_vesc_mcconf.name = getValueFromPacket_string(packet, &index);
                        break;
                }
            }
        }
}

// Main code
int main(int, char**)
{
    setenv("SDL_VIDEODRIVER", "wayland", 1);

    // ##########################
    // ##### Hostname stuff #####
    // ##########################
    gethostname(hostname, sizeof(hostname));
    printf("Hostname = %s\n", hostname);

    desktopEnvironment = getenv("XDG_CURRENT_DESKTOP");
    printf("Desktop Environment = %s\n", desktopEnvironment);

    // ####################
    // ##### Settings #####
    // ####################

    mcconf_legal.l_current_min_scale = 1.0;
    mcconf_legal.l_current_max_scale = 0.15;
    mcconf_legal.l_min_erpm = -(500*3);
    mcconf_legal.l_max_erpm = (3137*3); // 25km/h
    mcconf_legal.l_min_duty = 0.005;
    mcconf_legal.l_max_duty = 0.95;
    mcconf_legal.l_watt_min = -10000;
    mcconf_legal.l_watt_max = 250;
    mcconf_legal.l_in_current_min = -10;
    mcconf_legal.l_in_current_max = 96;
    mcconf_legal.name = "Legal";
    mcconf_legal.id = POWER_PROFILE::LEGAL;

    mcconf_eco.l_current_min_scale = 1.0;
    mcconf_eco.l_current_max_scale = 0.3;
    mcconf_eco.l_min_erpm = -(500*3);
    mcconf_eco.l_max_erpm = (3137*3); // 25km/h
    mcconf_eco.l_min_duty = 0.005;
    mcconf_eco.l_max_duty = 0.95;
    mcconf_eco.l_watt_min = -10000;
    mcconf_eco.l_watt_max = 750;
    mcconf_eco.l_in_current_min = -10;
    mcconf_eco.l_in_current_max = 96;
    mcconf_eco.name = "Eco";
    mcconf_eco.id = POWER_PROFILE::ECO;

    mcconf_balanced.l_current_min_scale = 1.0;
    mcconf_balanced.l_current_max_scale = 0.4;
    mcconf_balanced.l_min_erpm = -(500*3);
    mcconf_balanced.l_max_erpm = (5020*3); // 40km/h
    mcconf_balanced.l_min_duty = 0.005;
    mcconf_balanced.l_max_duty = 0.95;
    mcconf_balanced.l_watt_min = -10000;
    mcconf_balanced.l_watt_max = 2500;
    mcconf_balanced.l_in_current_min = -10;
    mcconf_balanced.l_in_current_max = 96;
    mcconf_balanced.name = "Balanced";
    mcconf_balanced.id = POWER_PROFILE::BALANCED;

    mcconf_performance.l_current_min_scale = 1.0;
    mcconf_performance.l_current_max_scale = 1.0;
    mcconf_performance.l_min_erpm = -(500*3);
    mcconf_performance.l_max_erpm = (6901*3); // 55km/h
    mcconf_performance.l_min_duty = 0.005;
    mcconf_performance.l_max_duty = 0.95;
    mcconf_performance.l_watt_min = -10000;
    mcconf_performance.l_watt_max = 7000;
    mcconf_performance.l_in_current_min = -10;
    mcconf_performance.l_in_current_max = 96;
    mcconf_performance.name = "Performance";
    mcconf_performance.id = POWER_PROFILE::PERFORMANCE;

    movingAverages.current.smoothingFactor = 0.7f;
    movingAverages.acceleration.smoothingFactor = 0.5f;
    movingAverages.wattage.smoothingFactor = 0.6f;
    movingAverages.wattageMoreSmooth.smoothingFactor = 0.1f;
    movingAverages.whOverKm.smoothingFactor = 0.1f;

    // ########################
    // ######### TOML #########
    // ########################

    // TODO: if settings.toml doesnt exist, create it
    toml::table tbl;
    tbl = toml::parse_file(SETTINGS_FILEPATH);

    // values
    settings.TARGET_FPS         = tbl["settings"]["framerate"].value_or(60);
    settings.LIMIT_FRAMERATE    = tbl["settings"]["limit_framerate"].value_or(0);
    settings.serialWriteWaitMs  = tbl["settings"]["serialWriteWaitMs"].value_or(50);
    settings.powerProfile       = tbl["settings"]["powerProfile"].value_or(POWER_PROFILE::BALANCED);

    // strings
    if (auto val = tbl.at_path("settings.serialPortName").value<std::string>()) {
        settings.serialPortName = *val;
    }

    setPowerProfile(settings.powerProfile);

    // ########################
    // ##### Serial Port ######
    // ########################

    std::thread backend([&]() -> int {
        std::cout << "[SERIAL] Initializing" << "\n";

        SerialP.init(settings.serialPortName.c_str());

        std::cout << "[SERIAL] Entering main while loop\n";
        while(!done) {
            cpuUsage_SerialThread.measureStart(1);
            SerialP.timeout_ms = settings.serialWriteWaitMs;

            to_send = "";

            if (!SerialP.succesfulCommunication) {
                std::string to_send = std::to_string(COMMAND_ID::ARE_YOU_ALIVE);
                SerialP.writeSerial(to_send.c_str());

                // hol'up
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (SerialP.succesfulCommunication) { //  && ready_to_write

                timePingEnd = std::chrono::steady_clock::now();
                if (std::chrono::duration<double, std::milli>(timePingEnd - timePingStart).count() >= 750.0) {
                    timePingStart = std::chrono::steady_clock::now();

                    commAddValue(&to_send, COMMAND_ID::PING, 0);
                    to_send.append("\n");

                    commAddValue(&to_send, COMMAND_ID::GET_FW, 0);
                    to_send.append("\n");

                    static int GET_VESC_MCCONF_COUNTER = 0;

                    GET_VESC_MCCONF_COUNTER++;
                    if (GET_VESC_MCCONF_COUNTER >= 3) {
                        GET_VESC_MCCONF_COUNTER = 0;

                        if (esp32_vesc_mcconf.name != mcconf_current.name) {
                            setPowerProfile(mcconf_current.id);
                        }


                        commAddValue(&to_send, COMMAND_ID::GET_VESC_MCCONF, 0);
                        to_send.append("\n");
                    }
                }

                commAddValue(&to_send, COMMAND_ID::GET_BATTERY, 0);
                to_send.append("\n");
                commAddValue(&to_send, COMMAND_ID::GET_STATS, 0);
                to_send.append("\n");

                to_send.append(to_send_extra);
                to_send_extra = "";

                SerialP.writeSerial(to_send.c_str());
            }

            SerialP.readSerial(processSerialRead);
            cpuUsage_SerialThread.measureEnd(1);
        }

        close(SerialP.serialPort);
        return 0;
    });



    // ###########################
    // ##### SDL/ Dear ImGUI #####
    // ###########################

    // [If using SDL_MAIN_USE_CALLBACKS: all code below until the main loop starts would likely be your SDL_AppInit() function]
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD))
    {
        printf("Error: SDL_Init(): %s\n", SDL_GetError());
        return -1;
    }

    // Decide GL+GLSL versions
    #if defined(IMGUI_IMPL_OPENGL_ES2)
        // GL ES 2.0 + GLSL 100 (WebGL 1.0)
        const char* glsl_version = "#version 100";
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    #elif defined(IMGUI_IMPL_OPENGL_ES3)
        // GL ES 3.0 + GLSL 300 es (WebGL 2.0)
        const char* glsl_version = "#version 300 es";
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    #elif defined(__APPLE__)
        // GL 3.2 Core + GLSL 150
        const char* glsl_version = "#version 150";
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG); // Always required on Mac
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    #else
        // GL 3.0 + GLSL 130
        const char* glsl_version = "#version 130";
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    #endif


    // Create window with graphics context
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    float main_scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
    SDL_WindowFlags window_flags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY;
    SDL_Window* window = SDL_CreateWindow("E-BIKE GUI", (int)(1257 * main_scale), (int)(583 * main_scale), window_flags);
    if (window == nullptr)
    {
        printf("Error: SDL_CreateWindow(): %s\n", SDL_GetError());
        return -1;
    }
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (gl_context == nullptr)
    {
        printf("Error: SDL_GL_CreateContext(): %s\n", SDL_GetError());
        return -1;
    }

    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    SDL_ShowWindow(window);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;       // Enable Viewports
    io.ConfigFlags |= ImGuiConfigFlags_IsTouchScreen;
    // io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking

    // Setup Dear ImGui style
    // ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();
    StyleColorsDarkBreeze(nullptr);

    // Setup scaling
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale);        // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing this requires resetting Style + calling this again)
    style.FontScaleDpi = main_scale;        // Set initial font scale. (using io.ConfigDpiScaleFonts=true makes this unnecessary. We leave both here for documentation purpose)

    // Setup Platform/Renderer backends
    ImGui_ImplSDL3_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    style.FontScaleDpi = 2.0f;

    // Main loop
    while (!done)
    {
        cpuUsage_ImGui.measureStart(1);
        cpuUsage_Everything.measureStart(0);

        static bool power_on_old = false;
        if (esp32.power_on && power_on_old != esp32.power_on) {
            power_on_old = esp32.power_on;

            // Run this when the bike gets powered on
            std::thread(setBrightnessHigh).detach();
        } else if (!esp32.power_on && power_on_old != esp32.power_on) {
            power_on_old = esp32.power_on;

            // Run this when the bike gets powered off
            std::thread(setBrightnessLow).detach();
        }

        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT)
                done = true;
            if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(window))
                done = true;
        }

        // [If using SDL_MAIN_USE_CALLBACKS: all code below would likely be your SDL_AppIterate() function]
        if (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED)
        {
            SDL_Delay(10);
            continue;
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        // ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->Pos);
        ImGui::SetNextWindowSize(io.DisplaySize);
        char text[1024];
        if (true) {
            timeDrawStart = std::chrono::steady_clock::now();
            ImGui::Begin("Main", nullptr, ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoDecoration|ImGuiWindowFlags_NoBringToFrontOnFocus); // Create a window called "Main" and append into it. //ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoMove

            // Account for the notch on the device's left side
            ImGui::Dummy(ImVec2(28.0, 10.0));
            ImGui::SameLine();
            ImGui::BeginGroup();

            ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_FittingPolicyScroll;
            if (ImGui::BeginTabBar("Tabbar", tab_bar_flags)) {
                if (ImGui::BeginTabItem("Main"))
                {
                    {
                        // Clock
                        time_t currentTime;
                        struct tm *localTime;

                        time( &currentTime );
                        localTime = localtime( &currentTime );

                        char text[100];
                        sprintf(text, "%02d:%02d:%02d  %02d.%02d.%d", localTime->tm_hour, localTime->tm_min, localTime->tm_sec, localTime->tm_mday, localTime->tm_mon+1, localTime->tm_year+1900);
                        ImVec2 textSize = ImGui::CalcTextSize(text);
                        ImGui::SetCursorPos(ImVec2((io.DisplaySize.x / 2.0) - (textSize.x / 2.0), 7.0));
                        ImGui::Text(text);
                    }

                    ImGui::BeginGroup(); // Starts here
                        ImGui::BeginGroup();
                            static bool succesfulCommunication_avoidMutex;
                            if (!SerialP.succesfulCommunication) {
                                succesfulCommunication_avoidMutex = false;
                                ImGui::BeginDisabled();
                            } else {
                                succesfulCommunication_avoidMutex = true;
                            }
                            ImGui::Text("Power");

                            ImGui::PushID(0);
                            ImGui::PushStyleColor(ImGuiCol_FrameBg, (ImVec4)ImColor::HSV(2 / 7.0f, 0.5f, 0.5f));
                            ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, (ImVec4)ImColor::HSV(2 / 7.0f, 0.6f, 0.5f));
                            ImGui::PushStyleColor(ImGuiCol_FrameBgActive, (ImVec4)ImColor::HSV(2 / 7.0f, 0.7f, 0.5f));
                            ImGui::PushStyleColor(ImGuiCol_SliderGrab, (ImVec4)ImColor::HSV(2 / 7.0f, 0.9f, 0.9f));
                            ImGui::VSliderFloat("##v", ImVec2(36*2, 160), &battery.voltage, battery.amphours_min_voltage, battery.amphours_max_voltage, "%0.1fV");
                            ImGui::PopStyleColor(4);
                            ImGui::PopID();

                            ImGui::PushID(1);
                            ImGui::SameLine();
                            ImGui::PushStyleColor(ImGuiCol_FrameBg, (ImVec4)ImColor::HSV(0 / 7.0f, 0.5f, 0.5f));
                            ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, (ImVec4)ImColor::HSV(0 / 7.0f, 0.6f, 0.5f));
                            ImGui::PushStyleColor(ImGuiCol_FrameBgActive, (ImVec4)ImColor::HSV(0 / 7.0f, 0.7f, 0.5f));
                            ImGui::PushStyleColor(ImGuiCol_SliderGrab, (ImVec4)ImColor::HSV(0 / 7.0f, 0.9f, 0.9f));
                            ImGui::VSliderFloat("##v", ImVec2(36*2, 160), &battery.current, 0.0f, 100.0f, "%0.2fA");
                            ImGui::PopStyleColor(4);
                            ImGui::PopID();

                            movingAverages.wattageMoreSmooth.moveAverage(battery.watts);
                            ImGui::PushID(2);
                            ImGui::SameLine();
                            ImGui::PushStyleColor(ImGuiCol_FrameBg, (ImVec4)ImColor::HSV(1 / 7.0f, 0.5f, 0.5f));
                            ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, (ImVec4)ImColor::HSV(1 / 7.0f, 0.6f, 0.5f));
                            ImGui::PushStyleColor(ImGuiCol_FrameBgActive, (ImVec4)ImColor::HSV(1 / 7.0f, 0.7f, 0.5f));
                            ImGui::PushStyleColor(ImGuiCol_SliderGrab, (ImVec4)ImColor::HSV(1 / 7.0f, 0.9f, 0.9f));
                            char sliderFormat[100];
                            // sprintf(sliderFormat, "%0.0fW\n%0.0fW\n%0.0f%%/h", battery.watts, movingAverages.wattageMoreSmooth.output, 100.0 / ((battery.ampHoursRated*72.0) / movingAverages.wattageMoreSmooth.output));
                            sprintf(sliderFormat, "%0.0fW\n%0.0fW\n", battery.watts, movingAverages.wattageMoreSmooth.output);
                            ImGui::VSliderFloat("##v", ImVec2(36*2, 160), &battery.watts, 0.0f, MAX_WATTAGE, sliderFormat);
                            ImGui::PopStyleColor(4);
                            ImGui::PopID();
                        ImGui::EndGroup();
                        //
                        ImGui::SameLine();
                        ImGui::Dummy(ImVec2(20,0));
                        // ImGui::SameLine();
                        //
                        ImGui::BeginGroup();
                            ImGui::Text("Power history");

                            ImGui::PushID(20);
                            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, (ImVec4)ImColor::HSV(2 / 7.0f, 0.9f, 0.9f));
                            ImGui::PlotHistogram("##", wattage_last_values_array, IM_ARRAYSIZE(wattage_last_values_array), 0, NULL, 0.0f, MAX_WATTAGE, ImVec2(250, 140.0f));
                            ImGui::PopStyleColor(1);
                            ImGui::PopID();

                            if (ImGui::Button("LIGHT", ImVec2(80 * main_scale, 50 * main_scale))) {
                                std::string append = std::format("{};\n", static_cast<int>(COMMAND_ID::TOGGLE_FRONT_LIGHT));
                                to_send_extra.append(append);
                            }

                        ImGui::EndGroup();
                    ImGui::EndGroup(); // Ends here

                    // VU METERS
                    // VU METERS
                    ImGui::SameLine();
                    ImGui::SetCursorPos(ImVec2(io.DisplaySize.x - 200.0f, 45.0f));
                    ImGui::BeginGroup();
                        ImGui::Dummy(ImVec2(0, 3));
                        ImVec2 groupStart = ImGui::GetCursorScreenPos(); // Top-left of the group
                        ImGui::BeginGroup();
                            addVUMeter(esp32.phase_current, 0.0f, 200.0f, "PhA", 0);
                            ImGui::SameLine();
                            ImGui::Dummy(ImVec2(2, 0));
                            ImGui::SameLine();
                            addVUMeter(esp32.temperature_motor, 30.0f, 130.0f, "T", 0);
                            ImGui::SameLine();
                            ImGui::Dummy(ImVec2(2, 0));
                            ImGui::SameLine();
                            addVUMeter(battery.percentage, 0.0f, 100.0f, "BAT", 1); //BAT
                        ImGui::EndGroup();
                        ImVec2 groupEnd = ImGui::GetItemRectMax();       // Bottom-right of the group
                        ImDrawList* drawList = ImGui::GetWindowDrawList();
                    ImGui::EndGroup();
                    groupStart.x -= 4;
                    groupStart.y -= 5;
                    groupEnd.x += 4;
                    groupEnd.y += 5;
                    drawList->AddRect(
                        groupStart,
                        groupEnd,
                        IM_COL32(90, 90, 90, 255), // Yellow color (RGBA)
                        2.0f,                       // Rounding
                        0,                          // Flags
                        2.0f                        // Thickness
                    );


                    // Wh/km
                    ImGui::SetCursorPos(ImVec2(io.DisplaySize.x / 5.2, io.DisplaySize.y / 1.7));
                    ImGui::BeginGroup();
                        movingAverages.wattage.moveAverage(battery.watts);
                        powerVerticalDiagonalHorizontal(movingAverages.wattage.output);
                        // ImGui::SameLine();
                        ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPosX() + 260.0, ImGui::GetCursorPosY() - 215.0));
                        ImGui::BeginGroup();
                            float whOverKmAveraged = 0.0;
                            if (esp32.speed_kmh > 0.5) {
                                whOverKmAveraged = movingAverages.whOverKm.moveAverage(battery.watts / esp32.speed_kmh);
                            } else {
                                whOverKmAveraged = 0.0;
                            }
                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                            ImGui::Text("Wh/km: %0.1f\nWh/km: %0.1f NOW", esp32.wh_over_km_average, whOverKmAveraged);
                            ImGui::PopFont();

                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.7);
                            if (esp32.speed_kmh >= 50.0) {
                                sprintf(text, "%0.1fkm/h >:(", esp32.speed_kmh);
                            } else {
                                sprintf(text, "%0.1fkm/h", esp32.speed_kmh);
                            }
                            ImGui::Text(text);
                            // TextCenteredOnLine(text, -0.5f, false);
                            ImGui::PopFont();

                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                                ImGui::Text("Range: %0.1f", esp32.range_left);
                                // movingAverages.acceleration.moveAverage(esp32.acceleration);
                                // ImGui::Text("Accel: %0.1f km/h/s", esp32.acceleration);
                                ImGui::Text("Motor RPM: %4.0f", esp32.motor_rpm);

                                {
                                    ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.25);

                                    ImGui::SetNextItemWidth(200.0);
                                    if (ImGui::Combo("##v", &POWER_PROFILE_CURRENT, "Legal\0Eco\0Balanced\0Performance\0")) {
                                        setPowerProfile(POWER_PROFILE_CURRENT);
                                        updateTableValue(SETTINGS_FILEPATH, "settings", "powerProfile", settings.powerProfile);
                                    }

                                    ImGui::SameLine();
                                    if (ImGui::Button("Reapply")) {
                                        setPowerProfile(POWER_PROFILE_CURRENT);
                                    }

                                    ImGui::PopFont();
                                }
                            ImGui::PopFont();
                        ImGui::EndGroup();


                    ImGui::EndGroup();


                    // ODOMETER / TRIP
                    ImGui::BeginGroup();

                        ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                        ImGui::SetCursorPosY(io.DisplaySize.y - 90.0f);
                            sprintf(text, "O: %0.0f", esp32.odometer_distance);
                            TextCenteredOnLine(text, 0.0f, false);
                            sprintf(text, "T:%5.2f", esp32.trip_distance);
                        ImGui::SetCursorPosY(io.DisplaySize.y - 90.0f);
                            TextCenteredOnLine(text, 1.0f, false);
                        ImGui::PopFont();

                    ImGui::EndGroup();

                    ImGui::SetCursorPos(ImVec2(0.0f, io.DisplaySize.y - 40.0f));
                    ImGui::Separator();
                    ImGui::Text("ESP32  Uptime: %2ldd %2ldh %2ldm %2lds\n", esp32.clockDaysSinceBoot, esp32.clockHoursSinceBoot, esp32.clockMinutesSinceBoot, esp32.clockSecondsSinceBoot);

                    if (!succesfulCommunication_avoidMutex) {
                        ImGui::EndDisabled();
                    }

                    ImGui::SameLine();
                    ImGui::SetCursorPos(ImVec2(io.DisplaySize.x - (float)(SerialP.succesfulCommunication ? 140.0f : 180.0f), io.DisplaySize.y - 35.0f));
                    ImVec4 color = SerialP.succesfulCommunication ? ImVec4(0.0f, 1.0f, 0.0f, 1.0f) // Green
                                                                  : ImVec4(1.0f, 0.0f, 0.0f, 1.0f); // Red
                    ImGui::TextColored(color, SerialP.succesfulCommunication ? "Connected" : "Disconnected");

                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Settings"))
                {
                    if (ImGui::BeginTabBar("TABBAR2", tab_bar_flags)) {
                        if (ImGui::BeginTabItem("App Menu"))
                        {
                            ImGui::BeginChild("Tab1Content", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                            ImGui::SeparatorText("ImGui");
                            ImGui::PopStyleColor();
                            ImGui::PopFont();

                            if(ImGui::Checkbox("Limit framerate", &settings.LIMIT_FRAMERATE)) {
                                updateTableValue(SETTINGS_FILEPATH, "settings", "limit_framerate", settings.LIMIT_FRAMERATE);
                            }

                            ImGui::SameLine();
                            ImGui::SetNextItemWidth(100);
                            const char* items[] = {"1", "5", "15", "30", "60", "90", "120", "240"};
                            static int item_current = findInArray_int(items, sizeof(items)/sizeof(items[0]), settings.TARGET_FPS);
                            if (ImGui::Combo("##v", &item_current, items, IM_ARRAYSIZE(items))) {
                                settings.TARGET_FPS = std::stof(items[item_current]);
                                updateTableValue(SETTINGS_FILEPATH, "settings", "framerate", settings.TARGET_FPS);
                            }

                            ImGui::Dummy(ImVec2(0, 20));
                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                            ImGui::SeparatorText("Serial");
                            ImGui::PopStyleColor();
                            ImGui::PopFont();

                            ImGui::Text("Serial write wait time ");
                            ImGui::SameLine();
                            ImGui::SetNextItemWidth(150.0f);
                            if (ImGui::InputInt("ms", &settings.serialWriteWaitMs, 1, 100)) {
                                updateTableValue(SETTINGS_FILEPATH, "settings", "serialWriteWaitMs", settings.serialWriteWaitMs);
                            }

                            if (ImGui::CollapsingHeader("SerialPort Buffer")) {
                                ImGui::Dummy(ImVec2(30, 0));
                                ImGui::SameLine();
                                ImGui::TextColored(ImVec4(1.0f, 0.89f, 0.32f, 1.0f), SerialP.receivedDataToRead.c_str());
                            }

                            ImGui::Text("Receive rate %0.1f ms / %0.1f Hz", SerialP.receiveRateMs, (1000.0 / SerialP.receiveRateMs));
                            ImGui::Text("Buffer: %d (bytes)", SerialP.bytesInBuffer);

                            ImGui::Dummy(ImVec2(0, 20));
                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                                ImGui::SeparatorText("System/App Statistics");
                                ImGui::PopStyleColor();
                            ImGui::PopFont();

                            ImGui::Dummy(ImVec2(0.0f, 20.0f));
                            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
                            ImGui::Text("Compiled on: %s @ %s\n", __DATE__, __TIME__);

                            ImGui::Dummy(ImVec2(0.0f, 20.0f));
                            ImGui::Text("CPU Usage (100%% is 1 core)");
                            ImGui::Text("       All:    %0.2f%%", cpuUsage_Everything.cpu_percent);
                            ImGui::Text("       ImGui:  %0.2f%%", cpuUsage_ImGui.cpu_percent);
                            ImGui::Text("       Serial: %0.2f%%", cpuUsage_SerialThread.cpu_percent);

                            ImGui::Dummy(ImVec2(0.0f, 20.0f));
                            ImGui::Text("Drawtime: %0.1fms", timeDrawDiff);
                            ImGui::Text("Rendertime: %0.1fms", timeRenderDiff);

                            ImGui::Dummy(ImVec2(0.0f, 20.0f));
                            ImGui::Text("Hostname: %s", hostname);
                            ImGui::Text("Settings filepath: %s", SETTINGS_FILEPATH);

                            ImGui::EndChild();
                            ImGui::EndTabItem();
                        }

                        if (ImGui::BeginTabItem("E-BIKE Menu"))
                        {
                            ImGui::BeginChild("Tab2Content", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);

                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                            ImGui::SeparatorText("ESP32 / EBIKE");
                            ImGui::PopStyleColor();
                            ImGui::PopFont();

                            ImGui::Text("Powered on: %s", esp32.power_on ? "True" : "False");
                            ImGui::Dummy(ImVec2(0.0f, 20.0f));

                            ImGui::Text("Firmware");
                            char text[50];
                            sprintf(text, "   Name: %s", esp32.fw_name.c_str());
                            ImGui::Text(text);

                            sprintf(text, "   Version: %s", esp32.fw_version.c_str());
                            ImGui::Text(text);

                            sprintf(text, "   Compile Time: %s", esp32.fw_compile_date_time.c_str());
                            ImGui::Text(text);

                            ImGui::Dummy(ImVec2(0.0f, 20.0f));
                            ImGui::Text("Core 0 loop exec time: %0.2f us", esp32.timeCore0_us);
                            ImGui::Text("Core 1 loop exec time: %0.2f us", esp32.timeCore1_us);

                            ImGui::Dummy(ImVec2(0, 20));

                            float buttonWidth = 260.0;
                            float buttonHeight = 80.0;

                            static char newOdometerValue[30];
                            ImGui::Text("Odometer = ");
                            ImGui::SameLine();
                            ImGui::SetNextItemWidth(100.0);
                            ImGui::InputText("km", newOdometerValue, sizeof(newOdometerValue));
                            // ImGui::SameLine();
                            if (ImGui::Button("Send", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                std::string append = std::format("{};{};\n", static_cast<int>(COMMAND_ID::SET_ODOMETER), newOdometerValue);
                                to_send_extra.append(append);
                            }

                            ImGui::Dummy(ImVec2(0, 20));
                            if (ImGui::Button("Save preferences", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                std::string append = std::format("{};\n", static_cast<int>(COMMAND_ID::SAVE_PREFERENCES));
                                to_send_extra.append(append);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Reset Trip", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                std::string append = std::format("{};\n", static_cast<int>(COMMAND_ID::RESET_TRIP));
                                to_send_extra.append(append);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Reset est. Range", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                std::string append = std::format("{};\n", static_cast<int>(COMMAND_ID::RESET_ESTIMATED_RANGE));
                                to_send_extra.append(append);
                            }

                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                            ImGui::SeparatorText("Battery");
                            ImGui::PopStyleColor();
                            ImGui::PopFont();

                            // TODO: amphours when new is hardcoded
                            ImGui::Text("State of Charge: %0.1f%%", battery.percentage);
                            ImGui::Text("Battery Health: %0.1f%%", (battery.ampHoursRated / 32.0) * 100.0);
                            ImGui::Dummy(ImVec2(0, 20));
                            ImGui::Text("Amphours when new: 32 Ah");
                            ImGui::Text("Amphours Rated: %0.2f Ah", battery.ampHoursRated);
                            ImGui::Text("Amphours Used: %0.2f Ah", battery.ampHoursUsed);
                            ImGui::Dummy(ImVec2(0, 20));
                            // Amphours used lifetime since 22.09.2025
                            ImGui::Text("Amphours Used (Lifetime): %0.2f Ah", battery.ampHoursUsedLifetime);

                            static char newAmphoursUsedLifetimeValue[30];
                            ImGui::Text("Set Amphours Used (Lifetime) value = ");
                            ImGui::SameLine();
                            ImGui::SetNextItemWidth(100.0);
                            ImGui::InputText("Ah", newAmphoursUsedLifetimeValue, sizeof(newAmphoursUsedLifetimeValue), ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
                            // ImGui::SameLine();
                            if (ImGui::Button("Send##xx", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                if (strlen(newAmphoursUsedLifetimeValue) > 0) {
                                    std::string append = std::format("{};{};\n", static_cast<int>(COMMAND_ID::SET_AMPHOURS_USED_LIFETIME), newAmphoursUsedLifetimeValue);
                                    to_send_extra.append(append);
                                }
                            }

                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                            ImGui::SeparatorText("VESC");
                            ImGui::PopStyleColor();
                            ImGui::PopFont();

                            ImGui::BeginGroup();
                                float ItemWidth = 150.0;
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Current Scaling (Braking)", &esp32_vesc_mcconf.l_current_min_scale);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Current Scaling (Accelerating)", &esp32_vesc_mcconf.l_current_max_scale);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Reverse RPM (times 3 && negative value)", &esp32_vesc_mcconf.l_min_erpm);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Forward RPM (times 3)", &esp32_vesc_mcconf.l_max_erpm);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Min Duty Cycle", &esp32_vesc_mcconf.l_min_duty);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Max Duty Cycle", &esp32_vesc_mcconf.l_max_duty);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Reverse Power (negative value)", &esp32_vesc_mcconf.l_watt_min);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Forward Power", &esp32_vesc_mcconf.l_watt_max);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Battery Braking Current (negative value)", &esp32_vesc_mcconf.l_in_current_min);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Battery Current", &esp32_vesc_mcconf.l_in_current_max);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Profile name = %s", esp32_vesc_mcconf.name.c_str());

                                // if (ImGui::Button("Get values", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                //     std::string append = std::format("{};\n", static_cast<int>(COMMAND_ID::GET_VESC_MCCONF));
                                //     to_send_extra.append(append);
                                // }
                                // ImGui::SameLine();
                                // if (ImGui::Button("Set values", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                //     setMcconfValues(esp32_vesc_mcconf);
                                // }
                            ImGui::EndGroup();

                            ImGui::EndChild();
                            ImGui::EndTabItem();
                        }

                        if (ImGui::BeginTabItem("Serial Log"))
                        {
                            std::string SerialLog;
                            SerialLog = SerialP.log;

                            ImGui::InputTextMultiline("##", (char*)SerialLog.c_str(), sizeof(SerialLog.c_str()), ImGui::GetContentRegionAvail());
                            ImGui::EndTabItem();
                        }
                        ImGui::EndTabBar(); //TABBAR2
                    }
                    ImGui::EndTabItem(); // if (ImGui::BeginTabItem("Settings"))
                }
                ImGui::EndTabBar(); //TABBAR1
            }

            ImGui::EndGroup();

            ImGui::End();
            timeDrawEnd = std::chrono::steady_clock::now();
            timeDrawDiff = std::chrono::duration<double, std::milli>(timeDrawEnd - timeDrawStart).count();
        }

        // Rendering
        timeRenderStart = std::chrono::steady_clock::now();
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Update and Render additional Platform Windows
        // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
        //  For this specific demo app we could also call SDL_GL_MakeCurrent(window, gl_context) directly)
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            SDL_Window* backup_current_window = SDL_GL_GetCurrentWindow();
            SDL_GLContext backup_current_context = SDL_GL_GetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            SDL_GL_MakeCurrent(backup_current_window, backup_current_context);
        }

        SDL_GL_SwapWindow(window);

        timeRenderEnd = std::chrono::steady_clock::now();
        timeRenderDiff = std::chrono::duration<double, std::milli>(timeRenderEnd - timeRenderStart).count();

        // limit framerate
        static double lasttime = (float)(SDL_GetTicks() / 1000.0f);;
        if (settings.LIMIT_FRAMERATE) {
            while ((float)(SDL_GetTicks() / 1000.0f) < lasttime + 1.0/settings.TARGET_FPS) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            lasttime += 1.0/settings.TARGET_FPS;
        }

        cpuUsage_ImGui.measureEnd(1);
        cpuUsage_Everything.measureEnd(0);
    }

    // Cleanup
    // [If using SDL_MAIN_USE_CALLBACKS: all code below would likely be your SDL_AppQuit() function]
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DestroyContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    backend.join();

    return 0;
}
