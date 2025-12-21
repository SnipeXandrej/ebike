// Dear ImGui: standalone application for SDL3 + OpenGL
// (SDL is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/ folder).
// - Introduction, links and more at the top of imgui.cpp

#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_opengl3.h"
#include <format>
#include <stdio.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>

#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include "toml.hpp"
#include <print>

#include "client.hpp"
#include "other.hpp"
#include "cpuUsage.hpp"
#include "../comm.h"
#include "arc_progress_bar.hpp"
#include "../timer.hpp"
#include "imguiGestures.hpp"

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
    std::string name;
    int id;
    // int motor_poles;
    // float gear_ratio;
    // float wheel_diameter;
};
VESC_MCCONF mcconf_vesc;

struct trip {
    double distance; // in km
    double wattHoursUsed;
    double wattHoursConsumed;
    double wattHoursRegenerated;
};

struct estRange {
    float range;
    float distance;
    float WhPerKm;
};

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
    float phase_current;
    float duty_cycle;
    float temperature_motor;
    float temperature_vesc;
    float timeCore0_us;
    float timeCore1_us;
    float acceleration;
    bool power_on = false;
    std::string log;
    bool regenerativeBraking;
    int currentPowerProfile;

    std::string fw_name;
    std::string fw_version;
    std::string fw_compile_date_time;

    trip trip_A;
    trip trip_B;
    estRange estimatedRange;
} backend;

// Limit FPS
struct {
    float TARGET_FPS;
    bool LIMIT_FRAMERATE;
    int ipcWriteWaitMs;
    int powerProfile;
    bool showMotorRPM;
    bool showAcceleration;
    bool showTripA;
    bool showMotorDutyInsteadOfMotorTemp;
    bool launchFullscreen;
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
    float watthoursFullyDischarged;
    float percentage;
    float ampHoursFullyCharged;
    float ampHoursFullyChargedWhenNew;
    float amphours_min_voltage;
    float amphours_max_voltage;
    bool charging = 0;

    float nominalVoltage;
} battery;

struct {
    MovingAverage wattage;
    MovingAverage wattageMoreSmooth;
    MovingAverage whOverKm;
} movingAverages;

struct {
    double analog0;
    double analog1;
    double analog2;
    double analog3;
    double analog4;
    double analog5;
    double analog6;
    double analog7;
} analogReadings;

bool done = false;

char currentTimeAndDate[100];

// TODO: do not hardcode filepaths :trol:
const char* SETTINGS_FILEPATH = "/home/snipex/.config/ebikegui/settings.toml";
char hostname[1024];
char *desktopEnvironment;
std::string serverAddress;

struct {
    Timer draw;
    Timer render;
    Timer ping;
} timer;

std::string availablePowerProfiles;
bool successfulCommunication = false;

std::string to_send;
std::string to_send_extra;

struct {
    CPUUsage ImGui;
    CPUUsage ipcThread;
    CPUUsage ipcThreadRead;
    CPUUsage Everything;
} cpuUsage;

ClientSocket IPC;

ArcProgressBar ArcBar_WhKmNow;
ArcProgressBar ArcBar_phaseCurrent;
ArcProgressBar ArcBar_motorTemp;
ArcProgressBar ArcBar_motorDutyCycle;

void setBrightnessLow() {
    // std::system("brightnessctl set 0%");
    if (strcmp(desktopEnvironment, "KDE") == 0) {
        std::system("kscreen-doctor --dpms off");
    } else {
        std::system("wlr-randr --output HDMI-A-1 --off");
    }
}

void setBrightnessHigh() {
    // std::system("brightnessctl set 100%");
    if (strcmp(desktopEnvironment, "KDE") == 0) {
        std::system("kscreen-doctor --dpms on");
    } else {
        std::system("wlr-randr --output HDMI-A-1 --on");
    }
}

void setMcconfCustomValues(VESC_MCCONF mcconf) {
    std::string append = std::format("{};{};{};{};{};{};{};{};{};{};{};\n"
                                        ,static_cast<int>(COMMAND_ID::SET_POWER_PROFILE_CUSTOM)
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
    );
    to_send_extra.append(append);
}

void setPowerProfile(int PROFILE) {
    std::string append = std::format("{};{};\n"
                                        ,static_cast<int>(COMMAND_ID::SET_POWER_PROFILE)
                                        ,PROFILE
    );
    to_send_extra.append(append);
}

void writeClock() {
    time_t currentTime;
    struct tm *localTime;

    time( &currentTime );
    localTime = localtime( &currentTime );

    char text[100];
    sprintf(text, "%02d:%02d:%02d  %02d.%02d.%d", localTime->tm_hour, localTime->tm_min, localTime->tm_sec, localTime->tm_mday, localTime->tm_mon+1, localTime->tm_year+1900);
    std::strcpy(currentTimeAndDate, text);
}

void processRead(std::string line) {
        if (!line.empty()) {
            // std::print("\n{}\n", line);

            auto readStringPacket = split(line, '\n');

            if (!readStringPacket.empty())
            for (int i = 0; i < (int)readStringPacket.size(); i++) {
                auto packet = split(readStringPacket[i], ';');
                if (!packet.empty()) {

                int index = 1;
                int command_id = 0;

                try {
                    command_id = std::stoi(packet[0]);
                } catch(...) {
                    std::println("Failed to convert command_id stoi()");
                    command_id = -1;
                }

                if (command_id == COMMAND_ID::ARE_YOU_ALIVE) {
                        std::cout << "[IPC] Successful communication with Atmega8!" << "\n";
                        successfulCommunication = true;
                }

                if (successfulCommunication) {
                    switch (command_id) {
                        case COMMAND_ID::GET_BATTERY:
                            battery.voltage = getValueFromPacket(packet, &index);
                            battery.current = getValueFromPacket(packet, &index);
                            battery.watts = getValueFromPacket(packet, &index);
                            battery.wattHoursUsed = getValueFromPacket(packet, &index);
                            battery.watthoursFullyDischarged = getValueFromPacket(packet, &index);
                            battery.ampHoursUsed = getValueFromPacket(packet, &index);
                            battery.ampHoursUsedLifetime = getValueFromPacket(packet, &index);
                            battery.ampHoursFullyCharged = getValueFromPacket(packet, &index);
                            battery.ampHoursFullyChargedWhenNew = getValueFromPacket(packet, &index);
                            battery.percentage = getValueFromPacket(packet, &index);
                            battery.voltage_min = getValueFromPacket(packet, &index);
                            battery.voltage_max = getValueFromPacket(packet, &index);
                            battery.nominalVoltage = getValueFromPacket(packet, &index);
                            battery.amphours_min_voltage = getValueFromPacket(packet, &index);
                            battery.amphours_max_voltage = getValueFromPacket(packet, &index);
                            battery.charging = getValueFromPacket(packet, &index);
                            break;

                        case COMMAND_ID::GET_STATS:
                            backend.speed_kmh = getValueFromPacket(packet, &index);
                            backend.motor_rpm = getValueFromPacket(packet, &index);
                            backend.odometer_distance = getValueFromPacket(packet, &index);
                            backend.trip_A.distance = getValueFromPacket(packet, &index);
                            backend.trip_A.wattHoursUsed = getValueFromPacket(packet, &index);
                            backend.trip_A.wattHoursConsumed = getValueFromPacket(packet, &index);
                            backend.trip_A.wattHoursRegenerated = getValueFromPacket(packet, &index);
                            backend.trip_B.distance = getValueFromPacket(packet, &index);
                            backend.trip_B.wattHoursUsed = getValueFromPacket(packet, &index);
                            backend.trip_B.wattHoursConsumed = getValueFromPacket(packet, &index);
                            backend.trip_B.wattHoursRegenerated = getValueFromPacket(packet, &index);
                            backend.phase_current = getValueFromPacket(packet, &index);
                            backend.duty_cycle = getValueFromPacket(packet, &index);
                            backend.estimatedRange.WhPerKm = getValueFromPacket(packet, &index);
                            backend.estimatedRange.distance = getValueFromPacket(packet, &index);
                            backend.estimatedRange.range = getValueFromPacket(packet, &index);
                            backend.temperature_motor = getValueFromPacket(packet, &index);
                            backend.temperature_vesc = getValueFromPacket(packet, &index);
                            backend.totalSecondsSinceBoot = getValueFromPacket(packet, &index);
                            backend.timeCore0_us = getValueFromPacket(packet, &index);
                            backend.timeCore1_us = getValueFromPacket(packet, &index);
                            backend.acceleration = getValueFromPacket(packet, &index);
                            backend.power_on = (bool)getValueFromPacket(packet, &index);
                            backend.regenerativeBraking = (bool)getValueFromPacket(packet, &index);
                            backend.currentPowerProfile = (int)getValueFromPacket(packet, &index);

                            backend.clockSecondsSinceBoot = (uint64_t)(backend.totalSecondsSinceBoot) % 60;
                            backend.clockMinutesSinceBoot = (uint64_t)(backend.totalSecondsSinceBoot / 60.0) % 60;
                            backend.clockHoursSinceBoot   = (uint64_t)(backend.totalSecondsSinceBoot / 60.0 / 60.0) % 24;
                            backend.clockDaysSinceBoot    = backend.totalSecondsSinceBoot / 60.0 / 60.0 / 24;

                            break;

                        case COMMAND_ID::GET_FW:
                            backend.fw_name = getValueFromPacket_string(packet, &index);
                            backend.fw_version = getValueFromPacket_string(packet, &index);
                            backend.fw_compile_date_time = getValueFromPacket_string(packet, &index);
                            break;

                        case COMMAND_ID::GET_VESC_MCCONF:
                            mcconf_vesc.l_current_min_scale = getValueFromPacket(packet, &index);
                            mcconf_vesc.l_current_max_scale = getValueFromPacket(packet, &index);
                            mcconf_vesc.l_min_erpm = getValueFromPacket(packet, &index);
                            mcconf_vesc.l_max_erpm = getValueFromPacket(packet, &index);
                            mcconf_vesc.l_min_duty = getValueFromPacket(packet, &index);
                            mcconf_vesc.l_max_duty = getValueFromPacket(packet, &index);
                            mcconf_vesc.l_watt_min = getValueFromPacket(packet, &index);
                            mcconf_vesc.l_watt_max = getValueFromPacket(packet, &index);
                            mcconf_vesc.l_in_current_min = getValueFromPacket(packet, &index);
                            mcconf_vesc.l_in_current_max = getValueFromPacket(packet, &index);
                            mcconf_vesc.name = getValueFromPacket_string(packet, &index);
                            break;

                        case COMMAND_ID::GET_ANALOG_READINGS:
                            analogReadings.analog0 = getValueFromPacket_double(packet, &index);
                            analogReadings.analog1 = getValueFromPacket_double(packet, &index);
                            analogReadings.analog2 = getValueFromPacket_double(packet, &index);
                            analogReadings.analog3 = getValueFromPacket_double(packet, &index);
                            analogReadings.analog4 = getValueFromPacket_double(packet, &index);
                            analogReadings.analog5 = getValueFromPacket_double(packet, &index);
                            analogReadings.analog6 = getValueFromPacket_double(packet, &index);
                            analogReadings.analog7 = getValueFromPacket_double(packet, &index);
                            break;

                        case COMMAND_ID::BACKEND_LOG:
                            backend.log.append(std::format("[{}] {}\n", currentTimeAndDate, getValueFromPacket_string(packet, &index)));
                            break;

                        case COMMAND_ID::GET_AVAILABLE_POWER_PROFILES:
                            availablePowerProfiles = getValueFromPacket_string(packet, &index);

                            break;

                        default:
                            // std::cout << "Received: " << line << "\n";
                            break;
                    }
                }
            }
            } // !packet.empty()
        }
}

// Main code
int main(int argc, char** argv)
{
    setenv("SDL_VIDEODRIVER", "wayland", 1);

    // ##########################
    // ##### Hostname stuff #####
    // ##########################
    gethostname(hostname, sizeof(hostname));
    printf("Hostname = %s\n", hostname);

    if (argc == 2) {
        serverAddress = argv[1];
    } else {
        serverAddress = "0.0.0.0";
    }
    std::print("Server address: {}\n", serverAddress);

    if (getenv("XDG_CURRENT_DESKTOP") == NULL) {
        desktopEnvironment = (char*)"unknown";
    } else {
        desktopEnvironment = getenv("XDG_CURRENT_DESKTOP");
    }
    printf("Desktop Environment = %s\n", desktopEnvironment);

    writeClock();

    // ####################
    // ##### Settings #####
    // ####################

    movingAverages.wattageMoreSmooth.smoothingFactor = 0.1f;
    movingAverages.whOverKm.smoothingFactor = 0.05f;

    // ########################
    // ######### TOML #########
    // ########################

    // TODO: if settings.toml doesnt exist, create it
    toml::table tbl;
    tbl = toml::parse_file(SETTINGS_FILEPATH);

    // values
    settings.TARGET_FPS         = tbl["settings"]["framerate"].value_or(60);
    settings.LIMIT_FRAMERATE    = tbl["settings"]["limit_framerate"].value_or(0);
    settings.ipcWriteWaitMs     = tbl["settings"]["ipcWriteWaitMs"].value_or(50);
    settings.showMotorRPM       = tbl["settings"]["showMotorRPM"].value_or(1);
    settings.showAcceleration   = tbl["settings"]["showAcceleration"].value_or(1);
    settings.showTripA          = tbl["settings"]["showTripA"].value_or(1);
    settings.showMotorDutyInsteadOfMotorTemp = tbl["settings"]["showMotorDutyInsteadOfMotorTemp"].value_or(0);
    settings.launchFullscreen   = tbl["settings"]["launchFullscreen"].value_or(0);

    ArcBar_WhKmNow.init(120.0, 180.0, 20.0, 0.0, 60.0, "Wh/km");
    ArcBar_phaseCurrent.init(120.0, 180.0, 20.0, 0.0, 250.0, "Phase");
    ArcBar_motorTemp.init(120.0, 180.0, 20.0, 25.0, 120.0, "Temp");
    ArcBar_motorDutyCycle.init(120.0, 180.0, 20.0, 0.0, 100.0, "Duty");

    // ################
    // ##### IPC ######
    // ################

    static std::chrono::duration<double, std::milli> msElapsedWrite;
    static std::chrono::duration<double, std::milli> msElapsedRead;
    std::thread commThread([&]() -> int {
        std::cout << "[IPC] Initializing" << "\n";
        if (IPC.createClientSocket(8080, serverAddress.c_str()) != 0) {
            std::printf("[IPC] Failed to initialize\n");
        }

        std::jthread commThreadRead([&] {
            while(!done) {
                cpuUsage.ipcThreadRead.measureStart(1);
                auto t1 = std::chrono::high_resolution_clock::now();
                std::string readFromIPC = IPC.read();

                processRead(readFromIPC);
                msElapsedRead = std::chrono::high_resolution_clock::now() - t1;
                cpuUsage.ipcThreadRead.measureEnd(1);
            }
        });

        std::cout << "[IPC] Entering main while loop\n";
        while(!done) {
            cpuUsage.ipcThread.measureStart(1);

            to_send = "";
            static bool sendOnce = false;

            if (!successfulCommunication) {
                sendOnce = false;

                commAddValue(&to_send, COMMAND_ID::ARE_YOU_ALIVE, 0);
                to_send.append("\n");

                IPC.write(to_send.data(), to_send.size());

                // hol'up
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (successfulCommunication) {
                auto t1 = std::chrono::high_resolution_clock::now();

                timer.ping.end();
                if (timer.ping.getTime_ms() >= 750.0) {
                    timer.ping.start();


                    commAddValue(&to_send, COMMAND_ID::PING, 0);
                    to_send.append("\n");

                    if (sendOnce == false) {
                        sendOnce = true;

                        commAddValue(&to_send, COMMAND_ID::GET_AVAILABLE_POWER_PROFILES, 0);
                        to_send.append("\n");

                        commAddValue(&to_send, COMMAND_ID::GET_VESC_MCCONF, 0);
                        to_send.append("\n");

                        commAddValue(&to_send, COMMAND_ID::GET_FW, 0);
                        to_send.append("\n");
                    }
                }

                commAddValue(&to_send, COMMAND_ID::GET_BATTERY, 0);
                to_send.append("\n");
                commAddValue(&to_send, COMMAND_ID::GET_STATS, 0);
                to_send.append("\n");
                commAddValue(&to_send, COMMAND_ID::GET_ANALOG_READINGS, 0);
                to_send.append("\n");

                to_send.append(to_send_extra);
                to_send_extra = "";

                IPC.write(to_send.data(), to_send.size());
                if (backend.power_on) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(settings.ipcWriteWaitMs));
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                msElapsedWrite = std::chrono::high_resolution_clock::now() - t1;
            }

            cpuUsage.ipcThread.measureEnd(1);
        }

        commThreadRead.join();
        return 0;
    });


    std::string titleBarName = std::format("E-BIKE GUI (Connected to: {})", serverAddress);

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
    // SDL_Window* window = SDL_CreateWindow("E-BIKE GUI", (int)(1257 * main_scale), (int)(583 * main_scale), window_flags);
    SDL_Window* window = SDL_CreateWindow(titleBarName.c_str(), (int)(800 * main_scale), (int)(480 * main_scale), window_flags);
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

    SDL_SetWindowMinimumSize(window, 800, 480);

    if (settings.launchFullscreen) {
        SDL_SetWindowFullscreen(window, true);
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

    io.Fonts->AddFontFromFileTTF("ProggyVector-Regular.ttf", 13.0);

    // Setup scaling
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale);        // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing this requires resetting Style + calling this again)
    style.FontScaleDpi = main_scale;        // Set initial font scale. (using io.ConfigDpiScaleFonts=true makes this unnecessary. We leave both here for documentation purpose)

    // Setup Platform/Renderer backends
    ImGui_ImplSDL3_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // style.FontScaleDpi = 2.0f;
    style.FontScaleDpi = 2.0f;

    // Main loop
    while (!done)
    {
        cpuUsage.ImGui.measureStart(1);
        cpuUsage.Everything.measureStart(0);

        static bool power_on_old = false;
        if (backend.power_on && power_on_old != backend.power_on) {
            power_on_old = backend.power_on;

            // Run this when the bike gets powered on
            std::thread(setBrightnessHigh).detach();
        } else if (!backend.power_on && power_on_old != backend.power_on) {
            power_on_old = backend.power_on;

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

        // Clock
        writeClock();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        // ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->Pos);
        ImGui::SetNextWindowSize(io.DisplaySize);

        if (true) {
            timer.draw.start();
            ImGui::Begin("Main", nullptr, ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoDecoration|ImGuiWindowFlags_NoBringToFrontOnFocus|ImGuiWindowFlags_NoScrollWithMouse); // Create a window called "Main" and append into it. //ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoMove

            ImGui::BeginGroup();

            ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_FittingPolicyScroll;
            if (ImGui::BeginTabBar("Tabbar", tab_bar_flags)) {
                if (ImGui::BeginTabItem("Main"))
                {

                    {
                        static ImGuiGesture gesture;

                        gesture.start();
                            if (ImGui::Begin("Gesture test", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize)) {
                                ImGui::Button("Test");

                                if (ImGui::Button("Close")) {
                                    gesture.closeGesture();
                                }
                            }
                            ImGui::End();
                        gesture.end();
                    }

                    {
                        ImVec2 textSize = ImGui::CalcTextSize(currentTimeAndDate);
                        ImGui::SetCursorPos(ImVec2((io.DisplaySize.x / 2.0) - (textSize.x / 2.0), 7.0));
                        ImGui::Text(currentTimeAndDate);
                    }

                    {
                        // Bike Battery
                        char text[100];
                        sprintf(text, "SOC: %0.1f", battery.percentage);
                        ImVec2 textSize = ImGui::CalcTextSize(text);
                        ImGui::SetCursorPos(ImVec2((io.DisplaySize.x - textSize.x) - 20.0, 7.0));
                        ImVec4 color = battery.charging ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 1.0, 1.0, 1.0);
                        ImGui::TextColored(color, text);

                        if (ImGui::IsItemClicked())
                            to_send_extra.append(std::format("{};\n", static_cast<int>(COMMAND_ID::TOGGLE_CHARGING_STATE)));

                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Click to toggle charging state\nCharging: %s", battery.charging ? "true" : "false");
                    }

                    ImGui::BeginGroup(); // Starts here
                        ImGui::BeginGroup();
                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.68);

                            ImGui::Text("Power info");
                            movingAverages.wattageMoreSmooth.moveAverage(battery.watts);

                            ImGui::TextColored(ImVec4(0.0, 1.0, 0.0, 1.0), "%7.2f V", battery.voltage);
                            ImGui::TextColored(ImVec4(1.0, 0.39, 0.196, 1.0), "%7.2f A", battery.current);
                            ImGui::TextColored(ImVec4(1.0, 1.0, 0.0, 1.0), "%7.2f W", battery.watts);

                            ImGui::Dummy(ImVec2(0.0, 40.0));
                                ImGui::TextColored(backend.regenerativeBraking ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 0.0, 0.0, 0.35),"REGEN");
                            ImGui::PopFont();
                            if (ImGui::IsItemClicked())
                                to_send_extra.append(std::format("{};\n", static_cast<int>(COMMAND_ID::TOGGLE_REGEN_BRAKING)));

                            if (ImGui::IsItemHovered())
                                ImGui::SetTooltip("Click to toggle regenerative braking\nRegen: %s", backend.regenerativeBraking ? "on" : "off");

                        ImGui::EndGroup();
                        //
                        ImGui::SameLine();
                        ImGui::Dummy(ImVec2(20,0));
                        // ImGui::SameLine();
                        //
                        ImGui::BeginGroup();

                            // if (ImGui::Button("LIGHT", ImVec2(80 * main_scale, 50 * main_scale))) {
                            //     std::string append = std::format("{};\n", static_cast<int>(COMMAND_ID::TOGGLE_FRONT_LIGHT));
                            //     to_send_extra.append(append);
                            // }

                        ImGui::EndGroup();
                    ImGui::EndGroup(); // Ends here

                    // METERS
                    // METERS
                    ImGui::SameLine();
                    ImGui::BeginGroup();
                        double whkmnow = battery.watts / backend.speed_kmh;
                        if (std::isnan(whkmnow) || whkmnow > 999.0 || whkmnow < -999.0) {
                            whkmnow = 0;
                        }
                        movingAverages.whOverKm.moveAverage((float)whkmnow);

                        ImGui::SetCursorPos(ImVec2(io.DisplaySize.x - 150.0f, io.DisplaySize.y - 0.0f - 150.0 - 130.0 - 130.0));
                        ArcBar_WhKmNow.ProgressBarArc(movingAverages.whOverKm.output);

                        ImGui::SetCursorPos(ImVec2(io.DisplaySize.x - 150.0f, io.DisplaySize.y - 0.0f - 150.0 - 130.0));
                        ArcBar_phaseCurrent.ProgressBarArc(backend.phase_current);

                        ImGui::SetCursorPos(ImVec2(io.DisplaySize.x - 150.0f, io.DisplaySize.y - 0.0f - 150.0));
                        if (settings.showMotorDutyInsteadOfMotorTemp) {
                            ArcBar_motorDutyCycle.ProgressBarArc(backend.duty_cycle);
                        } else {
                            ArcBar_motorTemp.ProgressBarArc(backend.temperature_motor);
                        }

                        if (ImGui::IsItemClicked()) {
                            settings.showMotorDutyInsteadOfMotorTemp = !settings.showMotorDutyInsteadOfMotorTemp;
                            updateTableValue(SETTINGS_FILEPATH, "settings", "showMotorDutyInsteadOfMotorTemp", settings.showMotorDutyInsteadOfMotorTemp);
                        }
                    ImGui::EndGroup();


                    // Wh/km
                    ImGui::SetCursorPosX(200);
                    ImGui::SetCursorPosY(75);
                    ImGui::BeginGroup();
                        int numOfBars = 100;
                        float maxWatts = 9000;
                        float indicateEveryWatts = 1000;
                        powerWidget(numOfBars, maxWatts, indicateEveryWatts, battery.watts);

                        ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPosX() + 100.0, ImGui::GetCursorPosY() - 35.0));
                        ImGui::BeginGroup();
                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.8);
                                if (settings.showTripA) {
                                    double whkm = backend.trip_A.wattHoursUsed / backend.trip_A.distance;
                                    whkm != whkm ? whkm = -1 : whkm = whkm;
                                    ImGui::Text("Wh/km: %0.1f¹", whkm);
                                } else {
                                    double whkm = backend.trip_B.wattHoursUsed / backend.trip_B.distance;
                                    whkm != whkm ? whkm = -1 : whkm = whkm;
                                    ImGui::Text("Wh/km: %0.1f²", whkm);
                                }

                                if (ImGui::IsItemClicked()) {
                                    settings.showTripA = !settings.showTripA;
                                    updateTableValue(SETTINGS_FILEPATH, "settings", "showTripA", settings.showTripA);
                                }
                            ImGui::PopFont();

                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("¹Watthour/km from Trip A\n²Watthour/km from Trip B\n");
                            }

                            {
                                char text[128];
                                ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 2.0);
                                if (backend.speed_kmh >= 50.0) {
                                    sprintf(text, "%0.1f >:(", backend.speed_kmh);
                                } else {
                                    sprintf(text, "%0.1f", backend.speed_kmh);
                                }
                                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 15.0);
                                ImGui::Text(text);
                                ImGui::SameLine();
                                ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 33.0);
                                ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.2);
                                ImGui::Text("km/h");
                                ImGui::PopFont();
                                // TextCenteredOnLine(text, -0.5f, false);
                                ImGui::PopFont();
                            }

                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.8);
                                ImGui::Text("Range: %0.1f", backend.estimatedRange.range);
                                if (settings.showAcceleration) {
                                    ImGui::Text("Accel: %0.1f", backend.acceleration);

                                    if (ImGui::IsItemHovered()) {
                                        ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.3);
                                        ImGui::SetTooltip("measured in km/h per second");
                                        ImGui::PopFont();
                                    }
                                }

                                if (settings.showMotorRPM)
                                    ImGui::Text("Motor RPM: %4.0f", backend.motor_rpm);

                            ImGui::PopFont();
                        ImGui::EndGroup();


                    ImGui::EndGroup();


                    // ODOMETER / TRIP
                    {
                        char text[128];
                        ImGui::BeginGroup();
                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                            ImGui::SetCursorPosY(io.DisplaySize.y - 55.0f);
                                ImGui::Separator();
                                sprintf(text, "O: %0.0f", backend.odometer_distance);
                                TextCenteredOnLine(text, 0.0f, false);
                                if (settings.showTripA) {
                                    sprintf(text, "T: %4.1f¹", backend.trip_A.distance);
                                } else {
                                    sprintf(text, "T: %4.1f²", backend.trip_B.distance);
                                }
                            ImGui::SetCursorPosY(io.DisplaySize.y - 52.0f);
                                TextCenteredOnLine(text, 1.0f, false);
                                if (ImGui::IsItemClicked()) {
                                    settings.showTripA = !settings.showTripA;
                                    updateTableValue(SETTINGS_FILEPATH, "settings", "showTripA", settings.showTripA);
                                }
                            ImGui::PopFont();

                            {
                                // ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.25);

                                ImVec2 cursorPos = ImGui::GetContentRegionAvail();
                                cursorPos.x = (cursorPos.x / 2.0) - 115;

                                ImGui::SetCursorPos(ImVec2(cursorPos.x, io.DisplaySize.y - 44.0f));

                                ImGui::SetNextItemWidth(230.0);
                                if (ImGui::Combo("##v", &backend.currentPowerProfile, availablePowerProfiles.data())) {
                                    setPowerProfile(backend.currentPowerProfile);
                                    to_send_extra.append(std::format("{};\n", static_cast<int>(COMMAND_ID::GET_VESC_MCCONF)));
                                }

                                // ImGui::PopFont();
                            }

                        ImGui::EndGroup();
                    }

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

                            if(ImGui::Button("SHUTDOWN")) {
                                std::system("sudo /sbin/shutdown -h now");
                            }
                            ImGui::SameLine();
                            if(ImGui::Button("REBOOT")) {
                                std::system("sudo /sbin/shutdown -r now");
                            }
                            ImGui::SameLine();
                            if(ImGui::Button("QUIT")) {
                                done = 1;
                            }

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

                            if (ImGui::Checkbox("Show acceleration", &settings.showAcceleration)) {
                                updateTableValue(SETTINGS_FILEPATH, "settings", "showAcceleration", settings.showAcceleration);
                            }

                            if (ImGui::Checkbox("Show motor RPM", &settings.showMotorRPM)) {
                                updateTableValue(SETTINGS_FILEPATH, "settings", "showMotorRPM", settings.showMotorRPM);
                            }

                            if (ImGui::Checkbox("Show trip A", &settings.showTripA)) {
                                updateTableValue(SETTINGS_FILEPATH, "settings", "showTripA", settings.showTripA);
                            }

                            if (ImGui::Checkbox("Show motor duty instead of motor temp", &settings.showMotorDutyInsteadOfMotorTemp)) {
                                updateTableValue(SETTINGS_FILEPATH, "settings", "showMotorDutyInsteadOfMotorTemp", settings.showMotorDutyInsteadOfMotorTemp);
                            }

                            if (ImGui::Checkbox("Launch fullscreen", &settings.launchFullscreen)) {
                                updateTableValue(SETTINGS_FILEPATH, "settings", "launchFullscreen", settings.launchFullscreen);
                            }


                            ImGui::Dummy(ImVec2(0, 20));
                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                            ImGui::SeparatorText("IPC");
                            ImGui::PopStyleColor();
                            ImGui::PopFont();

                            ImGui::Text("Status: %s", successfulCommunication ? "connected" : "disconnected");
                            ImGui::Text("Requests per second: %03.1f Hz (%03.1f ms)", (1000.0 / msElapsedWrite.count()), msElapsedWrite.count());
                            ImGui::Text("Reads per second:    %03.1f Hz (%03.1f ms)", (1000.0 / msElapsedRead.count()), msElapsedRead.count());
                            // if (ImGui::Button("Reconnect")) {
                            //     IPC.begin();
                            // }

                            ImGui::Text("Write wait time ");
                            ImGui::SameLine();
                            ImGui::SetNextItemWidth(150.0f);
                            if (ImGui::InputInt("ms", &settings.ipcWriteWaitMs, 1, 100)) {
                                updateTableValue(SETTINGS_FILEPATH, "settings", "ipcWriteWaitMs", settings.ipcWriteWaitMs);
                            }

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
                            ImGui::Text("       All:       %0.2f%%", cpuUsage.Everything.cpu_percent);
                            ImGui::Text("       ImGui:     %0.2f%%", cpuUsage.ImGui.cpu_percent);
                            ImGui::Text("       IPC Read:  %0.2f%%", cpuUsage.ipcThreadRead.cpu_percent);
                            ImGui::Text("       IPC Write: %0.2f%%", cpuUsage.ipcThread.cpu_percent);

                            ImGui::Dummy(ImVec2(0.0f, 20.0f));
                            ImGui::Text("Drawtime: %0.1fms", timer.draw.getTime_ms());
                            ImGui::Text("Rendertime: %0.1fms", timer.render.getTime_ms());

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
                            ImGui::SeparatorText("BACKEND / EBIKE");
                            ImGui::PopStyleColor();
                            ImGui::PopFont();

                            ImGui::Text("Powered on: %s", backend.power_on ? "True" : "False");
                            ImGui::Dummy(ImVec2(0.0f, 20.0f));

                            ImGui::Text("Firmware");
                            char text[50];
                            sprintf(text, "   Name: %s", backend.fw_name.c_str());
                            ImGui::Text(text);

                            sprintf(text, "   Version: %s", backend.fw_version.c_str());
                            ImGui::Text(text);

                            sprintf(text, "   Compile Time: %s", backend.fw_compile_date_time.c_str());
                            ImGui::Text(text);

                            ImGui::Text("   Uptime: %2ldd %2ldh %2ldm %2lds\n", backend.clockDaysSinceBoot, backend.clockHoursSinceBoot, backend.clockMinutesSinceBoot, backend.clockSecondsSinceBoot);

                            ImGui::Dummy(ImVec2(0.0f, 20.0f));
                            ImGui::Text("main while loop: %0.0f us / %0.0f Hz", backend.timeCore0_us, 1000000 / backend.timeCore0_us);
                            // ImGui::Text("Core 1 loop exec time: %0.0f us", backend.timeCore1_us);

                            float buttonWidth = 170.0;
                            float buttonHeight = 80.0;

                            ImGui::Dummy(ImVec2(0, 20));
                            if (ImGui::Button("Save\npreferences", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                std::string append = std::format("{};\n", static_cast<int>(COMMAND_ID::SAVE_PREFERENCES));
                                to_send_extra.append(append);
                            }

                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                            ImGui::SeparatorText("Trip/Range/Odometer");
                            ImGui::PopStyleColor();
                            ImGui::PopFont();

                            ImGui::Text("Trip A\n"
                                        "   Distance:       %0.3f km\n"
                                        "   Wh used:        %0.3f\n"
                                        "   Wh Consumed:    %0.3f\n"
                                        "   Wh Regenerated: %0.3f\n\n"
                                        , backend.trip_A.distance
                                        , backend.trip_A.wattHoursUsed
                                        , backend.trip_A.wattHoursConsumed
                                        , backend.trip_A.wattHoursRegenerated
                                       );

                            ImGui::Text("Trip B\n"
                                        "   Distance:       %0.3f km\n"
                                        "   Wh used:        %0.3f\n"
                                        "   Wh Consumed:    %0.3f\n"
                                        "   Wh Regenerated: %0.3f\n\n"
                                        , backend.trip_B.distance
                                        , backend.trip_B.wattHoursUsed
                                        , backend.trip_B.wattHoursConsumed
                                        , backend.trip_B.wattHoursRegenerated
                                       );

                            if (ImGui::Button("Reset\nTrip A    ", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                std::string append = std::format("{};\n", static_cast<int>(COMMAND_ID::RESET_TRIP_A));
                                to_send_extra.append(append);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Reset\nTrip B    ", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                std::string append = std::format("{};\n", static_cast<int>(COMMAND_ID::RESET_TRIP_B));
                                to_send_extra.append(append);
                            }

                            ImGui::Dummy(ImVec2(0, 20));
                            ImGui::Text("Estimated range\n"
                                        "   Range:       %0.3f km\n"
                                        "   Distance:    %0.3f km\n"
                                        "   Wh/km:       %0.3f\n\n"
                                        , backend.estimatedRange.range
                                        , backend.estimatedRange.distance
                                        , backend.estimatedRange.WhPerKm
                                        );

                            if (ImGui::Button("Reset\nest. Range", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                std::string append = std::format("{};\n", static_cast<int>(COMMAND_ID::RESET_ESTIMATED_RANGE));
                                to_send_extra.append(append);
                            }

                            ImGui::Dummy(ImVec2(0, 20));
                            ImGui::Text("Odometer: %0.3f km", backend.odometer_distance);

                            static char newOdometerValue[30];
                            ImGui::Text("New value: ");
                            ImGui::SameLine();
                            ImGui::SetNextItemWidth(100.0);
                            ImGui::InputText("km", newOdometerValue, sizeof(newOdometerValue));
                            // ImGui::SameLine();
                            if (ImGui::Button("Send", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                std::string append = std::format("{};{};\n", static_cast<int>(COMMAND_ID::SET_ODOMETER), newOdometerValue);
                                to_send_extra.append(append);
                            }

                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                            ImGui::SeparatorText("Battery");
                            ImGui::PopStyleColor();
                            ImGui::PopFont();


                            ImGui::Text("Charging: %s", battery.charging ? "true": "false");
                            // TODO: amphours when new is hardcoded
                            ImGui::Text("State of Charge: %0.1f%%", battery.percentage);
                            ImGui::Text("Battery Health: %0.1f%%", (battery.ampHoursFullyCharged / battery.ampHoursFullyChargedWhenNew) * 100.0);
                            ImGui::Text("Wh Capacity (When fully discharged): %0.1f Wh", (battery.watthoursFullyDischarged));
                            ImGui::Text("Wh Used: %0.5f Wh", battery.wattHoursUsed);
                            ImGui::Dummy(ImVec2(0, 20));
                            ImGui::Text("Amphours when new: %0.2f Ah", battery.ampHoursFullyChargedWhenNew);
                            ImGui::Text("Amphours Rated: %0.2f Ah", battery.ampHoursFullyCharged);
                            ImGui::Text("Amphours Used: %0.2f Ah", battery.ampHoursUsed);
                            // Amphours used lifetime since 22.09.2025
                            ImGui::Text("Amphours Used (Lifetime): %0.2f Ah", battery.ampHoursUsedLifetime);

                            ImGui::Dummy(ImVec2(0,40));

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

                            static char newAmphoursChargedValue[30];
                            ImGui::Text("Set Amphours when charged = ");
                            ImGui::SameLine();
                            ImGui::SetNextItemWidth(100.0);
                            ImGui::InputText("Ah##xx", newAmphoursChargedValue, sizeof(newAmphoursChargedValue), ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
                            // ImGui::SameLine();
                            if (ImGui::Button("Send##xxx", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                if (strlen(newAmphoursChargedValue) > 0) {
                                    std::string append = std::format("{};{};\n", static_cast<int>(COMMAND_ID::SET_AMPHOURS_CHARGED), newAmphoursChargedValue);
                                    to_send_extra.append(append);
                                }
                            }

                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                            ImGui::SeparatorText("VESC");
                            ImGui::PopStyleColor();
                            ImGui::PopFont();

                            ImGui::Text("VESC MOSFET Temperature: %0.1f°C", backend.temperature_vesc);
                            ImGui::Text(" ");

                            ImGui::BeginGroup();
                                float ItemWidth = 150.0;
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Current Scaling (Braking)", &mcconf_vesc.l_current_min_scale);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Current Scaling (Accelerating)", &mcconf_vesc.l_current_max_scale);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Reverse RPM (times 3 && negative value)", &mcconf_vesc.l_min_erpm);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Forward RPM (times 3)", &mcconf_vesc.l_max_erpm);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Min Duty Cycle", &mcconf_vesc.l_min_duty);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Max Duty Cycle", &mcconf_vesc.l_max_duty);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Reverse Power (negative value)", &mcconf_vesc.l_watt_min);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Forward Power", &mcconf_vesc.l_watt_max);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Battery Braking Current (negative value)", &mcconf_vesc.l_in_current_min);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Battery Current", &mcconf_vesc.l_in_current_max);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Profile name = %s", mcconf_vesc.name.c_str());

                                if (ImGui::Button("Get values", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                    std::string append = std::format("{};\n", static_cast<int>(COMMAND_ID::GET_VESC_MCCONF));
                                    to_send_extra.append(append);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Set values", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                                    setMcconfCustomValues(mcconf_vesc);
                                }
                            ImGui::EndGroup();


                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                            ImGui::SeparatorText("Analog Readings");
                            ImGui::PopStyleColor();
                            ImGui::PopFont();

                            ImGui::BeginGroup();
                            {
                                float ItemWidth = 150.0;
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog0:     %0.15lf", analogReadings.analog0);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog1:     %0.15lf", analogReadings.analog1);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog2:     %0.15lf", analogReadings.analog2);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog3:     %0.15lf", analogReadings.analog3);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog4:     %0.15lf", analogReadings.analog4);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog5:     %0.15lf", analogReadings.analog5);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog6:     %0.15lf", analogReadings.analog6);
                                ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog7:     %0.15lf", analogReadings.analog7);
                            }
                            ImGui::EndGroup();

                            ImGui::EndChild();
                            ImGui::EndTabItem();
                        }

                        if (ImGui::BeginTabItem("E-BIKE Log"))
                        {
                            std::string log_tmp = backend.log;

                            ImGui::InputTextMultiline("##", log_tmp.data(), log_tmp.size() + 1, ImGui::GetContentRegionAvail(), ImGuiInputTextFlags_ReadOnly);
                            ImGui::EndTabItem();
                        }

                        ImGui::EndTabBar(); //TABBAR2
                    }
                    ImGui::EndTabItem(); // if (ImGui::BeginTabItem("Settings"))
                } // Settings tab item
                ImGui::EndTabBar(); //TABBAR1
            }

            if (IPC.isConnected == false) {
                bool open = true;

                ImGui::OpenPopup("IPC Failed");
                if (ImGui::BeginPopupModal("IPC Failed", &open, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoDecoration)) {
                    ImGui::Text("IPC failed to connect to: %s", serverAddress.c_str());

                    if (ImGui::Button("Quit")) {
                        done = true;
                    }
                ImGui::EndPopup();
                }
            }

            ImGui::EndGroup();

            ImGui::End();
            timer.draw.end();
        }

        // Rendering
        timer.render.start();
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

        timer.render.end();

        // limit framerate
        static double lasttime = (float)(SDL_GetTicks() / 1000.0f);;
        if (settings.LIMIT_FRAMERATE && backend.power_on) {
            while ((float)(SDL_GetTicks() / 1000.0f) < lasttime + 1.0/settings.TARGET_FPS) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            lasttime += 1.0/settings.TARGET_FPS;
        }

        if (!backend.power_on && IPC.isConnected) {
            while ((float)(SDL_GetTicks() / 1000.0f) < lasttime + 1.0 / 3.0/* target fps*/) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            lasttime += 1.0 / 3.0/* target fps*/;
        }

        cpuUsage.ImGui.measureEnd(1);
        cpuUsage.Everything.measureEnd(0);
    }

    // Cleanup
    done = true;
    IPC.stop();
    commThread.join();

    // [If using SDL_MAIN_USE_CALLBACKS: all code below would likely be your SDL_AppQuit() function]
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DestroyContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
