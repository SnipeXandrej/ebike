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
#include "misc/cpp/imgui_stdlib.h"
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
#include "utils.hpp"
#include "commonUtils.hpp"
#include "cpuUsage.hpp"
#include "comm.h"
#include "arc_progress_bar.hpp"
#include "timer.hpp"
#include "imguiGestures.hpp"
#include "messagingUtils.hpp"
#include "waylandUtils.hpp"

#define GUI_VERSION "0.2.0"

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
    float c_current_phase_max;
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

    double range; // calculated at runtime
    double WhPerKm; // calculated at runtime
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
    float motor_rpmPerKmh;
    float motor_magnetPairs;
    float odometer_distance;
    float trip_distance;
    float phase_current;
    float phase_currentDAxis;
    float phase_currentQAxis;
    float duty_cycle;
    float temperature_motor;
    float temperature_vesc;
    float loopTimeMain_ms;
    float loopTimeThrottle_ms;
    float loopTimeVescValueProcessing_ms;
    float acceleration;
    bool power_on = false;
    std::string log;
    bool automaticRegenerativeBraking;
    int currentPowerProfile;
    bool minimizeDrivetrainBacklash;

    std::string fw_name;
    std::string fw_version;
    std::string fw_compile_date_time;

    trip trip_A;
    trip trip_B;
    double rollingRangeEstimation;
    double rollingWhPerKmEstimation;

    std::string availablePowerProfiles;

    std::string notes;
} backend;

// Limit FPS
struct {
    float TARGET_FPS;
    bool LIMIT_FRAMERATE;
    float ipcWriteWaitMs;
    int powerProfile;
    bool showMotorRPM;
    bool showAcceleration;
    bool showTripA;
    bool showMotorDutyInsteadOfMotorTemp;
    bool launchFullscreen;
    bool limitFramerateOnSwitchOff;
    bool useTripStatsForDisplayingRangeAndWhPerKm;
    bool useOnDemandRendering;
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
    MovingAverage wattageMoreSmooth;
    MovingAverage whOverKm;
    MovingAverage motorDCurrent;
    MovingAverage motorQCurrent;
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

bool successfulCommunication = false;

std::string toSend;
std::string toSendExtra;

struct {
    CPUUsage ImGui;
    CPUUsage ipcThread;
    CPUUsage ipcThreadRead;
    CPUUsage Everything;
} cpuUsage;

ClientSocket IPC;

struct {
    ArcProgressBar WhKmNow;
    ArcProgressBar phaseCurrent;
    ArcProgressBar motorTemp;
    ArcProgressBar motorDutyCycle;
    ArcProgressBar motorDCurrent;
    ArcProgressBar motorQCurrent;
} arcBar;

float buttonWidth = 170.0;
float buttonHeight = 80.0;

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
    std::string append = std::format("{};{};{};{};{};{};{};{};{};{};"
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
                                        ,mcconf.c_current_phase_max
    );

    msg::start(toSendExtra, COMMAND_ID::SET_POWER_PROFILE_CUSTOM);
    msg::addString(toSendExtra, "{}", append);
    msg::end(toSendExtra);
}

void setPowerProfile(int PROFILE) {
    msg::start(toSendExtra, COMMAND_ID::SET_POWER_PROFILE);
    msg::addValue(toSendExtra, PROFILE);
    msg::end(toSendExtra);
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
            auto readStringPacket = msg::split(line, msg::messageEnd);

            if (!readStringPacket.empty())
            for (int i = 0; i < (int)readStringPacket.size(); i++) {
                auto packet = msg::split(readStringPacket[i], ";");
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
                            battery.voltage = msg::getValueFromSplit(packet, index);
                            battery.current = msg::getValueFromSplit(packet, index);
                            battery.watts = msg::getValueFromSplit(packet, index);
                            battery.wattHoursUsed = msg::getValueFromSplit(packet, index);
                            battery.watthoursFullyDischarged = msg::getValueFromSplit(packet, index);
                            battery.ampHoursUsed = msg::getValueFromSplit(packet, index);
                            battery.ampHoursUsedLifetime = msg::getValueFromSplit(packet, index);
                            battery.ampHoursFullyCharged = msg::getValueFromSplit(packet, index);
                            battery.ampHoursFullyChargedWhenNew = msg::getValueFromSplit(packet, index);
                            battery.percentage = msg::getValueFromSplit(packet, index);
                            battery.voltage_min = msg::getValueFromSplit(packet, index);
                            battery.voltage_max = msg::getValueFromSplit(packet, index);
                            battery.nominalVoltage = msg::getValueFromSplit(packet, index);
                            battery.amphours_min_voltage = msg::getValueFromSplit(packet, index);
                            battery.amphours_max_voltage = msg::getValueFromSplit(packet, index);
                            battery.charging = msg::getValueFromSplit(packet, index);
                            break;

                        case COMMAND_ID::GET_STATS:
                            backend.speed_kmh = msg::getValueFromSplit(packet, index);
                            backend.motor_rpm = msg::getValueFromSplit(packet, index);
                            backend.motor_rpmPerKmh = msg::getValueFromSplit(packet, index);
                            backend.motor_magnetPairs = msg::getValueFromSplit(packet, index);
                            backend.odometer_distance = msg::getValueFromSplit(packet, index);
                            backend.trip_A.distance = msg::getValueFromSplit(packet, index);
                            backend.trip_A.wattHoursUsed = msg::getValueFromSplit(packet, index);
                            backend.trip_A.wattHoursConsumed = msg::getValueFromSplit(packet, index);
                            backend.trip_A.wattHoursRegenerated = msg::getValueFromSplit(packet, index);
                            backend.trip_A.range = msg::getValueFromSplit(packet, index);
                            backend.trip_B.distance = msg::getValueFromSplit(packet, index);
                            backend.trip_B.wattHoursUsed = msg::getValueFromSplit(packet, index);
                            backend.trip_B.wattHoursConsumed = msg::getValueFromSplit(packet, index);
                            backend.trip_B.wattHoursRegenerated = msg::getValueFromSplit(packet, index);
                            backend.trip_B.range = msg::getValueFromSplit(packet, index);
                            backend.phase_current = msg::getValueFromSplit(packet, index);
                            backend.phase_currentDAxis = -msg::getValueFromSplit(packet, index);
                            backend.phase_currentQAxis = msg::getValueFromSplit(packet, index);
                            backend.duty_cycle = msg::getValueFromSplit(packet, index);
                            backend.temperature_motor = msg::getValueFromSplit(packet, index);
                            backend.temperature_vesc = msg::getValueFromSplit(packet, index);
                            backend.totalSecondsSinceBoot = msg::getValueFromSplit(packet, index);
                            backend.loopTimeMain_ms = msg::getValueFromSplit(packet, index);
                            backend.loopTimeThrottle_ms = msg::getValueFromSplit(packet, index);
                            backend.loopTimeVescValueProcessing_ms = msg::getValueFromSplit(packet, index);
                            backend.acceleration = msg::getValueFromSplit(packet, index);
                            backend.power_on = (bool)msg::getValueFromSplit(packet, index);
                            backend.automaticRegenerativeBraking = (bool)msg::getValueFromSplit(packet, index);
                            backend.currentPowerProfile = (int)msg::getValueFromSplit(packet, index);
                            backend.minimizeDrivetrainBacklash = (bool)msg::getValueFromSplit(packet, index);
                            backend.rollingRangeEstimation = msg::getValueFromSplit(packet, index);
                            backend.rollingWhPerKmEstimation = msg::getValueFromSplit(packet, index);

                            backend.clockSecondsSinceBoot = (uint64_t)(backend.totalSecondsSinceBoot) % 60;
                            backend.clockMinutesSinceBoot = (uint64_t)(backend.totalSecondsSinceBoot / 60.0) % 60;
                            backend.clockHoursSinceBoot   = (uint64_t)(backend.totalSecondsSinceBoot / 60.0 / 60.0) % 24;
                            backend.clockDaysSinceBoot    = backend.totalSecondsSinceBoot / 60.0 / 60.0 / 24;

                            break;

                        case COMMAND_ID::GET_FW:
                            backend.fw_name = msg::getValueFromSplit_string(packet, index);
                            backend.fw_version = msg::getValueFromSplit_string(packet, index);
                            backend.fw_compile_date_time = msg::getValueFromSplit_string(packet, index);
                            break;

                        case COMMAND_ID::GET_VESC_MCCONF:
                            mcconf_vesc.l_current_min_scale = msg::getValueFromSplit(packet, index);
                            mcconf_vesc.l_current_max_scale = msg::getValueFromSplit(packet, index);
                            mcconf_vesc.l_min_erpm = msg::getValueFromSplit(packet, index);
                            mcconf_vesc.l_max_erpm = msg::getValueFromSplit(packet, index);
                            mcconf_vesc.l_min_duty = msg::getValueFromSplit(packet, index);
                            mcconf_vesc.l_max_duty = msg::getValueFromSplit(packet, index);
                            mcconf_vesc.l_watt_min = msg::getValueFromSplit(packet, index);
                            mcconf_vesc.l_watt_max = msg::getValueFromSplit(packet, index);
                            mcconf_vesc.l_in_current_min = msg::getValueFromSplit(packet, index);
                            mcconf_vesc.l_in_current_max = msg::getValueFromSplit(packet, index);
                            mcconf_vesc.name = msg::getValueFromSplit_string(packet, index);
                            mcconf_vesc.c_current_phase_max = msg::getValueFromSplit(packet, index);
                            break;

                        case COMMAND_ID::GET_ANALOG_READINGS:
                            analogReadings.analog0 = msg::getValueFromSplit(packet, index);
                            analogReadings.analog1 = msg::getValueFromSplit(packet, index);
                            analogReadings.analog2 = msg::getValueFromSplit(packet, index);
                            analogReadings.analog3 = msg::getValueFromSplit(packet, index);
                            analogReadings.analog4 = msg::getValueFromSplit(packet, index);
                            analogReadings.analog5 = msg::getValueFromSplit(packet, index);
                            analogReadings.analog6 = msg::getValueFromSplit(packet, index);
                            analogReadings.analog7 = msg::getValueFromSplit(packet, index);
                            break;

                        case COMMAND_ID::BACKEND_LOG:
                            backend.log.append(std::format("[{}] {}\n", currentTimeAndDate, msg::getValueFromSplit_string(packet, index)));
                            break;

                        case COMMAND_ID::GET_AVAILABLE_POWER_PROFILES:
                            backend.availablePowerProfiles = msg::getValueFromSplit_string(packet, index);

                            break;

                        case COMMAND_ID::GET_NOTES:
                            backend.notes = msg::getValueFromSplit_string(packet, index);

                            break;
                    }
                }
            }
            } // !packet.empty()
        }
}

void setupTOML(toml::table &tbl, const char* filepath) {
    tbl = toml::parse_file(filepath);

    // values
    settings.TARGET_FPS                      = tbl["settings"]["framerate"].value_or<float>(60);
    settings.LIMIT_FRAMERATE                 = tbl["settings"]["limit_framerate"].value_or<int8_t>(0);
    settings.ipcWriteWaitMs                  = tbl["settings"]["ipcWriteWaitMs"].value_or<float>(50);
    settings.showMotorRPM                    = tbl["settings"]["showMotorRPM"].value_or<int8_t>(1);
    settings.showAcceleration                = tbl["settings"]["showAcceleration"].value_or<int8_t>(1);
    settings.showTripA                       = tbl["settings"]["showTripA"].value_or<int8_t>(1);
    settings.showMotorDutyInsteadOfMotorTemp = tbl["settings"]["showMotorDutyInsteadOfMotorTemp"].value_or<int8_t>(0);
    settings.useTripStatsForDisplayingRangeAndWhPerKm = tbl["settings"]["useTripStatsForDisplayingRangeAndWhPerKm"].value_or<int8_t>(1);
    settings.launchFullscreen                = tbl["settings"]["launchFullscreen"].value_or<int8_t>(0);
    settings.limitFramerateOnSwitchOff       = tbl["settings"]["limitFramerateOnSwitchOff"].value_or<int8_t>(1);
    settings.useOnDemandRendering            = tbl["settings"]["useOnDemandRendering"].value_or<int8_t>(1);
}

void TOMLSave(toml::table &tbl, const char* filepath) {
    updateTableValue(tbl, "settings", "framerate", settings.TARGET_FPS);
    updateTableValue(tbl, "settings", "limit_framerate", settings.LIMIT_FRAMERATE);
    updateTableValue(tbl, "settings", "ipcWriteWaitMs", settings.ipcWriteWaitMs);
    updateTableValue(tbl, "settings", "showMotorRPM", settings.showMotorRPM);
    updateTableValue(tbl, "settings", "showAcceleration", settings.showAcceleration);
    updateTableValue(tbl, "settings", "showTripA", settings.showTripA);
    updateTableValue(tbl, "settings", "showMotorDutyInsteadOfMotorTemp", settings.showMotorDutyInsteadOfMotorTemp);
    updateTableValue(tbl, "settings", "useTripStatsForDisplayingRangeAndWhPerKm", settings.useTripStatsForDisplayingRangeAndWhPerKm);
    updateTableValue(tbl, "settings", "launchFullscreen", settings.launchFullscreen);
    updateTableValue(tbl, "settings", "limitFramerateOnSwitchOff", settings.limitFramerateOnSwitchOff);
    updateTableValue(tbl, "settings", "useOnDemandRendering", settings.useOnDemandRendering);
    saveTableToFile(tbl, filepath);
}

// Compute a hash of draw data to detect visual changes (skip GPU render when unchanged)
static uint64_t ComputeDrawDataHash(ImDrawData* draw_data, const ImVec2& display_size, const ImVec4& clear_color)
{
    uint64_t h = 0xcbf29ce484222325ULL; // FNV-1a offset basis
#define HASH_BYTES(ptr, len) do { \
    const unsigned char* _p = (const unsigned char*)(ptr); \
    for (size_t i = 0; i < (len); i++) { h ^= _p[i]; h *= 0x100000001b3ULL; } \
} while(0)

    HASH_BYTES(&display_size, sizeof(display_size));
    HASH_BYTES(&clear_color, sizeof(clear_color));
    if (draw_data && draw_data->Valid)
    {
        for (int n = 0; n < draw_data->CmdListsCount; n++)
        {
            const ImDrawList* cmd_list = draw_data->CmdLists[n];
            if (cmd_list->VtxBuffer.Size > 0)
                HASH_BYTES(cmd_list->VtxBuffer.Data, (size_t)cmd_list->VtxBuffer.Size * sizeof(ImDrawVert));
            if (cmd_list->IdxBuffer.Size > 0)
                HASH_BYTES(cmd_list->IdxBuffer.Data, (size_t)cmd_list->IdxBuffer.Size * sizeof(ImDrawIdx));
        }
    }
#undef HASH_BYTES
    return h;
}
uint64_t prev_draw_hash = 0;

// Main code
int main(int argc, char** argv)
{
    setenv("SDL_VIDEODRIVER", "wayland", 1);
    setenv("SDL_VIDEO_WAYLAND_ALLOW_LIBDECOR", "0", 1);

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
    movingAverages.motorDCurrent.smoothingFactor = 0.4f;
    movingAverages.motorQCurrent.smoothingFactor = 0.4f;

    // ########################
    // ######### TOML #########
    // ########################

    // TODO: if settings.toml doesnt exist, create it
    toml::table table;
    setupTOML(table, SETTINGS_FILEPATH);

    arcBar.WhKmNow.init(120.0, 180.0, 20.0, 0.0, 60.0, true, "Wh/km");
    arcBar.phaseCurrent.init(120.0, 180.0, 20.0, 0.0, 450.0, true, "Phase");
    arcBar.motorTemp.init(120.0, 180.0, 20.0, 25.0, 120.0, false, "Temp");
    arcBar.motorDutyCycle.init(120.0, 180.0, 20.0, 0.0, 100.0, true, "Duty");
    arcBar.motorDCurrent.init(120.0, 180.0, 20.0, 0.0, 100.0, true, "PhD");
    arcBar.motorQCurrent.init(120.0, 180.0, 20.0, 0.0, 450.0, true, "PhQ");

    // arcBar.motorDCurrent.setTextScaling(0.6);
    // arcBar.motorQCurrent.setTextScaling(0.6);

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

                if (strlen(readFromIPC.c_str()) == 0) {
                    successfulCommunication = false;
                }

                processRead(readFromIPC);
                msElapsedRead = std::chrono::high_resolution_clock::now() - t1;
                cpuUsage.ipcThreadRead.measureEnd(1);
            }
            std::print("[IPC Read] Thread stopped\n");
        });

        std::cout << "[IPC] Entering main while loop\n";
        while(!done) {
            cpuUsage.ipcThread.measureStart(1);

            toSend = "";
            static bool sendOnce = false;

            if (!successfulCommunication) {
                sendOnce = false;

                msg::start(toSend, COMMAND_ID::ARE_YOU_ALIVE);
                msg::end(toSend);

                IPC.write(toSend.data(), toSend.size());

                // hol'up
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (successfulCommunication) {
                auto t1 = std::chrono::high_resolution_clock::now();

                timer.ping.end();
                if (timer.ping.getTime_ms() >= 750.0) {
                    timer.ping.start();

                    msg::start(toSend, COMMAND_ID::PING);
                    msg::end(toSend);

                    if (sendOnce == false) {
                        sendOnce = true;

                        msg::start(toSend, COMMAND_ID::GET_AVAILABLE_POWER_PROFILES);
                        msg::end(toSend);

                        msg::start(toSend, COMMAND_ID::GET_VESC_MCCONF);
                        msg::end(toSend);

                        msg::start(toSend, COMMAND_ID::GET_FW);
                        msg::end(toSend);

                        msg::start(toSend, COMMAND_ID::GET_NOTES);
                        msg::end(toSend);
                    }
                }

                msg::start(toSend, COMMAND_ID::GET_BATTERY);
                msg::end(toSend);

                msg::start(toSend, COMMAND_ID::GET_STATS);
                msg::end(toSend);

                msg::start(toSend, COMMAND_ID::GET_ANALOG_READINGS);
                msg::end(toSend);

                msg::mtx.lock();
                    toSend.append(toSendExtra);
                    toSendExtra = "";
                msg::mtx.unlock();

                IPC.write(toSend.data(), toSend.size());
                if (backend.power_on) {
                    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(settings.ipcWriteWaitMs));
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                msElapsedWrite = std::chrono::high_resolution_clock::now() - t1;
            }

            cpuUsage.ipcThread.measureEnd(1);
        }

        commThreadRead.join();
        std::print("[IPC Write] Thread stopped\n");
        return 0;
    });


    std::string titleBarName = std::format("E-BIKE GUI (Connected to: {})", serverAddress);

    // ###########################
    // ##### SDL/ Dear ImGUI #####
    // ###########################

    // [If using SDL_MAIN_USE_CALLBACKS: all code below until the main loop starts would likely be your SDL_AppInit() function]
    if (!SDL_Init(SDL_INIT_VIDEO))
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

    wayland_utils::init(window);

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

    ImFont* nerdFont = io.Fonts->AddFontFromFileTTF("0xProtoNerdFont-Regular.ttf", 13.0);

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

    bool openAccelerationTester = false;

    // Main loop
    while (!done) {
        cpuUsage.ImGui.measureStart(1);
        cpuUsage.Everything.measureStart(0);
        uint64_t frameStart = SDL_GetTicksNS();

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

        static ImGuiGesture gesture;

        timer.draw.start();
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0);
        ImGui::Begin("Main", nullptr, ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoDecoration|ImGuiWindowFlags_NoBringToFrontOnFocus|ImGuiWindowFlags_NoScrollWithMouse); // Create a window called "Main" and append into it. //ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoMove

        ImGui::BeginGroup();

        ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_FittingPolicyScroll;
        {
            ImGui::PushFont(nerdFont);
                ImGui::SetCursorPosY(ImGui::GetCursorPosY() - 5.0);
                ImGui::Text("\uf013");
                if (ImGui::IsItemClicked()) {
                    gesture.openGesture();
                }
            ImGui::PopFont();
        }

        {
            // Drag handle: center title (time/date) — drag to move window via xdg-shell (Wayland)
            const float dragHandleWidth = 450.f;
            const float dragHandleHeight = 24.f;
            ImVec2 timePos((io.DisplaySize.x - dragHandleWidth) * 0.5f, 7.0f);
            ImGui::SetCursorPos(timePos);
            ImGui::InvisibleButton("##window_drag_handle", ImVec2(dragHandleWidth, dragHandleHeight));
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Drag to move window");
            static bool drag_was_active = false;
            if (ImGui::IsItemActive()) {
                if (!drag_was_active)
                    wayland_utils::drag_window(window);
                drag_was_active = true;
            } else {
                drag_was_active = false;
            }

            ImVec2 textSize = ImGui::CalcTextSize(currentTimeAndDate);
            ImGui::SetCursorPos(ImVec2((io.DisplaySize.x / 2.0) - (textSize.x / 2.0), 7.0));
            ImGui::Text("%s", currentTimeAndDate);
        }

        {
            // Bike Battery
            char text[100];
            sprintf(text, "SOC: %0.1f", battery.percentage);
            ImVec2 textSize = ImGui::CalcTextSize(text);
            ImGui::SetCursorPos(ImVec2((io.DisplaySize.x - textSize.x) - 20.0, 7.0));
            ImVec4 color = battery.charging ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 1.0, 1.0, 1.0);
            ImGui::TextColored(color, "%s", text);

            if (ImGui::IsItemClicked()) {
                msg::start(toSendExtra, COMMAND_ID::TOGGLE_CHARGING_STATE);
                msg::end(toSendExtra);
            }

            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Click to toggle charging state\nCharging: %s", battery.charging ? "true" : "false");
        }

        ImGui::Separator();

        ImGui::BeginGroup(); // Starts here
            ImGui::BeginGroup();
                ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.68);

                movingAverages.wattageMoreSmooth.moveAverage(battery.watts);
                ImGui::TextColored(ImVec4(0.0, 1.0, 0.0, 1.0), "%6.2f V", battery.voltage);
                ImGui::TextColored(ImVec4(1.0, 0.39, 0.196, 1.0), "%6.2f A", battery.current);
                ImGui::TextColored(ImVec4(1.0, 1.0, 0.0, 1.0), "%6.1f W", battery.watts);

                ImGui::PopFont();

            ImGui::EndGroup();
            //
            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20,0));
            // ImGui::SameLine();
            //
            ImGui::BeginGroup();
                // ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.68);

                // if (ImGui::Button("LIGHT", ImVec2(80 * main_scale, 50 * main_scale))) {
                //     std::string append = std::format("{};\n", static_cast<int>(COMMAND_ID::TOGGLE_FRONT_LIGHT));
                //     toSendExtra.append(append);
                // }
                // ImGui::PopFont();

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

            // Bottom Left
            ImGui::SetCursorPos(ImVec2(30.0f, io.DisplaySize.y - 0.0f - 150.0 - 105.0));
            arcBar.WhKmNow.ProgressBarArc(movingAverages.whOverKm.output);

            ImGui::SetCursorPos(ImVec2(30.0f, io.DisplaySize.y - 0.0f - 150.0));
            if (settings.showMotorDutyInsteadOfMotorTemp) {
                arcBar.motorDutyCycle.ProgressBarArc(backend.duty_cycle);
            } else {
                arcBar.motorTemp.ProgressBarArc(backend.temperature_motor);
            }
            if (ImGui::IsItemClicked()) {
                settings.showMotorDutyInsteadOfMotorTemp = !settings.showMotorDutyInsteadOfMotorTemp;
            }

            // Bottom right
            ImGui::SetCursorPos(ImVec2(io.DisplaySize.x - 150.0f, io.DisplaySize.y - 0.0f - 150.0 - 105.0));
            arcBar.motorDCurrent.ProgressBarArc(movingAverages.motorDCurrent.moveAverage(backend.phase_currentDAxis));

            ImGui::SetCursorPos(ImVec2(io.DisplaySize.x - 150.0f, io.DisplaySize.y - 0.0f - 150.0));
            arcBar.motorQCurrent.ProgressBarArc(movingAverages.motorQCurrent.moveAverage(backend.phase_currentQAxis));

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
                    if (settings.useTripStatsForDisplayingRangeAndWhPerKm) {
                        if (settings.showTripA) {
                            double whkm = backend.trip_A.wattHoursUsed / backend.trip_A.distance;
                            if (whkm != whkm) {
                                whkm = 0.0;
                            }

                            ImGui::Text("Wh/km: %0.1f¹", whkm);
                        } else {
                            double whkm = backend.trip_B.wattHoursUsed / backend.trip_B.distance;
                            if (whkm != whkm) {
                                whkm = 0.0;
                            }

                            ImGui::Text("Wh/km: %0.1f²", whkm);
                        }
                    } else {
                        ImGui::Text("Wh/km: %0.1f", backend.rollingWhPerKmEstimation);
                    }

                    // TODO: this is terrible...
                    if (ImGui::IsItemClicked()) {
                        if (settings.showTripA && settings.useTripStatsForDisplayingRangeAndWhPerKm)
                            settings.showTripA = false;
                        else if (!settings.showTripA) {
                            settings.showTripA = true;
                            settings.useTripStatsForDisplayingRangeAndWhPerKm = false;
                        } else if (!settings.useTripStatsForDisplayingRangeAndWhPerKm) {
                            settings.useTripStatsForDisplayingRangeAndWhPerKm = true;
                        }
                    }
                ImGui::PopFont();

                {
                    char text[128];
                    ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 2.0);
                    if (backend.speed_kmh >= 50.0) {
                        sprintf(text, "%0.1f >:(", backend.speed_kmh);
                    } else {
                        sprintf(text, "%0.1f", backend.speed_kmh);
                    }
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 15.0);
                    ImGui::Text("%s", text);
                    ImGui::SameLine();
                    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 33.0);
                    ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.2);
                    ImGui::Text("km/h");
                    ImGui::PopFont();
                    // TextCenteredOnLine(text, -0.5f, false);
                    ImGui::PopFont();
                }

                ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.8);
                    if (settings.useTripStatsForDisplayingRangeAndWhPerKm) {
                        if (settings.showTripA) {
                            ImGui::Text("Range: %0.1lf¹", backend.trip_A.range);
                        } else {
                            ImGui::Text("Range: %0.1lf²", backend.trip_B.range);
                        }
                    } else {
                        ImGui::Text("Range: %0.1lf", backend.rollingRangeEstimation);
                    }

                    // TODO: this is terrible...
                    if (ImGui::IsItemClicked()) {
                        if (settings.showTripA && settings.useTripStatsForDisplayingRangeAndWhPerKm)
                            settings.showTripA = false;
                        else if (!settings.showTripA) {
                            settings.showTripA = true;
                            settings.useTripStatsForDisplayingRangeAndWhPerKm = false;
                        } else if (!settings.useTripStatsForDisplayingRangeAndWhPerKm) {
                            settings.useTripStatsForDisplayingRangeAndWhPerKm = true;
                        }
                    }
                ImGui::PopFont();

                if (ImGui::IsItemHovered() && !settings.useTripStatsForDisplayingRangeAndWhPerKm) {
                    ImGui::SetTooltip(  "This is a rolling range\n"
                                        "estimation calculated from the\n"
                                        "last few kilometers travelled");
                }

                ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.8);
                    if (settings.showMotorRPM)
                        ImGui::Text("Motor RPM: %4.0f", backend.motor_rpm);

                    if (settings.showAcceleration) {
                        ImGui::Text("Accel: %0.1f", backend.acceleration);

                        if (ImGui::IsItemHovered()) {
                            ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.3);
                            ImGui::SetTooltip("Measured in km/h per second");
                            ImGui::PopFont();
                        }
                    }
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
                    }
                ImGui::PopFont();

                {
                    // ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.25);

                    ImVec2 cursorPos = ImGui::GetContentRegionAvail();
                    cursorPos.x = (cursorPos.x / 2.0) - 115;

                    ImGui::SetCursorPos(ImVec2(cursorPos.x, io.DisplaySize.y - 44.0f));

                    ImGui::SetNextItemWidth(230.0);
                    if (ImGui::Combo("##v", &backend.currentPowerProfile, backend.availablePowerProfiles.data())) {
                        setPowerProfile(backend.currentPowerProfile);

                        msg::start(toSendExtra, COMMAND_ID::GET_VESC_MCCONF);
                        msg::end(toSendExtra);
                    }

                    // ImGui::PopFont();
                }

                ImGui::EndGroup();
            }

        if (openAccelerationTester) {
            static int width = 300;
            static int maxSpeed = 90;
            static int startSpeed = 0;
            static int endSpeed = 0;
            static float time = 0;
            static float timeElapsed = 0;
            static Timer accelTimer;
            static bool startSpeedStarted = false;
            static bool endSpeedStarted = false;

            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.25f, 0.28f, 0.32f, 1.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 3.0);
            ImGui::SetNextWindowSize(ImVec2(width, 180));
            if (ImGui::Begin("Acceleration Tester", &openAccelerationTester, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse)) {
                ImGui::SetNextItemWidth(width - 16); ImGui::SliderInt("##Start km/h\n", &startSpeed, 0, maxSpeed, "Start speed: %d");
                ImGui::SetNextItemWidth(width - 16); ImGui::SliderInt("##End km/h\n", &endSpeed, 0, maxSpeed, "End speed: %d");
                ImGui::Text("%d - %d km/h = %0.3fs", startSpeed, endSpeed, time);
                ImGui::Dummy(ImVec2(20,0));
                ImGui::Text("Time elapsed: %0.3f", timeElapsed);

                int temp = endSpeed;
                if (startSpeed > endSpeed) {
                    endSpeed = startSpeed;
                }
                if (temp < startSpeed) {
                    startSpeed = temp;
                }

                if ((int)backend.speed_kmh > startSpeed) {
                    if (!startSpeedStarted) {
                        startSpeedStarted = true;
                        accelTimer.start();
                    }
                } else {
                    startSpeedStarted = false;
                }

                if ((int)backend.speed_kmh > endSpeed) {
                    if (!endSpeedStarted) {
                        endSpeedStarted = true;
                        accelTimer.end();
                        time = accelTimer.getTime_s();
                    }
                } else {
                    endSpeedStarted = false;
                    timeElapsed = accelTimer.getTime_ms_now() / 1000.0;
                }

                ImGui::End();
            }
            ImGui::PopStyleVar();
            ImGui::PopStyleColor();
        }

        if (gesture.start()) {
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 3.0);
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.25f, 0.28f, 0.32f, 1.0f));
            if (ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoDecoration)) {

            ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - 72.0);
            ImGui::SetCursorPosY(ImGui::GetWindowContentRegionMin().y - 3.0);
            if (ImGui::Button("Close")) {
                gesture.closeGesture();
            }

            ImGui::SetCursorPosY(ImGui::GetWindowContentRegionMin().y);
            ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMin().x);
            if (ImGui::BeginTabBar("Settings tabs", tab_bar_flags)) {
                if (ImGui::BeginTabItem("App Menu"))
                {
                    ImGui::BeginChild("Tab1Content", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
                    ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                    ImGui::SeparatorText("ImGui");
                    ImGui::PopStyleColor();
                    ImGui::PopFont();

                    ImGui::Text("App Version: %s", GUI_VERSION);

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

                    ImGui::Checkbox("Limit framerate", &settings.LIMIT_FRAMERATE);

                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(100);
                    const char* items[] = {"1", "5", "15", "30", "60", "90", "120", "240"};
                    static int item_current = findInArray_int(items, sizeof(items)/sizeof(items[0]), settings.TARGET_FPS);
                    if (ImGui::Combo("##v", &item_current, items, IM_ARRAYSIZE(items))) {
                        settings.TARGET_FPS = std::stof(items[item_current]);
                    }

                    ImGui::Checkbox("Show acceleration", &settings.showAcceleration);
                    ImGui::Checkbox("Show motor RPM", &settings.showMotorRPM);
                    ImGui::Checkbox("Show trip A", &settings.showTripA);
                    ImGui::Checkbox("Show motor duty instead of motor temp", &settings.showMotorDutyInsteadOfMotorTemp);
                    ImGui::Checkbox("Use Trip stats for displaying Range and Wh Per Km", &settings.useTripStatsForDisplayingRangeAndWhPerKm);
                    ImGui::Checkbox("Launch fullscreen", &settings.launchFullscreen);
                    ImGui::Checkbox("Limit framerate on switch off", &settings.limitFramerateOnSwitchOff);
                    ImGui::Checkbox("Enable On-Demand Rendering (saves processing power)", &settings.useOnDemandRendering);
                    if (ImGui::Checkbox("Open Acceleration Tester", &openAccelerationTester)) {
                        gesture.closeGesture();
                    }

                    if (ImGui::Button("Save\npreferences", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                        TOMLSave(table, SETTINGS_FILEPATH);
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
                    ImGui::Text("Bytes Sent:     %lu (%f MB)", IPC.amountOfDataSent, IPC.amountOfDataSent / 1000000.0);
                    ImGui::Text("Bytes Received: %lu (%f MB)", IPC.amountOfDataReceived, IPC.amountOfDataReceived / 1000000.0);
                    // if (ImGui::Button("Reconnect")) {
                    //     IPC.begin();
                    // }

                    ImGui::Text("Write wait time ");
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(150.0f);
                    ImGui::InputFloat("ms", &settings.ipcWriteWaitMs, 0.2, 100, "%.1f");

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

                    if (ImGui::Checkbox("Minimize Drivetrain Backlash", &backend.minimizeDrivetrainBacklash)) {
                        msg::start(toSendExtra, COMMAND_ID::SET_MINIMIZE_DRIVETRAIN_BACKLASH);
                        msg::addValue(toSendExtra, backend.minimizeDrivetrainBacklash);
                        msg::end(toSendExtra);
                    }

                    if (ImGui::Checkbox("Automatic Regenerative Braking", &backend.automaticRegenerativeBraking)) {
                        msg::start(toSendExtra, COMMAND_ID::SET_AUTOMATIC_REGEN_BRAKING);
                        msg::addValue(toSendExtra, backend.automaticRegenerativeBraking);
                        msg::end(toSendExtra);
                    }

                    ImGui::Text("Powered on: %s", backend.power_on ? "True" : "False");
                    ImGui::Dummy(ImVec2(0.0f, 20.0f));

                    ImGui::Text("Firmware");
                    char text[50];
                    sprintf(text, "   Name: %s", backend.fw_name.c_str());
                    ImGui::Text("%s", text);

                    sprintf(text, "   Version: %s", backend.fw_version.c_str());
                    ImGui::Text("%s", text);

                    sprintf(text, "   Compile Time: %s", backend.fw_compile_date_time.c_str());
                    ImGui::Text("%s", text);

                    ImGui::Text("   Uptime: %2ldd %2ldh %2ldm %2lds\n", backend.clockDaysSinceBoot, backend.clockHoursSinceBoot, backend.clockMinutesSinceBoot, backend.clockSecondsSinceBoot);

                    ImGui::Dummy(ImVec2(0.0f, 20.0f));
                    ImGui::Text("Main Loop Rate:                  %0.1f ms / %0.1f Hz", backend.loopTimeMain_ms, 1000.0 / backend.loopTimeMain_ms);
                    ImGui::Text("Throttle Loop Rate:              %0.1f ms / %0.1f Hz", backend.loopTimeThrottle_ms, 1000.0 / backend.loopTimeThrottle_ms);
                    ImGui::Text("Vesc Value Processing Loop Rate: %0.1f ms / %0.1f Hz", backend.loopTimeVescValueProcessing_ms, 1000.0 / backend.loopTimeVescValueProcessing_ms);

                    ImGui::Dummy(ImVec2(0, 20));
                    if (ImGui::Button("Save\npreferences", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                        msg::start(toSendExtra, COMMAND_ID::SAVE_PREFERENCES);
                        msg::end(toSendExtra);
                    }

                    ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                    ImGui::SeparatorText("Trip/Range/Odometer");
                    ImGui::PopStyleColor();
                    ImGui::PopFont();

                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                    ImGui::Text("Trip A");
                    ImGui::PopStyleColor();
                    ImGui::Text(
                                "   Distance:       %0.3f km\n"
                                "   Wh used:        %0.3f\n"
                                "   Wh Consumed:    %0.3f\n"
                                "   Wh Regenerated: %0.3f\n"
                                "   Range left:     %0.3f\n"
                                "   Wh/km:          %0.3f\n\n"
                                , backend.trip_A.distance
                                , backend.trip_A.wattHoursUsed
                                , backend.trip_A.wattHoursConsumed
                                , backend.trip_A.wattHoursRegenerated
                                , backend.trip_A.range
                                , backend.trip_A.wattHoursUsed / backend.trip_A.distance
                                );

                    ImGui::SameLine();
                    if (ImGui::Button("Reset##1", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                        msg::start(toSendExtra, COMMAND_ID::RESET_TRIP_A);
                        msg::end(toSendExtra);
                    }

                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                    ImGui::Text("Trip B");
                    ImGui::PopStyleColor();
                    ImGui::Text(
                                "   Distance:       %0.3f km\n"
                                "   Wh used:        %0.3f\n"
                                "   Wh Consumed:    %0.3f\n"
                                "   Wh Regenerated: %0.3f\n"
                                "   Range left:     %0.3f\n"
                                "   Wh/km:          %0.3f\n\n"
                                , backend.trip_B.distance
                                , backend.trip_B.wattHoursUsed
                                , backend.trip_B.wattHoursConsumed
                                , backend.trip_B.wattHoursRegenerated
                                , backend.trip_B.range
                                , backend.trip_B.wattHoursUsed / backend.trip_B.distance
                                );
                    ImGui::SameLine();
                    if (ImGui::Button("Reset##2", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                        msg::start(toSendExtra, COMMAND_ID::RESET_TRIP_B);
                        msg::end(toSendExtra);
                    }

                    ImGui::Dummy(ImVec2(0, 20));
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                    ImGui::Text("Odometer: ");
                    ImGui::PopStyleColor();
                    ImGui::SameLine();
                    ImGui::Text("%0.3f km", backend.odometer_distance);

                    static char newOdometerValue[30];
                    ImGui::Text("New value: ");
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(100.0);
                    ImGui::InputText("km", newOdometerValue, sizeof(newOdometerValue));
                    ImGui::SameLine();
                    if (ImGui::Button("Send", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                        msg::start(toSendExtra, COMMAND_ID::SET_ODOMETER);
                        msg::addString(toSendExtra, "{}", newOdometerValue);
                        msg::end(toSendExtra);
                    }

                    ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 1.0);
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0, 1.0, 0.78, 1.0));
                    ImGui::SeparatorText("Battery");
                    ImGui::PopStyleColor();
                    ImGui::PopFont();


                    ImGui::Text("Charging: %s", battery.charging ? "true": "false");
                    ImGui::Text("State of Charge: %0.1f%%", battery.percentage);
                    ImGui::Text("Battery Health:  %0.1f%%", (battery.ampHoursFullyCharged / battery.ampHoursFullyChargedWhenNew) * 100.0);
                    ImGui::Text("Wh Capacity:     %0.1f Wh", (battery.watthoursFullyDischarged));
                    ImGui::Text("Wh Used:         %0.1f Wh", battery.wattHoursUsed);
                    ImGui::Dummy(ImVec2(0, 20));
                    ImGui::Text("Amphours Rated (New):     %0.2f Ah", battery.ampHoursFullyChargedWhenNew);
                    ImGui::Text("Amphours Rated (Now):     %0.2f Ah", battery.ampHoursFullyCharged);
                    ImGui::Text("Amphours Used:            %0.2f Ah", battery.ampHoursUsed);
                    // Amphours used lifetime since 22.09.2025
                    ImGui::Text("Amphours Used (Lifetime): %0.2f Ah", battery.ampHoursUsedLifetime);

                    ImGui::Dummy(ImVec2(0,40));

                    static char newAmphoursUsedLifetimeValue[30];
                    ImGui::Text("Set Amphours Used (Lifetime) = ");
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(100.0);
                    ImGui::InputText("Ah", newAmphoursUsedLifetimeValue, sizeof(newAmphoursUsedLifetimeValue), ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
                    // ImGui::SameLine();
                    if (ImGui::Button("Send##xx", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                        if (strlen(newAmphoursUsedLifetimeValue) > 0) {
                            msg::start(toSendExtra, COMMAND_ID::SET_AMPHOURS_USED_LIFETIME);
                            msg::addString(toSendExtra, "{}", newAmphoursUsedLifetimeValue);
                            msg::end(toSendExtra);
                        }
                    }

                    static char newAmphoursChargedValue[30];
                    ImGui::Text("Set Amphours Rated (Now) = ");
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(100.0);
                    ImGui::InputText("Ah##xx", newAmphoursChargedValue, sizeof(newAmphoursChargedValue), ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
                    // ImGui::SameLine();
                    if (ImGui::Button("Send##xxx", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                        if (strlen(newAmphoursChargedValue) > 0) {
                            msg::start(toSendExtra, COMMAND_ID::SET_AMPHOURS_CHARGED);
                            msg::addString(toSendExtra, "{}", newAmphoursChargedValue);
                            msg::end(toSendExtra);
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
                        ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Max Reverse Speed (km/h): %0.1f", mcconf_vesc.l_min_erpm / backend.motor_magnetPairs / backend.motor_rpmPerKmh);
                        ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Max Forward Speed (km/h): %0.1f", mcconf_vesc.l_max_erpm / backend.motor_magnetPairs / backend.motor_rpmPerKmh);
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
                        ImGui::SetNextItemWidth(ItemWidth); ImGui::InputFloat("Phase Current", &mcconf_vesc.c_current_phase_max);
                        ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Profile name = %s", mcconf_vesc.name.c_str());

                        if (ImGui::Button("Get values", ImVec2(buttonWidth * main_scale, buttonHeight * main_scale))) {
                            msg::start(toSendExtra, COMMAND_ID::GET_VESC_MCCONF);
                            msg::end(toSendExtra);
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
                        ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog0:     %0.6lf", analogReadings.analog0);
                        ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog1:     %0.6lf", analogReadings.analog1);
                        ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog2:     %0.6lf", analogReadings.analog2);
                        ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog3:     %0.6lf", analogReadings.analog3);
                        ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog4:     %0.6lf", analogReadings.analog4);
                        ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog5:     %0.6lf", analogReadings.analog5);
                        ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog6:     %0.6lf", analogReadings.analog6);
                        ImGui::SetNextItemWidth(ItemWidth); ImGui::Text("Analog7:     %0.6lf", analogReadings.analog7);
                    }
                    ImGui::EndGroup();

                    ImGui::EndChild();
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("E-BIKE Log"))
                {
                    ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.4);
                    std::string log_tmp = backend.log;

                    ImGui::InputTextMultiline("##", log_tmp.data(), log_tmp.size() + 1, ImGui::GetContentRegionAvail(), ImGuiInputTextFlags_ReadOnly);
                    ImGui::PopFont();
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Notes"))
                {
                    if (ImGui::Button("Get Note")) {
                        msg::start(toSendExtra, COMMAND_ID::GET_NOTES);
                        msg::end(toSendExtra);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Save Note")) {
                        msg::start(toSendExtra, COMMAND_ID::SET_NOTES);
                        msg::addString(toSendExtra, "{}", backend.notes);
                        msg::end(toSendExtra);
                    }

                    ImGui::PushFont(ImGui::GetFont(),ImGui::GetFontSize() * 0.4);
                    ImGui::InputTextMultiline("##", &backend.notes, ImGui::GetContentRegionAvail(), ImGuiInputTextFlags_None);
                    ImGui::PopFont();
                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }
        }

        gesture.beforeEnd();

        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();
        }
        gesture.end();

        if (IPC.isConnected == false) {
            bool open = true;

            ImGui::OpenPopup("IPC Failed");
            ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f,0.5f));
            ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x * 0.85f, io.DisplaySize.y * 0.5f));
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
        ImGui::PopStyleVar();
        ImGui::PopStyleVar();
        timer.draw.end();

        // Rendering
        timer.render.start();
        ImGui::Render();
        ImDrawData* draw_data = ImGui::GetDrawData();
        uint64_t draw_hash = 0;
        static int toRender = 1;
        if (settings.useOnDemandRendering && toRender < 1) {
            draw_hash = ComputeDrawDataHash(draw_data, io.DisplaySize, clear_color);

            bool content_changed = (draw_hash != prev_draw_hash);
            if (content_changed) {
                toRender = 2;
                prev_draw_hash = draw_hash;
            }
        }

        if (toRender > 0 || !settings.useOnDemandRendering) {
            toRender--;
            glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
            glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(draw_data);
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
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(8));
        }

        // // Rendering
        // timer.render.start();
        // ImGui::Render();
        // glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        // glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        // glClear(GL_COLOR_BUFFER_BIT);
        // ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // // Update and Render additional Platform Windows
        // // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
        // //  For this specific demo app we could also call SDL_GL_MakeCurrent(window, gl_context) directly)
        // if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        // {
        //     SDL_Window* backup_current_window = SDL_GL_GetCurrentWindow();
        //     SDL_GLContext backup_current_context = SDL_GL_GetCurrentContext();
        //     ImGui::UpdatePlatformWindows();
        //     ImGui::RenderPlatformWindowsDefault();
        //     SDL_GL_MakeCurrent(backup_current_window, backup_current_context);
        // }

        // SDL_GL_SwapWindow(window);

        timer.render.end();

        if (settings.LIMIT_FRAMERATE || settings.limitFramerateOnSwitchOff) {
            bool isBikePoweredOff = (!backend.power_on && IPC.isConnected) ? true : false;

            double targetFrameTime;
            if (settings.limitFramerateOnSwitchOff && isBikePoweredOff) {
                targetFrameTime = 1e9 / 3.0;
            } else {
                targetFrameTime = 1e9 / settings.TARGET_FPS;
            }

            if (settings.LIMIT_FRAMERATE || isBikePoweredOff) {
                uint64_t frameTime = SDL_GetTicksNS() - frameStart;
                if (frameTime < targetFrameTime)
                {
                    SDL_DelayNS(targetFrameTime - frameTime);
                }
            }
        }

        cpuUsage.ImGui.measureEnd(1);
        cpuUsage.Everything.measureEnd(0);
    }

    // Cleanup
    done = true;
    IPC.stop();
    commThread.join();
    TOMLSave(table, SETTINGS_FILEPATH);

    // [If using SDL_MAIN_USE_CALLBACKS: all code below would likely be your SDL_AppQuit() function]
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DestroyContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
