#include <map>

enum PP_VALS {
    L_CURRENT_MIN_SCALE = 0,
    L_CURRENT_MAX_SCALE = 1,
    L_MIN_ERPM = 2,
    L_MAX_ERPM = 3,
    L_MIN_DUTY = 4,
    L_MAX_DUTY = 5,
    L_WATT_MIN = 6,
    L_WATT_MAX = 7,
    L_IN_CURRENT_MIN = 8,
    L_IN_CURRENT_MAX = 9,
    VALS_COUNT,
};

std::map<PP_VALS, std::string> PP_VALS_TO_STRING = {
    {PP_VALS::L_CURRENT_MIN_SCALE, "L_CURRENT_MIN_SCALE"},
    {PP_VALS::L_CURRENT_MAX_SCALE, "L_CURRENT_MAX_SCALE"},
    {PP_VALS::L_MIN_ERPM, "L_MIN_ERPM"},
    {PP_VALS::L_MAX_ERPM, "L_MAX_ERPM"},
    {PP_VALS::L_MIN_DUTY, "L_MIN_DUTY"},
    {PP_VALS::L_MAX_DUTY, "L_MAX_DUTY"},
    {PP_VALS::L_WATT_MIN, "L_WATT_MIN"},
    {PP_VALS::L_WATT_MAX, "L_WATT_MAX"},
    {PP_VALS::L_IN_CURRENT_MIN, "L_IN_CURRENT_MIN"},
    {PP_VALS::L_IN_CURRENT_MAX, "L_IN_CURRENT_MAX"},
};

std::map<std::string, PP_VALS> STRING_TO_PP_VALS = {
    {"L_CURRENT_MIN_SCALE", PP_VALS::L_CURRENT_MIN_SCALE},
    {"L_CURRENT_MAX_SCALE", PP_VALS::L_CURRENT_MAX_SCALE},
    {"L_MIN_ERPM", PP_VALS::L_MIN_ERPM},
    {"L_MAX_ERPM", PP_VALS::L_MAX_ERPM},
    {"L_MIN_DUTY", PP_VALS::L_MIN_DUTY},
    {"L_MAX_DUTY", PP_VALS::L_MAX_DUTY},
    {"L_WATT_MIN", PP_VALS::L_WATT_MIN},
    {"L_WATT_MAX", PP_VALS::L_WATT_MAX},
    {"L_IN_CURRENT_MIN", PP_VALS::L_IN_CURRENT_MIN},
    {"L_IN_CURRENT_MAX", PP_VALS::L_IN_CURRENT_MAX},
};

enum PROFILE {
    LEGAL = 0,
    ECO = 1,
    BALANCED = 2,
    PERFORMANCE1 = 3,
    PERFORMANCE2 = 4,
    CUSTOM = 5,
    PROFILE_COUNT,
};

std::map<PROFILE, std::string> PROFILE_TO_STRING = {
    {PROFILE::LEGAL, "Legal"},
    {PROFILE::ECO, "Eco"},
    {PROFILE::BALANCED, "Balanced"},
    {PROFILE::PERFORMANCE1, "Performance1"},
    {PROFILE::PERFORMANCE2, "Performance2"},
    {PROFILE::CUSTOM, "Custom"},
};

std::map<std::string, PROFILE> STRING_TO_PROFILE = {
    {"Legal", PROFILE::LEGAL},
    {"Eco", PROFILE::ECO},
    {"Balanced", PROFILE::BALANCED},
    {"Performance1", PROFILE::PERFORMANCE1},
    {"Performance2", PROFILE::PERFORMANCE2},
    {"Custom", PROFILE::CUSTOM},
};

class PowerProfiles {
public:
    void set(int PROFILE, int VARIABLE, double VALUE) {
        settings[PROFILE][VARIABLE][0] = VALUE;
    }

    double get(int PROFILE, int VARIABLE) {
        return settings[PROFILE][VARIABLE][0];
    }

    void setProfile(int profile) {
        currentProfile = profile;
    }

    int getProfile() {
        return currentProfile;
    }

private:
    int currentProfile;
    double settings[PROFILE::PROFILE_COUNT][PP_VALS::VALS_COUNT][1];
};