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
    C_PHASE_CURRENT_MAX = 10,
    C_FORCE_SINGLE_MOTOR_ACCELERATION = 11,
    VALS_COUNT,
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

std::map<PP_VALS, std::string> PP_VALS_TO_STRING;
std::map<std::string, PP_VALS> STRING_TO_PP_VALS;
std::map<PROFILE, std::string> PROFILE_TO_STRING;
std::map<std::string, PROFILE> STRING_TO_PROFILE;

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