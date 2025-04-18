// #define M_PI 3.14159

class Speedometer {
public:
    static uint32_t start;
    static uint32_t duration;

    static int buttonPressCount;

    static int wheel_diameter_mm;
    static int wheel_circumference_mm;
    static int min_time_between_interrupts_us;

    static double duration_ms;
    static double distance_travelled_mm;
    static double distance_travelled_meters;
    static double distance_travelled_in_km;
    static double speed_in_kmh;

    static void ISR() {
        duration = timer_delta_us(timer_u32() - start);

        // avoid spurious interrupts
        if (duration >= min_time_between_interrupts_us) {
            // the button sends two interrupts... once when pressed, and once when released
            // so run this function once every 2 interrupts
            if (buttonPressCount >= 2) {
                buttonPressCount = 1;
            } else {
                buttonPressCount++;
                return;
            }

            start = timer_u32();
            duration_ms = duration / 1000.0f;
            distance_travelled_mm = wheel_circumference_mm * (1000.0f/duration_ms);
            distance_travelled_meters = distance_travelled_mm / 1000.0f;
            distance_travelled_in_km = distance_travelled_meters / 1000.0f;
            speed_in_kmh = (distance_travelled_in_km * 3600.0f);

            // std::cout << "speed_in_kmh: " << speed_in_kmh << "km/h" << "\n";
        }
    }

    static void resetSpeedAfterTimeout() {
        if (timer_delta_ms(timer_u32() - start) >= 5000) {
            speed_in_kmh = 0;
        }
    }

    static void init(int WHEEL_DIAMETER_MM, int MIN_TIME_BETWEEN_INTERRUPTS_US) {
        wheel_diameter_mm = WHEEL_DIAMETER_MM;
        wheel_circumference_mm = wheel_diameter_mm * M_PI;
        min_time_between_interrupts_us = MIN_TIME_BETWEEN_INTERRUPTS_US;
    }
};

uint32_t Speedometer::start = timer_u32();
uint32_t Speedometer::duration = 0;
int Speedometer::wheel_circumference_mm = 0;
int Speedometer::wheel_diameter_mm = 0;
int Speedometer::min_time_between_interrupts_us = 0;
double Speedometer::speed_in_kmh = 0;
double Speedometer::duration_ms = 0;
double Speedometer::distance_travelled_mm = 0;
double Speedometer::distance_travelled_meters = 0;
double Speedometer::distance_travelled_in_km = 0;
int Speedometer::buttonPressCount = 2;