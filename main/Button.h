class Button {
    private:
        static int buttonPressCount;
        static uint32_t timeButton;
        static void (*callback)(); // Function pointer: takes no args, returns void
    
    public:
        static void registerCallback(void (*func)()) {
            callback = func;
        }
    
        static void interruptHandler() {
            if (callback) {
                callback(); // Call the registered function
            }
        }
    
        static void ISR() {
            if (timer_delta_ms(timer_u32() - timeButton) >= 20) {
                timeButton = timer_u32();
    
                // the button sends two interrupts... once when pressed, and once when released
                // so run this function once every 2 interrupts
                if (buttonPressCount >= 2) {
                    buttonPressCount = 1;
                } else {
                    buttonPressCount++;
                    return;
                }
    
                interruptHandler();  // Call instance method
            }
        }
    };
    // "buttonPressCount = 2" causes the button to activate specified function when pressing the button
    // "buttonPressCount = 1" causes the button to activate specified function *after* releasing the button
    int Button::buttonPressCount = 2;
    uint32_t Button::timeButton = 0;
    void (*Button::callback)() = nullptr;