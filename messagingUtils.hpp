#include <string>
#include <vector>
#include <cstdint>
#include <format>
#include <mutex>

namespace msg {
    extern std::string stringStart;
    extern std::string stringEnd;
    extern std::string messageEnd;

    extern std::mutex mtx;

    void start(std::string& message, int command);

    template<typename... Args>
    void addString(std::string& message, std::format_string<Args...> fmt, Args&&... args) {
        std::string value = std::format(fmt, std::forward<Args>(args)...);
        std::string formatted = std::format("{}{}{};", stringStart, value, stringEnd);
        message.append(formatted);
    }

    void addValue(std::string& message, double value);

    void addValue(std::string& message, double value, int precision);

    void end(std::string& message);

    std::vector<std::string> split(const std::string& input, const std::string& delimiter);

    float getValueFromSplit(std::vector<std::string> token, int &index);

    double getValueFromSplit_double(std::vector<std::string> token, int &index);

    std::string getValueFromSplit_string(std::vector<std::string> token, int &index);

    uint64_t getValueFromSplit_uint64(std::vector<std::string> token, int &index);
}