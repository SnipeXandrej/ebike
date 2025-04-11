float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
    int temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    if (temp < out_min)
        temp = out_min;

    if (temp > out_max)
        temp = out_max;

    return temp;
}

std::string removeStringWithEqualSignAtTheEnd(const std::string toRemove, std::string str) {
    size_t pos = str.find(toRemove);
    str.erase(pos, toRemove.length() + 1);

    // cout << str << "\n";
    return str;
}

// why have this?
float getValueFromString(const std::string toRemove, std::string str) {
    float value;

    value = stof(removeStringWithEqualSignAtTheEnd(toRemove, str));

    return value;
}