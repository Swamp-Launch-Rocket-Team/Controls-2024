#include <chrono>
#include <iostream>

std::chrono::time_point<std::chrono::system_clock> subtractFloatFromTimePoint(float floatValue, const std::chrono::time_point<std::chrono::system_clock>& timePoint) {
    using namespace std::chrono;
    // Convert the float to a duration (assuming it represents seconds)
    auto durationToSubtract = duration_cast<system_clock::duration>(std::chrono::duration<float>(floatValue));
    // Subtract the duration from the time point
    return timePoint - durationToSubtract;
}

int main() {
    float floatValue = 10.5; // Example float value to subtract (representing seconds)
    // Get the current time point
    auto currentTimePoint = std::chrono::high_resolution_clock::now();
    // Subtract the float value from the current time point
    // auto newTimePoint = subtractFloatFromTimePoint(floatValue, currentTimePoint);
    auto yea = currentTimePoint - std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::duration<float>(floatValue));
    std::cout << std::chrono::duration<double>(currentTimePoint - yea).count() << std::endl;

    // Do something with newTimePoint

    return 0;
}