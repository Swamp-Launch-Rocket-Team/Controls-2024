#include "logger.h"

int main()
{
    // Start the logging thread
    std::thread logger_thread(log_thread);

    auto start = std::chrono::high_resolution_clock::now();

    // Log some messages from the main thread
    log_message("Starting application...", start);

    state_t test;
    log_state(test, start, 0, 0, 0);
    // ... (your main application code)
    log_message("Application exited normally.", start);



    std::cout << "Test" << std::endl;

    // End logging thread
    log_stop();
    logger_thread.join();

    return 0;
}