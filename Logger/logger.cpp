#include "logger.h"

// Thread-safe queue for log messages
std::queue<LogEntry> log_queue;
std::mutex queue_mutex;
std::atomic<bool> should_stop(false); // Flag to signal the logging thread to stop

std::vector<std::string> files{"state","msg"};

// Logging thread function
void log_thread()
{   
    int log_index;
    {
        std::ifstream metadata_file("metadata",std::ios::in);
        metadata_file >> log_index;
    }
    log_index++;
    {
        std::ofstream metadata_file("metadata");
        metadata_file << log_index << std::endl;
    }

    for (int i = 0; i < files.size(); i++)
    {
        files[i] = files[i] + "-" + std::to_string(log_index) + ".csv";
    }

    std::ofstream data_file(files[0]);
    std::ofstream msg_file(files[1]);
    
    bool running = true;    
    while (running)
    {
        LogEntry log_message;
        {
            // Lock the queue mutex
            std::lock_guard<std::mutex> lock(queue_mutex);

            // Check if there are any log messages in the queue
            if (!log_queue.empty())
            {
                log_message = log_queue.front();
                log_queue.pop();
            }
            else
            {
                running = !should_stop;
            }
        }

        // If we have a log message,write it to the file
        if (!log_message.message.empty()) {
            std::string log_type_str;
            switch (log_message.type)
            {
                case LogType::STATE:
                    data_file << log_message.message << std::endl;
                    data_file.flush();
                    break;
                case LogType::MSG:
                    msg_file << log_message.message << std::endl;
                    msg_file.flush();
                    break;
            }
        }
    }

    data_file.close();
    msg_file.close();
}

// Function to push log messages to the queue
void log_message(const std::string& message, std::chrono::time_point<std::chrono::high_resolution_clock>  start)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(6) << chrono::duration<double>(chrono::high_resolution_clock::now() - start).count() << "," << message;

    std::string formatted = stream.str();
    
    std::lock_guard<std::mutex> lock(queue_mutex);
    log_queue.push(LogEntry{LogType::MSG,formatted});
}

void log_state(state_t state, std::chrono::time_point<std::chrono::high_resolution_clock> start, float P0, float T0, float loop_time)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(6) << chrono::duration<double>(chrono::high_resolution_clock::now() - start).count() << "," << state.status << "," << 
    state.altimeter.pressure4 << "," << state.altimeter.pressure3 << "," << state.altimeter.pressure2 << "," << state.altimeter.pressure1 << "," << 
    state.altimeter.pressure << "," << state.altimeter.filt_pressure4 << "," << state.altimeter.filt_pressure3 << "," << 
    state.altimeter.filt_pressure2 << "," << state.altimeter.filt_pressure1 << "," << state.altimeter.filt_pressure << "," <<
    state.altimeter.temp << "," << P0 << "," << T0 << "," << state.altimeter.z << "," <<state.imu_data.heading.x << "," <<
    state.imu_data.heading.y << "," << state.imu_data.heading.z << "," << state.imu_data.accel.x << "," << state.imu_data.accel.y << "," <<
    state.imu_data.accel.z << "," << state.velo.Mach << "," << state.velo.xdot_4 << "," << state.velo.xdot_3 << "," << state.velo.xdot_2 << "," <<
    state.velo.xdot_1 << "," << state.velo.xdot << "," << state.velo.zdot_4 << "," << state.velo.zdot_3 << "," << state.velo.zdot_2 << "," <<
    state.velo.zdot_1 << "," << state.velo.zdot << "," <<
    state.velo.integral_veloz << "," << state.velo.prev_integral_veloz << "," << state.theta_window1[0] <<
    "," << state.theta_window2[0] << "," << state.dt_window1[0] << "," << state.dt_window2[0] << "," << loop_time;

    std::string formatted = stream.str();

    std::lock_guard<std::mutex> lock(queue_mutex);
    log_queue.push(LogEntry{LogType::STATE,formatted});
}

void log_stop()
{
    should_stop = true;
}