#pragma once
#include <fstream>
#include <string>

class DataLogger {
private:
    std::ofstream log_file_;
    std::string log_filename_;

public:
    void init(const std::string &output_dir);
    void log(double timestamp, int frame_count, double fps, float ego_speed,
             int target_id, const std::string &action, int detection_count,
             float avg_distance, float front_speed, double processing_time,
             int max_speed, int acc_speed);
    ~DataLogger();
};
