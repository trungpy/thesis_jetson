#ifndef _TRAFFIC_SIGN_STABILIZER_H
#define _TRAFFIC_SIGN_STABILIZER_H
#include <deque>
#include <string>
#include <unordered_map>

class TrafficSignStabilizer {
public:
    TrafficSignStabilizer(int window_size = 5) : window_size_(window_size) {}

    // Add new detected sign and get the most stable sign
    std::string update(const std::string &new_sign) {
        // Add to history
        history_.push_back(new_sign);
        if (history_.size() > window_size_) {
            history_.pop_front();
        }

        // Count frequency
        std::unordered_map<std::string, int> counts;
        for (const auto &sign : history_) {
            counts[sign]++;
        }

        // Find most common sign
        int max_count = 0;
        std::string most_common;
        for (const auto &pair : counts) {
            if (pair.second > max_count) {
                max_count = pair.second;
                most_common = pair.first;
            }
        }

        return most_common;
    }
    void clear() { history_.clear(); }

private:
    size_t window_size_;
    std::deque<std::string> history_;
};

#endif //_TRAFFIC_SIGN_STABILIZER_H