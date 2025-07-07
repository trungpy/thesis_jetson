#ifndef LOGGER_H
#define LOGGER_H
#include <NvInfer.h>
#include <iostream>
/**
 * @brief Setting up Tensorrt logger
 */
class Logger : public nvinfer1::ILogger {
public:
    static Logger &getInstance() {
        static Logger instance;
        return instance;
    }

    void log(Severity severity, const char *msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;
    }

private:
    Logger() = default;
    ~Logger() = default;
    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;
};

#endif// Logger.h