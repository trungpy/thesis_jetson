#ifndef _APP_H
#define _APP_H

#include <process.h>

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
        if (severity <= Severity::kWARNING) std::cout << msg << std::endl;
    }

   private:
    Logger() = default;
    ~Logger() = default;
    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;
};

// Create CLI option parser
cxxopts::Options createOptions() {
    cxxopts::Options options("test", "Run inference on a video or images (choose only one)");
    options.add_options()("v,video", "Video path", cxxopts::value<std::string>())(
        "i,images", "Images path", cxxopts::value<std::string>())(
        "m,engine", "Engine path", cxxopts::value<std::string>())("h,help", "Print usage");
    return options;
}

class App {
   public:
    int runApp(int argc, char **argv) {
        auto options = createOptions();
        AppConfig config = parseArgs(argc, argv, options);

        if (config.enginePath.find(".onnx") != std::string::npos) {
            std::cout << "ℹ️ ONNX model detected, skipping inference.\n";
            return 0;
        }

        std::cout << "🔧 Loading engine from: " << config.enginePath << std::endl;
        Detect model(config.enginePath, Logger::getInstance());

        if (!config.videoPath.empty()) {
            std::cout << "🎞️ Running video inference on: " << config.videoPath << std::endl;
            if (checkVideo(config.videoPath))
                return runVideo(config.videoPath, model);
            else {
                std::cerr << "❌ Invalid video path.\n";
                return 1;
            }
        }

        if (!config.imagePath.empty()) {
            std::vector<std::string> imageList;
            std::cout << "🖼️ Running image inference in folder: " << config.imagePath
                      << std::endl;
            if (checkImages(config.imagePath, imageList))
                return runImages(imageList, model);
            else {
                std::cerr << "❌ No valid images found in: " << config.imagePath << std::endl;
                return 1;
            }
        }

        return 0;
    }

   private:
    struct AppConfig {
        std::string videoPath;
        std::string imagePath;
        std::string enginePath;
    };

    AppConfig parseArgs(int argc, char **argv, cxxopts::Options &options) {
        auto result = options.parse(argc, argv);

        if (result.count("help")) {
            std::cout << options.help() << std::endl;
            exit(0);
        }

        AppConfig config;
        if (result.count("engine"))
            config.enginePath = result["engine"].as<std::string>();
        else {
            std::cerr << "❌ Error: --engine is required.\n";
            exit(1);
        }

        if (result.count("video")) config.videoPath = result["video"].as<std::string>();
        if (result.count("images")) config.imagePath = result["images"].as<std::string>();

        if (!config.videoPath.empty() && !config.imagePath.empty()) {
            std::cerr << "❌ Error: Provide either --video or --images, not both.\n";
            exit(1);
        }

        if (config.videoPath.empty() && config.imagePath.empty()) {
            std::cerr << "❌ Error: Must provide either --video or --images.\n";
            exit(1);
        }

        return config;
    }
};

#endif  // _APP_H