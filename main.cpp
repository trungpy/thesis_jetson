#include <app.h>
int main(int argc, char **argv) {
    try {  // Load configuration from JSON file
        std::cout << "🔧 Loading configuration..." << std::endl;
        Config::loadConfig("config.json");
        CameraSettings cameraSettings = Config::config.camera;
        std::cout << cameraSettings.focalLength << "focalLength" << cameraSettings.realObjectWidth
                  << " @ " << cameraSettings.fps << " FPS\n";
        std::cout << "✅ Configuration loaded successfully.\n";
        STrack::initializeEstimator();
        App app;
        return app.runApp(argc, argv);
    } catch (const std::exception &e) {
        std::cerr << "❌ Exception: " << e.what() << std::endl;
        return 1;
    }
}