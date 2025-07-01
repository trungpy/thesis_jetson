#include <app.h>


int main(int argc, char **argv) {
    try {
        App app;
        return app.runApp(argc, argv);
    } catch (const std::exception &e) {
        std::cerr << "❌ Exception: " << e.what() << std::endl;
        return 1;
    }
}