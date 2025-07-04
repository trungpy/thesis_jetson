#include <config.h>

#include <utils.hpp>
bool IsPathExist(const string &path) {
    return (access(path.c_str(), F_OK) == 0);
}

bool IsFile(const string &path) {
    if (!IsPathExist(path)) {
        printf("%s:%d %s not exist\n", __FILE__, __LINE__, path.c_str());
        return false;
    }

    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0 && S_ISREG(buffer.st_mode));
}

bool checkVideo(const string &path) {
    if (!IsFile(path)) {
        std::cerr << "❌ Path does not exist or is not a file: " << path
                  << std::endl;
        return false;
    }

    string suffix = path.substr(path.find_last_of('.') + 1);
    // Convert to lowercase for robustness
    std::transform(suffix.begin(), suffix.end(), suffix.begin(), ::tolower);

    const vector<string> validExt = {"mp4", "avi", "m4v", "mpeg",
                                     "mov", "mkv", "webm"};

    if (std::find(validExt.begin(), validExt.end(), suffix) != validExt.end()) {
        return true;
    } else {
        std::cerr << "❌ Invalid video format: ." << suffix << std::endl;
        return false;
    }
}

bool checkImages(const string &path, vector<string> &imagePathList) {
    if (IsFile(path)) {
        string suffix = path.substr(path.find_last_of('.') + 1);
        if (suffix == "jpg" || suffix == "jpeg" || suffix == "png") {
            imagePathList.push_back(path);
            return true;
        } else {
            printf("Suffix %s is not supported!\n", suffix.c_str());
            return false;
        }
    } else if (IsPathExist(path)) {
        vector<string> extensions = {"*.jpg", "*.jpeg", "*.png"};
        for (const auto &ext : extensions) {
            vector<string> tempList;
            cv::glob(path + "/" + ext, tempList, false);
            imagePathList.insert(imagePathList.end(), tempList.begin(),
                                 tempList.end());
        }
        return !imagePathList.empty();
    } else {
        printf("Path %s does not exist!\n", path.c_str());
        return false;
    }
}

bool isTrackingClass(int classId) {
    // for (auto &c : Config::config.objectTracking.trackClasses) {
    //     if (c == classId) return true;
    //   if (classId == c) return true;
    // }
    return true;
}

double getCurrentTimeInSeconds() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
               .count() /
           1000.0;
}

int getTotalMilliseconds(const std::chrono::system_clock::time_point &start,
                         const std::chrono::system_clock::time_point &end) {
    return static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count());
}