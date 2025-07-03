#include <EgoVehicle.h>
#include <process.h>

#include <numeric>

int runImages(const vector<string> imagePathList, Detect &model) {
    // Path to folder containing images
    int fps = 30;
    BYTETracker tracker(fps, 30);
    for (const auto &imagePath : imagePathList) {
        // Open image
        Mat image = imread(imagePath);
        if (image.empty()) {
            cerr << "Error reading image: " << imagePath << endl;
            continue;
        }

        vector<Detection> res;
        model.preprocess(image);

        auto start = std::chrono::system_clock::now();
        model.infer();
        auto end = std::chrono::system_clock::now();

        model.postprocess(image, res);
        std::vector<Object> objects = filterDetections(res);
        std::vector<STrack> outputStracks = tracker.update(objects);
        model.draw(image, outputStracks);
        auto tc =
            (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() /
            1000.;
        printf("cost %2.4lf ms\n", tc);

        imshow("Result", image);

        waitKey(0);
    }
    return 0;
}
/*
 //* Example GStreamer pipeline for Jetson (commented out)


        cv::VideoCapture('nvarguscamerasrc !
        video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! nvvidconv !
        video/x-raw,format=BGRx ! videoconvert ! appsink', cv::CAP_GSTREAMER);
        std::string capture_pipeline =
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=" + std::to_string(width) +
            ", height=" + std::to_string(height) +
            ", format=NV12, framerate=" + std::to_string(fps) + "/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink drop=true";
            cv::VideoCapture cap(capture_pipeline, cv::CAP_GSTREAMER);
*/
int runVideo(const std::string &path, Detect &model) {
    cout << "Opening video: " << path << endl;

    cv::VideoCapture cap(path);

    if (!cap.isOpened()) {
        cerr << "Error: Cannot open video file!" << endl;
        return 0;
    }
    // Get frame width, height, and fps
    double fps = static_cast<int>(cap.get(cv::CAP_PROP_FPS));

    BYTETracker tracker(fps, 30);
    int frameCount = 0;
    auto fpsStartTime = std::chrono::steady_clock::now();
    int maxSpeed = -1;  // km/h

    int accSpeed = 60;  // km/h
    LaneDetector laneDetector;
    // Speed control logic
    std::string action = "FREE DRIVE";
    cv::Scalar actionColor = cv::Scalar(0, 255, 0);
    // --- Ego Vehicle Control Variables ---
    float currentEgoSpeed = config.speedControl.initialSpeedKph;
    double lastSpeedUpdateTime = 0;
    std::deque<float> speedChangeHistory;
    std::deque<float> distanceHistory;

    // --- Object tracking buffers ---
    std::map<int, std::deque<float>> objectBuffers;
    std::map<int, float> prevDistances;
    std::map<int, double> prevTimes;
    std::map<int, float> smoothedSpeeds;

    int targetId = -1;
    cv::Rect bestBox;
    int lostTargetCount = 0;
    const int MAX_LOST_FRAMES = 5;

    while (cap.isOpened()) {
        auto now = std::chrono::steady_clock::now();
        auto start = std::chrono::system_clock::now();
        double timeStart = getCurrentTimeInSeconds();
        cv::Mat image;
        cap >> image;

        if (image.empty()) {
            break;
        }
        // Resize the image to fit the window
        // cv::resize(image, image, cv::Size(width, height));
        vector<Detection> res;

        model.preprocess(image);

        model.infer();

        model.postprocess(image, res);

        std::vector<Object> objects = filterDetections(res);

        std::vector<cv::Vec4i> lanes = laneDetector.detectLanes(image);
        // Tracking
        std::vector<STrack> outputStracks = tracker.update(objects);

        auto end = std::chrono::system_clock::now();

        model.draw(image, outputStracks);

        // --- Ego Vehicle Speed Control Logic ---
        int detectedTargetId = -1;
        cv::Rect bestBoxTmp;
        float maxBottomY = -1;
        bool currentTargetStillExists = false;
        bool currentTargetInLane = false;
        float currentTargetBottomY = -1;

        // Switching criteria thresholds
        const float DISTANCE_THRESHOLD = 50.0f;  // pixels - how much closer new target must be
        const int FRAMES_OUTSIDE_LANE = 10;      // frames current target has been outside lane
        static int framesCurrentTargetOutsideLane = 0;

        // First, check if our current target still exists and update its bounding box
        if (targetId != -1) {
            auto it =
                std::find_if(outputStracks.begin(), outputStracks.end(),
                             [targetId](const STrack &obj) { return obj.track_id == targetId; });

            if (it != outputStracks.end()) {
                currentTargetStillExists = true;
                const auto &tlbr = it->tlbr;
                bestBox = cv::Rect(tlbr[0], tlbr[1], tlbr[2] - tlbr[0], tlbr[3] - tlbr[1]);
                currentTargetBottomY = tlbr[3];
            }
        }

        // Log detected objects and find lane candidates
        for (const STrack &obj : outputStracks) {
            const auto &tlbr = obj.tlbr;
            float h = tlbr[3] - tlbr[1];
            if (h > 400) continue;

            int classId = obj.classId;
            float conf = obj.score;

            // Detect speed limit signs
            if (classId >= 12 && classId <= 17 && conf > 0.6f) {
                maxSpeed = (classId - 9) * 10;
            }

            // Detect vehicles in lane
            if ((classId == 2 || classId == 4 || classId == 5) && lanes.size() >= 2) {
                cv::Point bottom_center((tlbr[0] + tlbr[2]) / 2.0f, tlbr[3]);

                std::vector<cv::Point> lane_area = {{lanes[0][0], lanes[0][1]},
                                                    {lanes[1][0], lanes[1][1]},
                                                    {lanes[1][2], lanes[1][3]},
                                                    {lanes[0][2], lanes[0][3]}};

                // Check if vehicle is in lane
                if (cv::pointPolygonTest(lane_area, bottom_center, false) >= 0) {
                    cv::Point center((tlbr[0] + tlbr[2]) / 2.0f, (tlbr[1] + tlbr[3]) / 2.0f);
                    cv::circle(image, center, 5, cv::Scalar(0, 255, 0), -1);

                    // Check if this is our current target
                    if (obj.track_id == targetId) {
                        currentTargetInLane = true;
                        lostTargetCount = 0;                 // Reset lost counter
                        framesCurrentTargetOutsideLane = 0;  // Reset outside lane counter
                    }
                    // Consider new targets (even if we have a current target)
                    else if (tlbr[3] > maxBottomY) {
                        bestBoxTmp = cv::Rect(tlbr[0], tlbr[1], tlbr[2] - tlbr[0], h);
                        detectedTargetId = obj.track_id;
                        maxBottomY = tlbr[3];  // update closest
                    }
                }
            }
        }

        // Update outside lane counter
        if (targetId != -1 && !currentTargetInLane) {
            framesCurrentTargetOutsideLane++;
        }

        // Target switching logic
        bool shouldSwitchTarget = false;
        std::string switchReason = "";

        if (targetId == -1) {
            // No current target - assign new one if found in lane
            if (detectedTargetId != -1) {
                shouldSwitchTarget = true;
                switchReason = "No current target";
            }
        } else if (!currentTargetStillExists) {
            // Current target disappeared from tracking
            lostTargetCount++;
            if (lostTargetCount >= MAX_LOST_FRAMES) {
                if (detectedTargetId != -1) {
                    shouldSwitchTarget = true;
                    switchReason = "Current target lost";
                } else {
                    targetId = -1;  // No replacement available
                    lostTargetCount = 0;
                }
            }
        } else if (detectedTargetId != -1 && detectedTargetId != targetId) {
            // We have both current and new target candidates
            // Check switching criteria:

            // 1. Current target has been outside lane detection for too long
            if (framesCurrentTargetOutsideLane >= FRAMES_OUTSIDE_LANE) {
                shouldSwitchTarget = true;
                switchReason = "Current target outside lane too long";
            }
            // 2. New target is significantly closer (more relevant for following)
            else if (currentTargetInLane &&
                     (maxBottomY - currentTargetBottomY) > DISTANCE_THRESHOLD) {
                shouldSwitchTarget = true;
                switchReason = "New target significantly closer";
            }
            // 3. Current target not in lane but new target is
            else if (!currentTargetInLane) {
                shouldSwitchTarget = true;
                switchReason = "New target in lane, current not";
            }
        }

        // Execute target switch if needed
        if (shouldSwitchTarget && detectedTargetId != -1) {
            targetId = detectedTargetId;
            bestBox = bestBoxTmp;
            lostTargetCount = 0;
            framesCurrentTargetOutsideLane = 0;

            // Debug output
            std::cout << "Target switched to ID " << targetId << " - Reason: " << switchReason
                      << std::endl;
        }

        float avgDistance = 0.0f;
        float frontAbsoluteSpeed = 0.0f;
        EgoVehicle::updateSpeedControl(timeStart, targetId, bestBox, currentEgoSpeed,
                                       lastSpeedUpdateTime, objectBuffers, prevDistances, prevTimes,
                                       smoothedSpeeds, speedChangeHistory, avgDistance,
                                       frontAbsoluteSpeed, action, actionColor);

        // Always display information on frame
        drawHUD(image, currentEgoSpeed, accSpeed, maxSpeed, frontAbsoluteSpeed, avgDistance, action,
                actionColor, fps, targetId);

        laneDetector.drawLanes(image, lanes);

        // fps calculation
        frameCount++;

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - fpsStartTime).count();
        if (elapsed >= 1) {
            fps = frameCount / static_cast<double>(elapsed);
            frameCount = 0;
            fpsStartTime = now;
        }

        if (maxSpeed != -1) {
            if (accSpeed < maxSpeed && accSpeed < config.speedControl.cruiseSpeedKph) {
                accSpeed += 1;  // Increase speed by 1 km/h
            } else if (accSpeed > maxSpeed && accSpeed > 0) {
                accSpeed -= 1;  // Decrease speed by 1 km/h
            }
        } else {
            accSpeed = config.speedControl
                           .cruiseSpeedKph;  // Reset to max speed if no speed limit detected
        }

        cv::imshow("Result", image);
        if (cv::waitKey(1) == 'q') {  // Press 'q' to exit
            break;
        }
    }

    // Release resources
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
void drawHUD(cv::Mat &image, float currentEgoSpeed, int accSpeed, int maxSpeed, float frontSpeed,
             float avgDistance, const std::string &action, const cv::Scalar &actionColor,
             double fps, int targetId) {
    // 1. FPS
    cv::putText(image, cv::format("FPS: %.1f", fps), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, white,
                2);

    // 2. Speed Limit
    if (maxSpeed != -1) {
        cv::putText(image, "Max Speed: " + std::to_string(maxSpeed) + " Km/h", {10, 60},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, red, 2);
    } else {
        cv::putText(image, "No Speed Limit Detected", {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.7, red,
                    2);
    }

    // 3. Cruise Control
    cv::putText(image,
                "Cruise Control: " + std::to_string(config.speedControl.cruiseSpeedKph) + " Km/h",
                {10, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.7, yellow, 2);

    // 4. Front Vehicle Speed & Distance
    if (targetId != -1 && avgDistance > 0) {
        cv::putText(image, "Front Speed: " + std::to_string((int)frontSpeed) + " Km/h", {10, 120},
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, orange, 2);

        // Distance color zones
        cv::Scalar zoneColor = (avgDistance < config.speedControl.criticalDistance)
                                   ? cv::Scalar(0, 0, 255)  // Red
                               : (avgDistance < config.speedControl.minFollowingDistance)
                                   ? cv::Scalar(0, 128, 255)  // Orange
                               : (avgDistance < config.speedControl.targetFollowingDistance)
                                   ? cv::Scalar(0, 255, 255)  // Yellow
                                   : cv::Scalar(0, 255, 0);   // Green

        cv::circle(image, {image.cols - 100, 100}, 30, zoneColor, -1);
        cv::putText(image, std::to_string((int)avgDistance) + "m", {image.cols - 120, 110},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, white, 1);
    } else {
        cv::putText(image, "Front Speed: -- km/h", {10, 120}, cv::FONT_HERSHEY_SIMPLEX, 0.8, gray,
                    2);
    }

    // 5. Ego Vehicle Speed
    cv::putText(image, "Ego Speed: " + std::to_string((int)currentEgoSpeed) + " Km/h", {10, 150},
                cv::FONT_HERSHEY_SIMPLEX, 0.8, green, 2);

    // 6. Driving Action
    cv::putText(image, "Action: " + action, {10, 180}, cv::FONT_HERSHEY_SIMPLEX, 0.8, actionColor,
                2);
    // 7. Target ID
    if (targetId != -1) {
        cv::putText(image, "Flow car id: " + std::to_string(targetId
                ), {10, 210}, cv::FONT_HERSHEY_SIMPLEX, 0.8, white, 2);
    } else {
        cv::putText(image, "No flow", {10, 210},
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, gray, 2);
    }
}

std::vector<Object> filterDetections(const std::vector<Detection> &res) {
    std::vector<Object> objects;
    for (const auto &obj : res) {
        if (isTrackingClass(obj.classId)) {
            objects.push_back({obj.bbox, obj.classId, obj.conf});
        }
    }
    return objects;
}

void selectTarget(const std::vector<STrack> &tracks, float xMin, float xMax, int &targetId,
                  cv::Rect &bestBox, float &maxHeight) {
    for (const auto &track : tracks) {
        if (!track.is_activated) continue;
        auto &tlbr = track.tlbr;
        float x1 = tlbr[0], y1 = tlbr[1], x2 = tlbr[2], y2 = tlbr[3];
        float xCenter = (x1 + x2) / 2.0f;
        float height = y2 - y1;

        if (xMin <= xCenter && xCenter <= xMax && height > maxHeight) {
            maxHeight = height;
            targetId = track.track_id;
            bestBox = cv::Rect(x1, y1, x2 - x1, y2 - y1);
        }
    }
}
