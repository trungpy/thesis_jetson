#ifndef _PROCESS_HPP_
#define _PROCESS_HPP_

#include <BYTETracker.h>
#include <LaneDetector.h>

#include <cxxopts.hpp>
#include <iostream>
#include <string>
#include <utils.hpp>
#include <vector>

#include "Detect.h"
#include "config.h"
using namespace Config;
int runVideo(const std::string &path, Detect &model);
;
int runImages(const vector<string> imagePathList, Detect &model);

void selectTarget(const std::vector<STrack> &tracks, float xMin, float xMax, int &targetId,
                  cv::Rect &bestBox, float &maxHeight);
std::vector<Object> filterDetections(const std::vector<Detection> &res);

void updateSpeedControl(double timeStart, int targetId, const cv::Rect &bestBox,
                        float &currentEgoSpeed, double &lastSpeedUpdateTime,
                        std::map<int, std::deque<float>> &objectBuffers,
                        std::map<int, float> &prevDistances, std::map<int, double> &prevTimes,
                        std::map<int, float> &smoothedSpeeds, std::deque<float> &speedChangeHistory,
                        float &avgDistance, float &frontSpeed, std::string &action,
                        cv::Scalar &actionColor);

void drawHUD(cv::Mat &image, float currentEgoSpeed, int accSpeed, int maxSpeed,
             float frontSpeed, float avgDistance, const std::string &action,
             const cv::Scalar &actionColor, double fps, int targetId);

#endif