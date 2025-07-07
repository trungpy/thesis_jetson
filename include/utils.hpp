#ifndef _UTILS_H
#define _UTILS_H

#include <BYTETracker.h>
#include <Detect.h>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
using namespace std;
bool checkVideo(const string &path);

bool checkImages(const string &path, vector<string> &imagePathList);

bool isTrackingClass(int classId);
double getCurrentTimeInSeconds();
std::vector<Object> filterDetections(const std::vector<Detection> &res);
int getTotalMilliseconds(const std::chrono::system_clock::time_point &start,
                         const std::chrono::system_clock::time_point &end);
#endif