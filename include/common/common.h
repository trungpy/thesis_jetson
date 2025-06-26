// common.h
// =============================================================================
// Project: LaneVision

#pragma once
#include <iostream>
#include <vector>

const std::vector<std::string> CLASS_NAMES = {
    "person",                    // 0
    "bicycle",                   // 1
    "car",                       // 2
    "motorcycle",                // 3
    "bus",                       // 4
    "truck",                     // 5
    "stop sign",                // 6
    "other-vehicle",             // 7
    "crosswalk",                 // 8
    "red light",                 // 9
    "yellow light",              // 10
    "green light",               // 11
    "Speed limit 30km-h",        // 12
    "Speed limit 40km-h",        // 13
    "Speed limit 50km-h",        // 14
    "Speed limit 60km-h",        // 15
    "Speed limit 70km-h",        // 16
    "Speed limit 80km-h",        // 17
    "End of speed limit 60km-h", // 18
    "End of speed limit 70km-h", // 19
    "End of speed limit 80km-h"  // 20
};

const std::vector<std::vector<unsigned int>> COLORS = {
    {220, 20, 60},    // person
    {119, 172, 48},   // bicycle
    {0, 114, 189},    // car
    {237, 177, 32},   // motorcycle
    {126, 47, 142},   // bus
    {217, 83, 25},    // truck
    {255, 0, 0},      // stop sign (bright red)
    {153, 153, 153},  // other-vehicle
    {255, 255, 255},  // crosswalk (white)
    {255, 0, 0},      // red light
    {255, 255, 0},    // yellow light
    {0, 255, 0},      // green light
    {170, 255, 0},    // Speed limit 30km-h
    {200, 255, 0},    // Speed limit 40km-h
    {255, 255, 0},    // Speed limit 50km-h
    {255, 200, 0},    // Speed limit 60km-h
    {255, 170, 0},    // Speed limit 70km-h
    {255, 85, 0},     // Speed limit 80km-h
    {180, 180, 180},  // End of speed limit 60km-h (light gray)
    {140, 140, 140},  // End of speed limit 70km-h (medium gray)
    {100, 100, 100}   // End of speed limit 80km-h (dark gray)
};
