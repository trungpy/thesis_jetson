#pragma once
#include <iostream>
#include <stdexcept>

class FrontDistanceEstimator {
public:
    FrontDistanceEstimator();
    double estimate(double pixelDistance, double focalLength, double realObjectWidth);
};