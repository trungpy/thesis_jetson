#pragma once
#include <iostream>
#include <stdexcept>

class FrontDistanceEstimator {
   public:
    FrontDistanceEstimator(double focalLength, double realObjectWidth);

    double estimate(double pixelDistance);

   private:
    double m_focalLength;
    double m_realObjectWidth;
};
