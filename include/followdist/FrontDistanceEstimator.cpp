#include <followdist/FrontDistanceEstimator.h>

FrontDistanceEstimator::FrontDistanceEstimator() {}
double FrontDistanceEstimator::estimate(double pixelDistance, double focalLength, double realObjectWidth) {
    if (focalLength <= 0 || realObjectWidth <= 0) {
        throw std::invalid_argument("Focal length and real object width must be positive.");
    }    
    return (realObjectWidth * focalLength) / pixelDistance;
}