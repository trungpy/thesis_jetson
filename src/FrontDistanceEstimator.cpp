#include <FrontDistanceEstimator.h>

FrontDistanceEstimator::FrontDistanceEstimator(double focalLength, double realObjectWidth)
    : m_focalLength(focalLength), m_realObjectWidth(realObjectWidth) {
    std::cout << "FrontDistanceEstimator initialized with focal length: " << m_focalLength
              << " and real object width: " << m_realObjectWidth << std::endl;

    return;
}

double FrontDistanceEstimator::estimate(double pixelDistance) {
    if (this->m_focalLength <= 0 || this->m_realObjectWidth <= 0) {
        throw std::invalid_argument("Focal length and real object width must be positive.");
    }
    return (this->m_realObjectWidth * this->m_focalLength) / pixelDistance;
}
