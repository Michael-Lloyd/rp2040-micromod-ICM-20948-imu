#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "picoEKFTypes.h"

namespace picoEKF {

class ExtendedKalmanFilter {
public:
    virtual ~ExtendedKalmanFilter() = default;
    
    virtual void init(const EKFConfig& cfg) = 0;
    virtual void reset() = 0;
    
    virtual void update(const IMUMeasurement& measurement) = 0;
    
    virtual Quaternion getOrientation() const = 0;
    virtual Vector3 getGyroBias() const = 0;
    
    virtual void getEulerAngles(float& roll, float& pitch, float& yaw) const = 0;
    
    virtual bool isInitialized() const = 0;
};

enum class EKFType {
    Standard,
    Iterative,
    // Future types can be added here
    // Unscented,
    // SquareRoot,
    // etc.
};

}

#endif