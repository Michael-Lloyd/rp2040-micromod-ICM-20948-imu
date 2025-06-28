#ifndef STANDARD_EKF_H
#define STANDARD_EKF_H

#include "picoEKFTypes.h"
#include "ExtendedKalmanFilter.h"

namespace picoEKF {

class StandardEKF : public ExtendedKalmanFilter {
private:
    EKFState state;
    Matrix P;
    Matrix Q;
    Matrix R;
    EKFConfig config;
    
    bool initialized;
    
    void predictState(const IMUMeasurement& meas);
    void updateState(const IMUMeasurement& meas);
    void computeJacobians(Matrix& F, const Vector3& omega, float dt);
    void computeMeasurementJacobian(Matrix& H, const Quaternion& q);
    
public:
    StandardEKF();
    virtual ~StandardEKF() override;
    
    virtual void init(const EKFConfig& cfg) override;
    virtual void reset() override;
    
    virtual void update(const IMUMeasurement& measurement) override;
    
    virtual Quaternion getOrientation() const override { return state.orientation; }
    virtual Vector3 getGyroBias() const override { return state.gyroBias; }
    
    virtual void getEulerAngles(float& roll, float& pitch, float& yaw) const override;
    
    virtual bool isInitialized() const override { return initialized; }
};

}

#endif