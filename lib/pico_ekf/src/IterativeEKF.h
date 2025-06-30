#ifndef ITERATIVE_EKF_H
#define ITERATIVE_EKF_H

#include "ExtendedKalmanFilter.h"
#include "picoEKFTypes.h"

namespace picoEKF {

class IterativeEKF : public ExtendedKalmanFilter {
public:
    IterativeEKF();
    ~IterativeEKF() override = default;

    void init(const EKFConfig& config) override;
    void reset() override;
    void update(const IMUMeasurement& measurement) override;
    Quaternion getOrientation() const override;
    Vector3 getGyroBias() const override;
    void getEulerAngles(float& roll, float& pitch, float& yaw) const override;
    bool isInitialized() const override;

private:
    // State and covariance
    EKFState state_;
    Matrix P_{7, 7};  // State covariance (7x7)
    
    // Configuration
    EKFConfig config_;
    bool initialized_;
    
    // Iteration parameters
    static constexpr int MAX_ITERATIONS = 5;
    static constexpr float CONVERGENCE_THRESHOLD = 1e-5f;
    
    // Helper functions
    void predictState(const IMUMeasurement& measurement);
    void updateStateIterative(const IMUMeasurement& measurement);
    
    // Measurement model functions
    Vector3 predictGravity(const Quaternion& q) const;
    Vector3 predictMagnetometer(const Quaternion& q) const;
    
    // Jacobian computation
    void computeMeasurementJacobian(const Quaternion& q, const Vector3& gyroBias,
                                   float H[6][7], bool useMag) const;
    
    // Matrix operations
    void computeKalmanGain(const float H[6][7], const float R[6][6], 
                          float K[7][6], int measDim) const;
    bool matrixInverse3x3(const float A[3][3], float Ainv[3][3]) const;
    bool matrixInverse6x6(const float A[6][6], float Ainv[6][6]) const;
    
    // State update helpers
    void applyStateCorrection(const float K[7][6], const float innovation[6], int measDim);
    void normalizeQuaternion();
    void enforceCovarianaceSymmetry();
    
    // Debug utilities
    void printDebugInfo(const char* stage, const IMUMeasurement& measurement) const;
    void printIterationInfo(int iteration, const float* stateCorrection, float convergenceMetric) const;
};

} // namespace picoEKF

#endif // ITERATIVE_EKF_H