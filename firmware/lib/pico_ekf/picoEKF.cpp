#include "picoEKF.h"

namespace picoEKF {

Vector3 Quaternion::rotate(const Vector3& v) const {
    Quaternion vq(0, v.x, v.y, v.z);
    Quaternion result = (*this) * vq * conjugate();
    return Vector3(result.x, result.y, result.z);
}

void Quaternion::toRotationMatrix(float R[3][3]) const {
    float ww = w * w;
    float xx = x * x;
    float yy = y * y;
    float zz = z * z;
    float wx = w * x;
    float wy = w * y;
    float wz = w * z;
    float xy = x * y;
    float xz = x * z;
    float yz = y * z;
    
    R[0][0] = ww + xx - yy - zz;
    R[0][1] = 2.0f * (xy - wz);
    R[0][2] = 2.0f * (xz + wy);
    
    R[1][0] = 2.0f * (xy + wz);
    R[1][1] = ww - xx + yy - zz;
    R[1][2] = 2.0f * (yz - wx);
    
    R[2][0] = 2.0f * (xz - wy);
    R[2][1] = 2.0f * (yz + wx);
    R[2][2] = ww - xx - yy + zz;
}

ExtendedKalmanFilter::ExtendedKalmanFilter() 
    : P(7, 7), Q(7, 7), R(6, 6), initialized(false) {
    reset();
}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {
}

void ExtendedKalmanFilter::init(const EKFConfig& cfg) {
    config = cfg;
    
    Q.setIdentity();
    for (int i = 0; i < 3; i++) {
        Q(i, i) = config.gyroNoise * config.gyroNoise;
    }
    Q(3, 3) = 0.0f;
    for (int i = 4; i < 7; i++) {
        Q(i, i) = config.gyroBiasNoise * config.gyroBiasNoise;
    }
    
    R.setIdentity();
    for (int i = 0; i < 3; i++) {
        R(i, i) = config.accelNoise * config.accelNoise;
    }
    if (config.useMagnetometer) {
        for (int i = 3; i < 6; i++) {
            R(i, i) = config.magNoise * config.magNoise;
        }
    }
    
    initialized = true;
    EKF_DEBUG_PRINT("EKF initialized\n");
}

void ExtendedKalmanFilter::reset() {
    state.orientation = Quaternion(1, 0, 0, 0);
    state.gyroBias = Vector3(0, 0, 0);
    
    P.setIdentity();
    for (int i = 0; i < 4; i++) {
        P(i, i) = 0.01f;
    }
    for (int i = 4; i < 7; i++) {
        P(i, i) = 0.001f;
    }
    
    EKF_DEBUG_PRINT("EKF reset\n");
}

void ExtendedKalmanFilter::update(const IMUMeasurement& measurement) {
    if (!initialized) {
        EKF_DEBUG_PRINT("EKF not initialized!\n");
        return;
    }
    
    predictState(measurement);
    updateState(measurement);
    
    state.orientation = state.orientation.normalized();
}

void ExtendedKalmanFilter::predictState(const IMUMeasurement& meas) {
    Vector3 omega = meas.gyro - state.gyroBias;
    
    float omegaNorm = omega.norm();
    Quaternion dq;
    
    if (omegaNorm > 1e-6f) {
        float halfAngle = 0.5f * omegaNorm * meas.dt;
        float sinHalfAngle = sinf(halfAngle);
        float cosHalfAngle = cosf(halfAngle);
        
        Vector3 axis = omega * (1.0f / omegaNorm);
        dq = Quaternion(
            cosHalfAngle,
            axis.x * sinHalfAngle,
            axis.y * sinHalfAngle,
            axis.z * sinHalfAngle
        );
    } else {
        dq = Quaternion(1, 0.5f * omega.x * meas.dt, 0.5f * omega.y * meas.dt, 0.5f * omega.z * meas.dt);
    }
    
    state.orientation = state.orientation * dq;
    
    Matrix F(7, 7);
    computeJacobians(F, omega, meas.dt);
    
    Matrix P_pred(7, 7);
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P_pred(i, j) = 0;
            for (int k = 0; k < 7; k++) {
                P_pred(i, j) += F(i, k) * P(k, j);
            }
        }
    }
    
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P(i, j) = 0;
            for (int k = 0; k < 7; k++) {
                P(i, j) += P_pred(i, k) * F(j, k);
            }
            P(i, j) += Q(i, j);
        }
    }
    
    EKF_DEBUG_PRINT("Prediction: q=[%.3f,%.3f,%.3f,%.3f]\n", 
                    state.orientation.w, state.orientation.x, 
                    state.orientation.y, state.orientation.z);
}

void ExtendedKalmanFilter::updateState(const IMUMeasurement& meas) {
    Vector3 g_predicted = state.orientation.conjugate().rotate(config.gravityRef);
    Vector3 accel_error = meas.accel - g_predicted;
    
    int measurementDim = 3;
    Vector3 mag_error;
    
    if (config.useMagnetometer) {
        Vector3 m_predicted = state.orientation.conjugate().rotate(config.magRef);
        mag_error = meas.mag - m_predicted;
        measurementDim = 6;
    }
    
    Matrix H(measurementDim, 7);
    computeMeasurementJacobian(H, state.orientation);
    
    Matrix S(measurementDim, measurementDim);
    for (int i = 0; i < measurementDim; i++) {
        for (int j = 0; j < measurementDim; j++) {
            S(i, j) = 0;
            for (int k = 0; k < 7; k++) {
                S(i, j) += H(i, k) * P(k, j);
            }
        }
    }
    
    for (int i = 0; i < measurementDim; i++) {
        for (int j = 0; j < measurementDim; j++) {
            float sum = 0;
            for (int k = 0; k < 7; k++) {
                sum += S(i, k) * H(j, k);
            }
            S(i, j) = sum + R(i, j);
        }
    }
    
    Matrix K(7, measurementDim);
    Matrix P_HT(7, measurementDim);
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < measurementDim; j++) {
            P_HT(i, j) = 0;
            for (int k = 0; k < 7; k++) {
                P_HT(i, j) += P(i, k) * H(j, k);
            }
        }
    }
    
    Matrix S_inv(measurementDim, measurementDim);
    if (measurementDim == 3) {
        float det = S(0,0) * (S(1,1) * S(2,2) - S(2,1) * S(1,2)) -
                   S(0,1) * (S(1,0) * S(2,2) - S(1,2) * S(2,0)) +
                   S(0,2) * (S(1,0) * S(2,1) - S(1,1) * S(2,0));
        
        if (fabsf(det) > 1e-6f) {
            float inv_det = 1.0f / det;
            S_inv(0,0) = (S(1,1) * S(2,2) - S(2,1) * S(1,2)) * inv_det;
            S_inv(0,1) = (S(0,2) * S(2,1) - S(0,1) * S(2,2)) * inv_det;
            S_inv(0,2) = (S(0,1) * S(1,2) - S(0,2) * S(1,1)) * inv_det;
            S_inv(1,0) = (S(1,2) * S(2,0) - S(1,0) * S(2,2)) * inv_det;
            S_inv(1,1) = (S(0,0) * S(2,2) - S(0,2) * S(2,0)) * inv_det;
            S_inv(1,2) = (S(1,0) * S(0,2) - S(0,0) * S(1,2)) * inv_det;
            S_inv(2,0) = (S(1,0) * S(2,1) - S(2,0) * S(1,1)) * inv_det;
            S_inv(2,1) = (S(2,0) * S(0,1) - S(0,0) * S(2,1)) * inv_det;
            S_inv(2,2) = (S(0,0) * S(1,1) - S(1,0) * S(0,1)) * inv_det;
        }
    } else {
        for (int i = 0; i < measurementDim; i++) {
            S_inv(i, i) = 1.0f / S(i, i);
        }
    }
    
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < measurementDim; j++) {
            K(i, j) = 0;
            for (int k = 0; k < measurementDim; k++) {
                K(i, j) += P_HT(i, k) * S_inv(k, j);
            }
        }
    }
    
    float innovation[6];
    innovation[0] = accel_error.x;
    innovation[1] = accel_error.y;
    innovation[2] = accel_error.z;
    if (config.useMagnetometer) {
        innovation[3] = mag_error.x;
        innovation[4] = mag_error.y;
        innovation[5] = mag_error.z;
    }
    
    float stateUpdate[7] = {0};
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < measurementDim; j++) {
            stateUpdate[i] += K(i, j) * innovation[j];
        }
    }
    
    state.orientation.w += stateUpdate[0];
    state.orientation.x += stateUpdate[1];
    state.orientation.y += stateUpdate[2];
    state.orientation.z += stateUpdate[3];
    state.gyroBias.x += stateUpdate[4];
    state.gyroBias.y += stateUpdate[5];
    state.gyroBias.z += stateUpdate[6];
    
    Matrix I_KH(7, 7);
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            I_KH(i, j) = (i == j) ? 1.0f : 0.0f;
            for (int k = 0; k < measurementDim; k++) {
                I_KH(i, j) -= K(i, k) * H(k, j);
            }
        }
    }
    
    Matrix P_new(7, 7);
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P_new(i, j) = 0;
            for (int k = 0; k < 7; k++) {
                P_new(i, j) += I_KH(i, k) * P(k, j);
            }
        }
    }
    P = P_new;
    
    EKF_DEBUG_PRINT("Update: innovation=[%.3f,%.3f,%.3f]\n", 
                    innovation[0], innovation[1], innovation[2]);
}

void ExtendedKalmanFilter::computeJacobians(Matrix& F, const Vector3& omega, float dt) {
    F.setIdentity();
    
    float halfDt = 0.5f * dt;
    
    F(0, 1) = -halfDt * omega.x;
    F(0, 2) = -halfDt * omega.y;
    F(0, 3) = -halfDt * omega.z;
    F(0, 4) = halfDt * state.orientation.x;
    F(0, 5) = halfDt * state.orientation.y;
    F(0, 6) = halfDt * state.orientation.z;
    
    F(1, 0) = halfDt * omega.x;
    F(1, 2) = halfDt * omega.z;
    F(1, 3) = -halfDt * omega.y;
    F(1, 4) = -halfDt * state.orientation.w;
    F(1, 5) = halfDt * state.orientation.z;
    F(1, 6) = -halfDt * state.orientation.y;
    
    F(2, 0) = halfDt * omega.y;
    F(2, 1) = -halfDt * omega.z;
    F(2, 3) = halfDt * omega.x;
    F(2, 4) = -halfDt * state.orientation.z;
    F(2, 5) = -halfDt * state.orientation.w;
    F(2, 6) = halfDt * state.orientation.x;
    
    F(3, 0) = halfDt * omega.z;
    F(3, 1) = halfDt * omega.y;
    F(3, 2) = -halfDt * omega.x;
    F(3, 4) = halfDt * state.orientation.y;
    F(3, 5) = -halfDt * state.orientation.x;
    F(3, 6) = -halfDt * state.orientation.w;
}

void ExtendedKalmanFilter::computeMeasurementJacobian(Matrix& H, const Quaternion& q) {
    for (int i = 0; i < H.rows; i++) {
        for (int j = 0; j < H.cols; j++) {
            H(i, j) = 0;
        }
    }
    
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    
    Vector3 g = config.gravityRef;
    
    // Accelerometer measurement Jacobian (∂a/∂q where a = R^T * g_ref)
    H(0, 0) = 2 * (qz * g.y - qy * g.z);
    H(0, 1) = 2 * (qy * g.y + qz * g.z);
    H(0, 2) = 2 * (-2 * qy * g.x + qx * g.y - qw * g.z);
    H(0, 3) = 2 * (-2 * qz * g.x + qw * g.y + qx * g.z);
    
    H(1, 0) = 2 * (-qz * g.x + qx * g.z);
    H(1, 1) = 2 * (qy * g.x - 2 * qx * g.y + qw * g.z);
    H(1, 2) = 2 * (qx * g.x + qz * g.z);
    H(1, 3) = 2 * (-qw * g.x - 2 * qz * g.y + qy * g.z);
    
    H(2, 0) = 2 * (qy * g.x - qx * g.y);
    H(2, 1) = 2 * (qz * g.x - qw * g.y - 2 * qx * g.z);
    H(2, 2) = 2 * (qw * g.x + qz * g.y - 2 * qy * g.z);
    H(2, 3) = 2 * (qx * g.x + qy * g.y);
    
    if (config.useMagnetometer && H.rows >= 6) {
        Vector3 m = config.magRef;
        
        // Magnetometer measurement Jacobian (∂m/∂q where m = R^T * m_ref)
        H(3, 0) = 2 * (qz * m.y - qy * m.z);
        H(3, 1) = 2 * (qy * m.y + qz * m.z);
        H(3, 2) = 2 * (-2 * qy * m.x + qx * m.y - qw * m.z);
        H(3, 3) = 2 * (-2 * qz * m.x + qw * m.y + qx * m.z);
        
        H(4, 0) = 2 * (-qz * m.x + qx * m.z);
        H(4, 1) = 2 * (qy * m.x - 2 * qx * m.y + qw * m.z);
        H(4, 2) = 2 * (qx * m.x + qz * m.z);
        H(4, 3) = 2 * (-qw * m.x - 2 * qz * m.y + qy * m.z);
        
        H(5, 0) = 2 * (qy * m.x - qx * m.y);
        H(5, 1) = 2 * (qz * m.x - qw * m.y - 2 * qx * m.z);
        H(5, 2) = 2 * (qw * m.x + qz * m.y - 2 * qy * m.z);
        H(5, 3) = 2 * (qx * m.x + qy * m.y);
    }
}

void ExtendedKalmanFilter::getEulerAngles(float& roll, float& pitch, float& yaw) const {
    float qw = state.orientation.w;
    float qx = state.orientation.x;
    float qy = state.orientation.y;
    float qz = state.orientation.z;
    
    roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
    
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) {
        pitch = copysignf(M_PI / 2.0f, sinp);
    } else {
        pitch = asinf(sinp);
    }
    
    yaw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
}

}