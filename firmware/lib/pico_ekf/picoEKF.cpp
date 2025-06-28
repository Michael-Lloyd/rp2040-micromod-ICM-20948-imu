#include "picoEKF.h"

namespace picoEKF {

Vector3 Quaternion::rotate(const Vector3& v) const {
    // Using the formula: v' = q * v * q^*
    // Optimized version: v' = v + 2*q_w*(q_xyz × v) + 2*q_xyz × (q_xyz × v)
    Vector3 qvec(x, y, z);
    Vector3 t = qvec.cross(v) * 2.0f;
    return v + t * w + qvec.cross(t);
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
    
    // Process noise covariance matrix Q
    Q.setIdentity();
    // First 4 elements are for quaternion - but we'll scale them properly
    float dt_nominal = 0.01f; // Assume nominal dt for setting Q
    for (int i = 0; i < 4; i++) {
        Q(i, i) = 0.25f * config.gyroNoise * config.gyroNoise * dt_nominal;
    }
    // Last 3 elements are for gyro bias
    for (int i = 4; i < 7; i++) {
        Q(i, i) = config.gyroBiasNoise * config.gyroBiasNoise * dt_nominal;
    }
    
    // Measurement noise covariance matrix R
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
        P(i, i) = 0.001f;  // Small initial quaternion uncertainty
    }
    for (int i = 4; i < 7; i++) {
        P(i, i) = 0.0001f;  // Small initial bias uncertainty
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
    
    // Compute state transition Jacobian
    Matrix F(7, 7);
    computeJacobians(F, omega, meas.dt);
    
    // Scale Q by actual dt
    Matrix Q_scaled(7, 7);
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            Q_scaled(i, j) = Q(i, j) * (meas.dt / 0.01f); // Scale from nominal dt
        }
    }
    
    // P = F * P * F' + Q using Joseph form for numerical stability
    Matrix FP(7, 7);
    Matrix FPFT(7, 7);
    
    // FP = F * P
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            FP(i, j) = 0;
            for (int k = 0; k < 7; k++) {
                FP(i, j) += F(i, k) * P(k, j);
            }
        }
    }
    
    // FPFT = FP * F'
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            FPFT(i, j) = 0;
            for (int k = 0; k < 7; k++) {
                FPFT(i, j) += FP(i, k) * F(j, k);
            }
        }
    }
    
    // P = FPFT + Q_scaled
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P(i, j) = FPFT(i, j) + Q_scaled(i, j);
        }
    }
    
    // Ensure P remains symmetric and positive semi-definite
    for (int i = 0; i < 7; i++) {
        for (int j = i + 1; j < 7; j++) {
            P(i, j) = P(j, i) = 0.5f * (P(i, j) + P(j, i));
        }
        // Ensure diagonal elements are positive
        if (P(i, i) < 1e-9f) {
            P(i, i) = 1e-9f;
        }
    }
    
    EKF_DEBUG_PRINT("Prediction: q=[%.3f,%.3f,%.3f,%.3f]\n", 
                    state.orientation.w, state.orientation.x, 
                    state.orientation.y, state.orientation.z);
}

void ExtendedKalmanFilter::updateState(const IMUMeasurement& meas) {
    
    // Predict measurements based on current state
    Vector3 g_predicted = state.orientation.conjugate().rotate(config.gravityRef);
    Vector3 accel_residual = meas.accel - g_predicted;
    
    int measurementDim = 3;
    Vector3 mag_residual;
    
    if (config.useMagnetometer) {
        Vector3 m_predicted = state.orientation.conjugate().rotate(config.magRef);
        mag_residual = meas.mag - m_predicted;
        measurementDim = 6;
    }
    
    // Compute measurement Jacobian
    Matrix H(measurementDim, 7);
    computeMeasurementJacobian(H, state.orientation);
    
    
    // Innovation covariance: S = H * P * H' + R
    Matrix HP(measurementDim, 7);
    Matrix HPH(measurementDim, measurementDim);
    Matrix S(measurementDim, measurementDim);
    
    // HP = H * P
    for (int i = 0; i < measurementDim; i++) {
        for (int j = 0; j < 7; j++) {
            HP(i, j) = 0;
            for (int k = 0; k < 7; k++) {
                HP(i, j) += H(i, k) * P(k, j);
            }
        }
    }
    
    // HPH = HP * H'
    for (int i = 0; i < measurementDim; i++) {
        for (int j = 0; j < measurementDim; j++) {
            HPH(i, j) = 0;
            for (int k = 0; k < 7; k++) {
                HPH(i, j) += HP(i, k) * H(j, k);
            }
        }
    }
    
    // S = HPH + R
    for (int i = 0; i < measurementDim; i++) {
        for (int j = 0; j < measurementDim; j++) {
            S(i, j) = HPH(i, j) + R(i, j);
        }
    }
    
    
    // Check S diagonal for issues
    bool hasSIssue = false;
    for (int i = 0; i < measurementDim; i++) {
        if (S(i, i) <= 0) {
            hasSIssue = true;
        }
    }
    
    if (hasSIssue) {
        return;
    }
    
    // Kalman gain: K = P * H' * S^-1
    Matrix PH(7, measurementDim);
    
    // PH = P * H'
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < measurementDim; j++) {
            PH(i, j) = 0;
            for (int k = 0; k < 7; k++) {
                PH(i, j) += P(i, k) * H(j, k);
            }
        }
    }
    
    // Invert S (simplified for 3x3 or using Cholesky for larger)
    Matrix S_inv(measurementDim, measurementDim);
    if (measurementDim == 3) {
        // 3x3 matrix inversion
        float det = S(0,0) * (S(1,1) * S(2,2) - S(2,1) * S(1,2)) -
                   S(0,1) * (S(1,0) * S(2,2) - S(1,2) * S(2,0)) +
                   S(0,2) * (S(1,0) * S(2,1) - S(1,1) * S(2,0));
        
        if (fabsf(det) > 1e-10f) {
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
        } else {
            return;
        }
    } else {
        // For 6x6, use simplified diagonal approximation
        for (int i = 0; i < measurementDim; i++) {
            for (int j = 0; j < measurementDim; j++) {
                if (i == j && fabsf(S(i, i)) > 1e-10f) {
                    S_inv(i, i) = 1.0f / S(i, i);
                } else {
                    S_inv(i, j) = 0.0f;
                }
            }
        }
    }
    
    // K = PH * S_inv
    Matrix K(7, measurementDim);
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < measurementDim; j++) {
            K(i, j) = 0;
            for (int k = 0; k < measurementDim; k++) {
                K(i, j) += PH(i, k) * S_inv(k, j);
            }
        }
    }
    
    
    // State update
    float residual[6];
    residual[0] = accel_residual.x;
    residual[1] = accel_residual.y;
    residual[2] = accel_residual.z;
    if (config.useMagnetometer) {
        residual[3] = mag_residual.x;
        residual[4] = mag_residual.y;
        residual[5] = mag_residual.z;
    }
    
    float stateUpdate[7] = {0};
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < measurementDim; j++) {
            stateUpdate[i] += K(i, j) * residual[j];
        }
    }
    
    
    // Apply state update
    state.orientation.w += stateUpdate[0];
    state.orientation.x += stateUpdate[1];
    state.orientation.y += stateUpdate[2];
    state.orientation.z += stateUpdate[3];
    state.gyroBias.x += stateUpdate[4];
    state.gyroBias.y += stateUpdate[5];
    state.gyroBias.z += stateUpdate[6];
    
    
    // Covariance update using Joseph form: P = (I-KH)*P*(I-KH)' + K*R*K'
    Matrix I_KH(7, 7);
    Matrix KH(7, 7);
    
    // KH = K * H
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            KH(i, j) = 0;
            for (int k = 0; k < measurementDim; k++) {
                KH(i, j) += K(i, k) * H(k, j);
            }
        }
    }
    
    // I_KH = I - KH
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            I_KH(i, j) = (i == j) ? 1.0f : 0.0f;
            I_KH(i, j) -= KH(i, j);
        }
    }
    
    // First part: (I-KH)*P
    Matrix IKH_P(7, 7);
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            IKH_P(i, j) = 0;
            for (int k = 0; k < 7; k++) {
                IKH_P(i, j) += I_KH(i, k) * P(k, j);
            }
        }
    }
    
    // Second part: (I-KH)*P*(I-KH)'
    Matrix P_new(7, 7);
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P_new(i, j) = 0;
            for (int k = 0; k < 7; k++) {
                P_new(i, j) += IKH_P(i, k) * I_KH(j, k);
            }
        }
    }
    
    // Add K*R*K' for numerical stability
    Matrix KR(7, measurementDim);
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < measurementDim; j++) {
            KR(i, j) = 0;
            for (int k = 0; k < measurementDim; k++) {
                KR(i, j) += K(i, k) * R(k, j);
            }
        }
    }
    
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < measurementDim; k++) {
                P_new(i, j) += KR(i, k) * K(j, k);
            }
        }
    }
    
    P = P_new;
    
    // Ensure P remains symmetric and positive semi-definite
    for (int i = 0; i < 7; i++) {
        for (int j = i + 1; j < 7; j++) {
            P(i, j) = P(j, i) = 0.5f * (P(i, j) + P(j, i));
        }
        // Ensure diagonal elements are positive
        if (P(i, i) < 1e-9f) {
            P(i, i) = 1e-9f;
        }
    }
    
    
    EKF_DEBUG_PRINT("Update: innovation=[%.3f,%.3f,%.3f]\n", 
                    residual[0], residual[1], residual[2]);
}

void ExtendedKalmanFilter::computeJacobians(Matrix& F, const Vector3& omega, float dt) {
    F.setIdentity();
    
    // The state transition for quaternion uses: q_new = q * q_delta
    // where q_delta ≈ [1, 0.5*omega*dt]
    // The Jacobian is derived from this relationship
    
    float halfDt = 0.5f * dt;
    Quaternion& q = state.orientation;
    
    // Partial derivatives of quaternion w.r.t quaternion (4x4 upper left block)
    F(0, 0) = 1.0f;
    F(0, 1) = -halfDt * omega.x;
    F(0, 2) = -halfDt * omega.y;
    F(0, 3) = -halfDt * omega.z;
    
    F(1, 0) = halfDt * omega.x;
    F(1, 1) = 1.0f;
    F(1, 2) = halfDt * omega.z;
    F(1, 3) = -halfDt * omega.y;
    
    F(2, 0) = halfDt * omega.y;
    F(2, 1) = -halfDt * omega.z;
    F(2, 2) = 1.0f;
    F(2, 3) = halfDt * omega.x;
    
    F(3, 0) = halfDt * omega.z;
    F(3, 1) = halfDt * omega.y;
    F(3, 2) = -halfDt * omega.x;
    F(3, 3) = 1.0f;
    
    // Partial derivatives of quaternion w.r.t gyro bias (4x3 upper right block)
    F(0, 4) = halfDt * q.x;
    F(0, 5) = halfDt * q.y;
    F(0, 6) = halfDt * q.z;
    
    F(1, 4) = -halfDt * q.w;
    F(1, 5) = halfDt * q.z;
    F(1, 6) = -halfDt * q.y;
    
    F(2, 4) = -halfDt * q.z;
    F(2, 5) = -halfDt * q.w;
    F(2, 6) = halfDt * q.x;
    
    F(3, 4) = halfDt * q.y;
    F(3, 5) = -halfDt * q.x;
    F(3, 6) = -halfDt * q.w;
    
    // Gyro bias doesn't change in prediction (3x3 lower right block is identity)
    // Already set by setIdentity()
    
}

void ExtendedKalmanFilter::computeMeasurementJacobian(Matrix& H, const Quaternion& q) {
    // Clear H matrix
    for (int i = 0; i < H.rows; i++) {
        for (int j = 0; j < H.cols; j++) {
            H(i, j) = 0;
        }
    }
    
    // For accelerometer: h = R^T * g_world
    // The Jacobian is ∂h/∂q
    Vector3 g = config.gravityRef;
    
    // Corrected formulas for quaternion rotation derivatives
    // These come from differentiating: h = q^* ⊗ [0; g] ⊗ q
    
    H(0, 0) = 2 * (q.w * g.x - q.z * g.y + q.y * g.z);
    H(0, 1) = 2 * (q.x * g.x + q.y * g.y + q.z * g.z);
    H(0, 2) = 2 * (-q.y * g.x + q.x * g.y + q.w * g.z);
    H(0, 3) = 2 * (-q.z * g.x - q.w * g.y + q.x * g.z);
    
    H(1, 0) = 2 * (q.z * g.x + q.w * g.y - q.x * g.z);
    H(1, 1) = 2 * (q.y * g.x - q.x * g.y - q.w * g.z);
    H(1, 2) = 2 * (q.x * g.x + q.y * g.y + q.z * g.z);
    H(1, 3) = 2 * (q.w * g.x - q.z * g.y + q.y * g.z);
    
    H(2, 0) = 2 * (-q.y * g.x + q.x * g.y + q.w * g.z);
    H(2, 1) = 2 * (q.z * g.x + q.w * g.y - q.x * g.z);
    H(2, 2) = 2 * (-q.w * g.x + q.z * g.y - q.y * g.z);
    H(2, 3) = 2 * (q.x * g.x + q.y * g.y + q.z * g.z);
    
    // For magnetometer (if used)
    if (config.useMagnetometer && H.rows >= 6) {
        Vector3 m = config.magRef;
        
        H(3, 0) = 2 * (q.w * m.x - q.z * m.y + q.y * m.z);
        H(3, 1) = 2 * (q.x * m.x + q.y * m.y + q.z * m.z);
        H(3, 2) = 2 * (-q.y * m.x + q.x * m.y + q.w * m.z);
        H(3, 3) = 2 * (-q.z * m.x - q.w * m.y + q.x * m.z);
        
        H(4, 0) = 2 * (q.z * m.x + q.w * m.y - q.x * m.z);
        H(4, 1) = 2 * (q.y * m.x - q.x * m.y - q.w * m.z);
        H(4, 2) = 2 * (q.x * m.x + q.y * m.y + q.z * m.z);
        H(4, 3) = 2 * (q.w * m.x - q.z * m.y + q.y * m.z);
        
        H(5, 0) = 2 * (-q.y * m.x + q.x * m.y + q.w * m.z);
        H(5, 1) = 2 * (q.z * m.x + q.w * m.y - q.x * m.z);
        H(5, 2) = 2 * (-q.w * m.x + q.z * m.y - q.y * m.z);
        H(5, 3) = 2 * (q.x * m.x + q.y * m.y + q.z * m.z);
    }
    
    // No dependence on gyro bias for accel/mag measurements
    // So columns 4-6 remain zero
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