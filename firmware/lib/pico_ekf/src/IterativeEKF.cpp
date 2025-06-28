#include "IterativeEKF.h"
#include <cmath>
#include <cstring>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef PICO_EKF_DEBUG
#include <cstdio>
#endif

namespace picoEKF {

IterativeEKF::IterativeEKF() : initialized_(false) {
    reset();
}

void IterativeEKF::init(const EKFConfig& config) {
    config_ = config;
    reset();
    initialized_ = true;
    
    EKF_DEBUG_PRINT("IterativeEKF initialized with:\n");
    EKF_DEBUG_PRINT("  Process noise - gyro: %.6f, bias: %.6f\n", 
                    config.gyroNoise, config.gyroBiasNoise);
    EKF_DEBUG_PRINT("  Measurement noise - accel: %.6f, mag: %.6f\n",
                    config.accelNoise, config.magNoise);
    EKF_DEBUG_PRINT("  Max iterations: %d, convergence threshold: %.6f\n",
                    MAX_ITERATIONS, CONVERGENCE_THRESHOLD);
}

void IterativeEKF::reset() {
    // Initialize state to identity quaternion and zero bias
    state_.orientation = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    state_.gyroBias = Vector3(0.0f, 0.0f, 0.0f);
    
    // Initialize covariance matrix
    memset(P_.data, 0, sizeof(P_.data));
    
    // Initial quaternion uncertainty
    for (int i = 0; i < 4; ++i) {
        P_.data[i][i] = 0.1f * 0.1f;
    }
    
    // Initial gyro bias uncertainty
    for (int i = 4; i < 7; ++i) {
        P_.data[i][i] = 0.01f * 0.01f;
    }
    
    EKF_DEBUG_PRINT("IterativeEKF reset to initial state\n");
}

void IterativeEKF::update(const IMUMeasurement& measurement) {
    if (!initialized_) {
        EKF_DEBUG_PRINT("Warning: IterativeEKF not initialized\n");
        return;
    }
    
    // Validate time step
    float dt = measurement.dt;
    if (dt <= 0.0f || dt > 1.0f) {
        EKF_DEBUG_PRINT("Warning: Invalid time step %.6f, clamping to [0.001, 1.0]\n", dt);
        dt = std::max(0.001f, std::min(dt, 1.0f));
    }
    
    // Prediction step
    predictState(measurement);
    
    // Update step with iterations
    updateStateIterative(measurement);
}

void IterativeEKF::predictState(const IMUMeasurement& measurement) {
    float dt = std::max(0.001f, std::min(measurement.dt, 1.0f));
    
    EKF_DEBUG_PRINT("\n--- Prediction Step ---\n");
    EKF_DEBUG_PRINT("dt: %.6f\n", dt);
    EKF_DEBUG_PRINT("Gyro measurement: [%.6f, %.6f, %.6f]\n", 
                    measurement.gyro.x, measurement.gyro.y, measurement.gyro.z);
    
    // Correct gyroscope measurement with bias
    Vector3 omega = measurement.gyro - state_.gyroBias;
    
    // Compute rotation angle
    float angle = omega.norm() * dt;
    
    // Update quaternion using exponential map
    Quaternion dq;
    if (angle > 1e-6f) {
        float half_angle = angle * 0.5f;
        float sinc = std::sin(half_angle) / angle;
        dq = Quaternion(std::cos(half_angle), 
                       omega.x * sinc * dt,
                       omega.y * sinc * dt,
                       omega.z * sinc * dt);
    } else {
        // Small angle approximation
        dq = Quaternion(1.0f, 
                       omega.x * dt * 0.5f,
                       omega.y * dt * 0.5f,
                       omega.z * dt * 0.5f);
    }
    
    // Update state
    state_.orientation = state_.orientation * dq;
    state_.orientation = state_.orientation.normalized();
    
    // Compute state transition Jacobian F
    float F[7][7];
    memset(F, 0, sizeof(F));
    
    // Identity for quaternion part
    for (int i = 0; i < 4; ++i) {
        F[i][i] = 1.0f;
    }
    
    // Quaternion derivative w.r.t. quaternion
    float wx = omega.x * dt * 0.5f;
    float wy = omega.y * dt * 0.5f;
    float wz = omega.z * dt * 0.5f;
    
    F[0][1] = -wx; F[0][2] = -wy; F[0][3] = -wz;
    F[1][0] =  wx; F[1][2] =  wz; F[1][3] = -wy;
    F[2][0] =  wy; F[2][1] = -wz; F[2][3] =  wx;
    F[3][0] =  wz; F[3][1] =  wy; F[3][2] = -wx;
    
    // Quaternion derivative w.r.t. gyro bias
    float q0 = state_.orientation.w;
    float q1 = state_.orientation.x;
    float q2 = state_.orientation.y;
    float q3 = state_.orientation.z;
    
    F[0][4] =  q1 * dt * 0.5f; F[0][5] =  q2 * dt * 0.5f; F[0][6] =  q3 * dt * 0.5f;
    F[1][4] = -q0 * dt * 0.5f; F[1][5] =  q3 * dt * 0.5f; F[1][6] = -q2 * dt * 0.5f;
    F[2][4] = -q3 * dt * 0.5f; F[2][5] = -q0 * dt * 0.5f; F[2][6] =  q1 * dt * 0.5f;
    F[3][4] =  q2 * dt * 0.5f; F[3][5] = -q1 * dt * 0.5f; F[3][6] = -q0 * dt * 0.5f;
    
    // Identity for bias part
    for (int i = 4; i < 7; ++i) {
        F[i][i] = 1.0f;
    }
    
    // Process noise covariance Q
    float Q[7][7];
    memset(Q, 0, sizeof(Q));
    
    // Quaternion process noise
    float gyro_noise = config_.gyroNoise * dt;
    for (int i = 0; i < 4; ++i) {
        Q[i][i] = gyro_noise * gyro_noise;
    }
    
    // Bias process noise
    float bias_noise = config_.gyroBiasNoise * dt;
    for (int i = 4; i < 7; ++i) {
        Q[i][i] = bias_noise * bias_noise;
    }
    
    // Update covariance: P = F * P * F' + Q
    float P_temp[7][7];
    
    // P_temp = F * P
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            P_temp[i][j] = 0.0f;
            for (int k = 0; k < 7; ++k) {
                P_temp[i][j] += F[i][k] * P_.data[k][j];
            }
        }
    }
    
    // P = P_temp * F' + Q
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            P_.data[i][j] = Q[i][j];
            for (int k = 0; k < 7; ++k) {
                P_.data[i][j] += P_temp[i][k] * F[j][k];
            }
        }
    }
    
    // Ensure symmetry
    enforceCovarianaceSymmetry();
    
    EKF_DEBUG_PRINT("Predicted state - q: [%.6f, %.6f, %.6f, %.6f], bias: [%.6f, %.6f, %.6f]\n",
                    state_.orientation.w, state_.orientation.x, state_.orientation.y, state_.orientation.z,
                    state_.gyroBias.x, state_.gyroBias.y, state_.gyroBias.z);
}

void IterativeEKF::updateStateIterative(const IMUMeasurement& measurement) {
    EKF_DEBUG_PRINT("\n--- Iterative Update Step ---\n");
    
    bool useMag = (measurement.mag.norm() > 0.1f);
    int measDim = useMag ? 6 : 3;
    
    // Measurement noise covariance R
    float R[6][6];
    memset(R, 0, sizeof(R));
    
    float accel_var = config_.accelNoise * config_.accelNoise;
    for (int i = 0; i < 3; ++i) {
        R[i][i] = accel_var;
    }
    
    if (useMag) {
        float mag_var = config_.magNoise * config_.magNoise;
        for (int i = 3; i < 6; ++i) {
            R[i][i] = mag_var;
        }
    }
    
    // Store initial state for convergence check
    float x_current[7];
    x_current[0] = state_.orientation.w;
    x_current[1] = state_.orientation.x;
    x_current[2] = state_.orientation.y;
    x_current[3] = state_.orientation.z;
    x_current[4] = state_.gyroBias.x;
    x_current[5] = state_.gyroBias.y;
    x_current[6] = state_.gyroBias.z;
    
    // Iterative correction
    int iteration;
    for (iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
        EKF_DEBUG_PRINT("\n  Iteration %d:\n", iteration + 1);
        
        // Compute measurement Jacobian H at current estimate
        float H[6][7];
        computeMeasurementJacobian(state_.orientation, state_.gyroBias, H, useMag);
        
        // Predict measurements at current estimate
        Vector3 g_pred = predictGravity(state_.orientation);
        Vector3 m_pred = useMag ? predictMagnetometer(state_.orientation) : Vector3(0, 0, 0);
        
        // Compute innovation (measurement residual)
        float innovation[6];
        innovation[0] = measurement.accel.x - g_pred.x;
        innovation[1] = measurement.accel.y - g_pred.y;
        innovation[2] = measurement.accel.z - g_pred.z;
        
        if (useMag) {
            innovation[3] = measurement.mag.x - m_pred.x;
            innovation[4] = measurement.mag.y - m_pred.y;
            innovation[5] = measurement.mag.z - m_pred.z;
        }
        
        EKF_DEBUG_PRINT("    Innovation: [%.6f, %.6f, %.6f", 
                        innovation[0], innovation[1], innovation[2]);
        if (useMag) {
            EKF_DEBUG_PRINT(", %.6f, %.6f, %.6f]\n", 
                            innovation[3], innovation[4], innovation[5]);
        } else {
            EKF_DEBUG_PRINT("]\n");
        }
        
        // Compute Kalman gain K = P * H' * (H * P * H' + R)^-1
        float K[7][6];
        computeKalmanGain(H, R, K, measDim);
        
        // Store previous state for convergence check
        float x_prev[7];
        memcpy(x_prev, x_current, sizeof(x_prev));
        
        // Apply state correction
        applyStateCorrection(K, innovation, measDim);
        
        // Update current state array
        x_current[0] = state_.orientation.w;
        x_current[1] = state_.orientation.x;
        x_current[2] = state_.orientation.y;
        x_current[3] = state_.orientation.z;
        x_current[4] = state_.gyroBias.x;
        x_current[5] = state_.gyroBias.y;
        x_current[6] = state_.gyroBias.z;
        
        // Compute convergence metric
        float convergenceMetric = 0.0f;
        float stateCorrection[7];
        for (int i = 0; i < 7; ++i) {
            stateCorrection[i] = x_current[i] - x_prev[i];
            convergenceMetric += stateCorrection[i] * stateCorrection[i];
        }
        convergenceMetric = std::sqrt(convergenceMetric);
        
        printIterationInfo(iteration + 1, stateCorrection, convergenceMetric);
        
        // Check convergence
        if (convergenceMetric < CONVERGENCE_THRESHOLD) {
            EKF_DEBUG_PRINT("    Converged after %d iterations\n", iteration + 1);
            break;
        }
    }
    
    if (iteration == MAX_ITERATIONS) {
        EKF_DEBUG_PRINT("    Maximum iterations reached without convergence\n");
    }
    
    // Final covariance update using last H and K
    // P = (I - K * H) * P using Joseph form for numerical stability
    float H_final[6][7];
    computeMeasurementJacobian(state_.orientation, state_.gyroBias, H_final, useMag);
    
    float K_final[7][6];
    computeKalmanGain(H_final, R, K_final, measDim);
    
    // I - K * H
    float IKH[7][7];
    memset(IKH, 0, sizeof(IKH));
    for (int i = 0; i < 7; ++i) {
        IKH[i][i] = 1.0f;
    }
    
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            for (int k = 0; k < measDim; ++k) {
                IKH[i][j] -= K_final[i][k] * H_final[k][j];
            }
        }
    }
    
    // P = IKH * P * IKH' + K * R * K' (Joseph form)
    float P_temp[7][7];
    
    // P_temp = IKH * P
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            P_temp[i][j] = 0.0f;
            for (int k = 0; k < 7; ++k) {
                P_temp[i][j] += IKH[i][k] * P_.data[k][j];
            }
        }
    }
    
    // P = P_temp * IKH'
    float P_new[7][7];
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            P_new[i][j] = 0.0f;
            for (int k = 0; k < 7; ++k) {
                P_new[i][j] += P_temp[i][k] * IKH[j][k];
            }
        }
    }
    
    // Add K * R * K'
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            for (int k = 0; k < measDim; ++k) {
                for (int l = 0; l < measDim; ++l) {
                    P_new[i][j] += K_final[i][k] * R[k][l] * K_final[j][l];
                }
            }
        }
    }
    
    // Copy back to P_
    memcpy(P_.data, P_new, sizeof(P_.data));
    
    // Ensure symmetry and positive definiteness
    enforceCovarianaceSymmetry();
    
    // Ensure minimum diagonal values
    for (int i = 0; i < 7; ++i) {
        if (P_.data[i][i] < 1e-6f) {
            P_.data[i][i] = 1e-6f;
        }
    }
    
    EKF_DEBUG_PRINT("\nFinal state - q: [%.6f, %.6f, %.6f, %.6f], bias: [%.6f, %.6f, %.6f]\n",
                    state_.orientation.w, state_.orientation.x, state_.orientation.y, state_.orientation.z,
                    state_.gyroBias.x, state_.gyroBias.y, state_.gyroBias.z);
}

Vector3 IterativeEKF::predictGravity(const Quaternion& q) const {
    // Rotate gravity vector from world to body frame
    // Use conjugate().rotate() for inverse rotation
    return q.conjugate().rotate(config_.gravityRef);
}

Vector3 IterativeEKF::predictMagnetometer(const Quaternion& q) const {
    // Rotate magnetic field from world to body frame
    // Use conjugate().rotate() for inverse rotation
    return q.conjugate().rotate(config_.magRef);
}

void IterativeEKF::computeMeasurementJacobian(const Quaternion& q, const Vector3& gyroBias,
                                             float H[6][7], bool useMag) const {
    memset(H, 0, 6 * 7 * sizeof(float));
    
    // Jacobian of gravity measurement w.r.t. quaternion
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
    float g = config_.gravityRef.norm();
    
    // dg/dq0
    H[0][0] = 2.0f * g * q2;
    H[1][0] = -2.0f * g * q3;
    H[2][0] = 0.0f;
    
    // dg/dq1
    H[0][1] = 2.0f * g * q3;
    H[1][1] = 2.0f * g * q0;
    H[2][1] = -4.0f * g * q1;
    
    // dg/dq2
    H[0][2] = 2.0f * g * q0;
    H[1][2] = 2.0f * g * q1;
    H[2][2] = -4.0f * g * q2;
    
    // dg/dq3
    H[0][3] = 2.0f * g * q1;
    H[1][3] = -2.0f * g * q0;
    H[2][3] = 0.0f;
    
    // Gravity measurement doesn't depend on gyro bias
    // H[0-2][4-6] remain 0
    
    if (useMag) {
        // Jacobian of magnetometer measurement w.r.t. quaternion
        float mx = config_.magRef.x;
        float my = config_.magRef.y;
        float mz = config_.magRef.z;
        
        // dm/dq0
        H[3][0] = 2.0f * (q0*mx - q3*my + q2*mz);
        H[4][0] = 2.0f * (q3*mx + q0*my - q1*mz);
        H[5][0] = 2.0f * (-q2*mx + q1*my + q0*mz);
        
        // dm/dq1
        H[3][1] = 2.0f * (q1*mx + q2*my + q3*mz);
        H[4][1] = 2.0f * (q2*mx - q1*my - q0*mz);
        H[5][1] = 2.0f * (q3*mx + q0*my - q1*mz);
        
        // dm/dq2
        H[3][2] = 2.0f * (-q2*mx + q1*my + q0*mz);
        H[4][2] = 2.0f * (q1*mx + q2*my + q3*mz);
        H[5][2] = 2.0f * (-q0*mx + q3*my - q2*mz);
        
        // dm/dq3
        H[3][3] = 2.0f * (-q3*mx - q0*my + q1*mz);
        H[4][3] = 2.0f * (q0*mx - q3*my + q2*mz);
        H[5][3] = 2.0f * (q1*mx + q2*my + q3*mz);
        
        // Magnetometer measurement doesn't depend on gyro bias
        // H[3-5][4-6] remain 0
    }
}

void IterativeEKF::computeKalmanGain(const float H[6][7], const float R[6][6],
                                   float K[7][6], int measDim) const {
    // K = P * H' * (H * P * H' + R)^-1
    
    // First compute P * H'
    float PHt[7][6];
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < measDim; ++j) {
            PHt[i][j] = 0.0f;
            for (int k = 0; k < 7; ++k) {
                PHt[i][j] += P_.data[i][k] * H[j][k];
            }
        }
    }
    
    // Compute S = H * P * H' + R
    float S[6][6];
    for (int i = 0; i < measDim; ++i) {
        for (int j = 0; j < measDim; ++j) {
            S[i][j] = R[i][j];
            for (int k = 0; k < 7; ++k) {
                S[i][j] += H[i][k] * PHt[k][j];
            }
        }
    }
    
    // Invert S
    if (measDim == 3) {
        float S3[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                S3[i][j] = S[i][j];
            }
        }
        
        float S3_inv[3][3];
        if (matrixInverse3x3(S3, S3_inv)) {
            // K = PHt * S_inv
            for (int i = 0; i < 7; ++i) {
                for (int j = 0; j < 3; ++j) {
                    K[i][j] = 0.0f;
                    for (int k = 0; k < 3; ++k) {
                        K[i][j] += PHt[i][k] * S3_inv[k][j];
                    }
                }
            }
        } else {
            EKF_DEBUG_PRINT("Warning: Failed to invert 3x3 innovation covariance\n");
            memset(K, 0, 7 * 6 * sizeof(float));
        }
    } else {
        float S6_inv[6][6];
        if (matrixInverse6x6(S, S6_inv)) {
            // K = PHt * S_inv
            for (int i = 0; i < 7; ++i) {
                for (int j = 0; j < 6; ++j) {
                    K[i][j] = 0.0f;
                    for (int k = 0; k < 6; ++k) {
                        K[i][j] += PHt[i][k] * S6_inv[k][j];
                    }
                }
            }
        } else {
            EKF_DEBUG_PRINT("Warning: Failed to invert 6x6 innovation covariance\n");
            memset(K, 0, 7 * 6 * sizeof(float));
        }
    }
}

bool IterativeEKF::matrixInverse3x3(const float A[3][3], float Ainv[3][3]) const {
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    
    if (std::abs(det) < 1e-9f) {
        return false;
    }
    
    float inv_det = 1.0f / det;
    
    Ainv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
    Ainv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
    Ainv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;
    
    Ainv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
    Ainv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
    Ainv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;
    
    Ainv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
    Ainv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
    Ainv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;
    
    return true;
}

bool IterativeEKF::matrixInverse6x6(const float A[6][6], float Ainv[6][6]) const {
    // For simplicity in embedded systems, use block diagonal approximation
    // assuming accelerometer and magnetometer measurements are uncorrelated
    
    // Upper-left 3x3 block (accelerometer)
    float A_accel[3][3], Ainv_accel[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            A_accel[i][j] = A[i][j];
        }
    }
    
    if (!matrixInverse3x3(A_accel, Ainv_accel)) {
        return false;
    }
    
    // Lower-right 3x3 block (magnetometer)
    float A_mag[3][3], Ainv_mag[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            A_mag[i][j] = A[i+3][j+3];
        }
    }
    
    if (!matrixInverse3x3(A_mag, Ainv_mag)) {
        return false;
    }
    
    // Construct block diagonal inverse
    memset(Ainv, 0, 36 * sizeof(float));
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Ainv[i][j] = Ainv_accel[i][j];
            Ainv[i+3][j+3] = Ainv_mag[i][j];
        }
    }
    
    return true;
}

void IterativeEKF::applyStateCorrection(const float K[7][6], const float innovation[6], int measDim) {
    // Compute state correction: dx = K * innovation
    float dx[7] = {0};
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < measDim; ++j) {
            dx[i] += K[i][j] * innovation[j];
        }
    }
    
    // Apply correction to quaternion
    state_.orientation.w += dx[0];
    state_.orientation.x += dx[1];
    state_.orientation.y += dx[2];
    state_.orientation.z += dx[3];
    
    // Normalize quaternion
    normalizeQuaternion();
    
    // Apply correction to gyro bias
    state_.gyroBias.x += dx[4];
    state_.gyroBias.y += dx[5];
    state_.gyroBias.z += dx[6];
}

void IterativeEKF::normalizeQuaternion() {
    float norm = std::sqrt(state_.orientation.w * state_.orientation.w +
                          state_.orientation.x * state_.orientation.x +
                          state_.orientation.y * state_.orientation.y +
                          state_.orientation.z * state_.orientation.z);
    
    if (norm > 0.0f) {
        float inv_norm = 1.0f / norm;
        state_.orientation.w *= inv_norm;
        state_.orientation.x *= inv_norm;
        state_.orientation.y *= inv_norm;
        state_.orientation.z *= inv_norm;
    }
}

void IterativeEKF::enforceCovarianaceSymmetry() {
    for (int i = 0; i < 7; ++i) {
        for (int j = i + 1; j < 7; ++j) {
            float avg = 0.5f * (P_.data[i][j] + P_.data[j][i]);
            P_.data[i][j] = avg;
            P_.data[j][i] = avg;
        }
    }
}

void IterativeEKF::printIterationInfo(int iteration, const float* stateCorrection, float convergenceMetric) const {
    EKF_DEBUG_PRINT("    State correction: [%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f]\n",
                    stateCorrection[0], stateCorrection[1], stateCorrection[2], stateCorrection[3],
                    stateCorrection[4], stateCorrection[5], stateCorrection[6]);
    EKF_DEBUG_PRINT("    Convergence metric: %.8f\n", convergenceMetric);
}

Quaternion IterativeEKF::getOrientation() const {
    return state_.orientation;
}

Vector3 IterativeEKF::getGyroBias() const {
    return state_.gyroBias;
}

void IterativeEKF::getEulerAngles(float& roll, float& pitch, float& yaw) const {
    // Convert quaternion to Euler angles
    float q0 = state_.orientation.w;
    float q1 = state_.orientation.x;
    float q2 = state_.orientation.y;
    float q3 = state_.orientation.z;
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (std::abs(sinp) >= 1.0f)
        pitch = std::copysign(M_PI / 2.0f, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

bool IterativeEKF::isInitialized() const {
    return initialized_;
}

} // namespace picoEKF