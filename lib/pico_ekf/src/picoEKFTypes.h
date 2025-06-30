#ifndef PICO_EKF_TYPES_H
#define PICO_EKF_TYPES_H

#include <cmath>
#include <cstring>

#ifndef PICO_EKF_DEBUG
#define PICO_EKF_DEBUG 0
#endif

#if PICO_EKF_DEBUG
#include <stdio.h>
#define EKF_DEBUG_PRINT(...) printf("[EKF] " __VA_ARGS__)
#else
#define EKF_DEBUG_PRINT(...)
#endif

namespace picoEKF {

struct Vector3 {
    float x, y, z;
    
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }
    
    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }
    
    Vector3 operator*(float s) const {
        return Vector3(x * s, y * s, z * s);
    }
    
    float dot(const Vector3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }
    
    Vector3 cross(const Vector3& v) const {
        return Vector3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }
    
    float norm() const {
        return sqrtf(x * x + y * y + z * z);
    }
    
    Vector3 normalized() const {
        float n = norm();
        if (n > 0) {
            return *this * (1.0f / n);
        }
        return *this;
    }
};

struct Quaternion {
    float w, x, y, z;
    
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
    
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }
    
    Quaternion operator+(const Quaternion& q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }
    
    Quaternion operator*(float s) const {
        return Quaternion(w * s, x * s, y * s, z * s);
    }
    
    float norm() const {
        return sqrtf(w * w + x * x + y * y + z * z);
    }
    
    Quaternion normalized() const {
        float n = norm();
        if (n > 0) {
            return *this * (1.0f / n);
        }
        return *this;
    }
    
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
    
    Vector3 rotate(const Vector3& v) const;
    
    void toRotationMatrix(float R[3][3]) const;
};

struct Matrix {
    float data[7][7];
    int rows, cols;
    
    Matrix(int r, int c) : rows(r), cols(c) {
        memset(data, 0, sizeof(data));
    }
    
    float& operator()(int i, int j) {
        return data[i][j];
    }
    
    const float& operator()(int i, int j) const {
        return data[i][j];
    }
    
    void setIdentity() {
        memset(data, 0, sizeof(data));  // Clear all values first
        for (int i = 0; i < rows && i < cols; i++) {
            data[i][i] = 1.0f;
        }
    }
};

struct EKFState {
    Quaternion orientation;
    Vector3 gyroBias;
};

struct IMUMeasurement {
    Vector3 gyro;
    Vector3 accel;
    Vector3 mag;
    float dt;
};

struct EKFConfig {
    float gyroNoise;
    float gyroBiasNoise;
    float accelNoise;
    float magNoise;
    
    Vector3 gravityRef;
    Vector3 magRef;
    
    bool useMagnetometer;
    
    EKFConfig() {
        gyroNoise = 0.01f;
        gyroBiasNoise = 0.0001f;
        accelNoise = 0.1f;
        magNoise = 0.1f;
        
        gravityRef = Vector3(0, 0, 9.81f);
        magRef = Vector3(1, 0, 0);
        
        useMagnetometer = true;
    }
};

}

#endif