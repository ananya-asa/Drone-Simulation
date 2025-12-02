// ============================================================================
// FILE: src/kalman_filter.cpp
// VERSION: 6.0.1 (FIXED - NO REDEFINITION ERRORS)
// FIX: Removed duplicate function definitions (inline in header already)
// ============================================================================

#include "kalman_filter.h"
#include <iostream>
#include <cmath>

// ============================================================================
// KALMAN FILTER CONSTRUCTOR
// ============================================================================
KalmanFilter::KalmanFilter() {
    x_.setZero();
    P_.setIdentity();

    dt_ = 0.05f;  // 20 Hz

    // Motion Model Matrix (F_)
    F_.setIdentity();
    F_(0, 3) = dt_;
    F_(1, 4) = dt_;
    F_(2, 5) = dt_;

    // Measurement Model Matrix (H_)
    H_.setZero();
    H_(0, 0) = 1.0f;
    H_(1, 1) = 1.0f;
    H_(2, 2) = 1.0f;

    // Process Noise Covariance (Q_)
    Q_.setZero();
    Q_(0, 0) = 0.01f;   // Position noise (0.1m std)
    Q_(1, 1) = 0.01f;
    Q_(2, 2) = 0.01f;
    Q_(3, 3) = 0.1f;    // Velocity noise (0.316 m/s std)
    Q_(4, 4) = 0.1f;
    Q_(5, 5) = 0.1f;

    // Measurement Noise Covariance (R_)
    R_.setZero();
    R_(0, 0) = 0.0025f;  // LIDAR 5cm std → variance = 0.0025
    R_(1, 1) = 0.0025f;
    R_(2, 2) = 0.0025f;

    std::cout << "[KALMAN] ✅ Kalman Filter constructed" << std::endl;
}

// ============================================================================
// INITIALIZATION FUNCTION
// ============================================================================
void KalmanFilter::init(const Eigen::Vector3f& pos, float dt_seconds) {
    dt_ = dt_seconds;

    // Rebuild F with updated dt
    F_.setIdentity();
    F_(0, 3) = dt_;
    F_(1, 4) = dt_;
    F_(2, 5) = dt_;

    std::cout << "[KALMAN] Reinitializing with dt=" << dt_ << "s" << std::endl;

    // Initialize state vector
    x_.setZero();
    x_(0) = pos.x();
    x_(1) = pos.y();
    x_(2) = pos.z();
    x_(3) = 0.0f;  // Velocity unknown initially
    x_(4) = 0.0f;
    x_(5) = 0.0f;

    std::cout << "[KALMAN] Initial position: " << pos.transpose() << std::endl;

    // Initialize state covariance
    P_.setZero();
    P_(0, 0) = 0.1f;   // Position covariance
    P_(1, 1) = 0.1f;
    P_(2, 2) = 0.1f;
    P_(3, 3) = 10.0f;  // Velocity covariance (very large - unknown)
    P_(4, 4) = 10.0f;
    P_(5, 5) = 10.0f;

    std::cout << "[KALMAN] ✅ Filter initialized" << std::endl;

    initialized_ = true;
}

// ============================================================================
// PREDICT STEP (Time Update)
// ============================================================================
void KalmanFilter::predict() {
    if (!initialized_) {
        std::cerr << "[KALMAN] ❌ Filter not initialized! Call init() first." << std::endl;
        return;
    }

    // State prediction: x = F * x
    x_ = F_ * x_;

    // Covariance prediction: P = F * P * F^T + Q
    P_ = F_ * P_ * F_.transpose() + Q_;
}

// ============================================================================
// UPDATE STEP (Measurement Update)
// ============================================================================
void KalmanFilter::update(const Eigen::Vector3f& meas) {
    if (!initialized_) {
        std::cout << "[KALMAN] ⚠️  Not initialized, initializing from first measurement" << std::endl;
        init(meas, dt_);
        return;
    }

    // Measurement vector
    Eigen::Matrix<float, MEAS_DIM, 1> z;
    z << meas.x(), meas.y(), meas.z();

    // Innovation (measurement residual): y = z - H*x
    Eigen::Matrix<float, MEAS_DIM, 1> y = z - H_ * x_;

    // Innovation covariance: S = H*P*H^T + R
    Eigen::Matrix<float, MEAS_DIM, MEAS_DIM> S =
        H_ * P_ * H_.transpose() + R_;

    // Kalman gain: K = P*H^T * S^-1
    Eigen::Matrix<float, STATE_DIM, MEAS_DIM> K =
        P_ * H_.transpose() * S.inverse();

    // State correction: x = x + K*y
    x_ = x_ + K * y;

    // Covariance correction: P = (I - K*H)*P
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> I;
    I.setIdentity();
    P_ = (I - K * H_) * P_;
}

// ============================================================================
// NOTE: getState(), getPosition(), getVelocity(), isInitialized()
// are defined INLINE in kalman_filter.h
// Do NOT redefine them here - that causes linker errors!
// ============================================================================