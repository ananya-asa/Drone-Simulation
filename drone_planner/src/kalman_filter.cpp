#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {
    x_.setZero();
    P_.setIdentity();

    // Default dt (can be changed in init)
    dt_ = 0.05f; // 20 Hz

    // Constant-velocity model
    F_.setIdentity();
    F_(0, 3) = dt_;
    F_(1, 4) = dt_;
    F_(2, 5) = dt_;

    // Measure position only: z = [x, y, z]^T
    H_.setZero();
    H_(0, 0) = 1.0f;
    H_(1, 1) = 1.0f;
    H_(2, 2) = 1.0f;

    // Process noise (tune these)
    Q_.setZero();
    Q_(0, 0) = 0.01f;
    Q_(1, 1) = 0.01f;
    Q_(2, 2) = 0.01f;
    Q_(3, 3) = 0.1f;
    Q_(4, 4) = 0.1f;
    Q_(5, 5) = 0.1f;

    // Measurement noise (position noise)
    R_.setZero();
    R_(0, 0) = 0.05f; // x noise
    R_(1, 1) = 0.05f; // y noise
    R_(2, 2) = 0.05f; // z noise
}

void KalmanFilter::init(const Eigen::Vector3f& pos, float dt_seconds) {
    dt_ = dt_seconds;

    // Rebuild F with new dt
    F_.setIdentity();
    F_(0, 3) = dt_;
    F_(1, 4) = dt_;
    F_(2, 5) = dt_;

    // Initial state: position from measurement, velocity = 0
    x_.setZero();
    x_(0) = pos.x();
    x_(1) = pos.y();
    x_(2) = pos.z();

    // Large initial uncertainty in velocity
    P_.setZero();
    P_(0, 0) = 0.1f;
    P_(1, 1) = 0.1f;
    P_(2, 2) = 0.1f;
    P_(3, 3) = 10.0f;
    P_(4, 4) = 10.0f;
    P_(5, 5) = 10.0f;

    initialized_ = true;
}

void KalmanFilter::predict() {
    if (!initialized_) return;

    // x = F x
    x_ = F_ * x_;

    // P = F P F^T + Q
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::Vector3f& meas) {
    if (!initialized_) {
        init(meas, dt_);
        return;
    }

    Eigen::Matrix<float, MEAS_DIM, 1> z;
    z << meas.x(), meas.y(), meas.z();

    // Innovation: y = z - H x
    Eigen::Matrix<float, MEAS_DIM, 1> y = z - H_ * x_;

    // Innovation covariance: S = H P H^T + R
    Eigen::Matrix<float, MEAS_DIM, MEAS_DIM> S =
        H_ * P_ * H_.transpose() + R_;

    // Kalman gain: K = P H^T S^-1
    Eigen::Matrix<float, STATE_DIM, MEAS_DIM> K =
        P_ * H_.transpose() * S.inverse();

    // Update state: x = x + K y
    x_ = x_ + K * y;

    // Update covariance: P = (I - K H) P
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> I;
    I.setIdentity();
    P_ = (I - K * H_) * P_;
}
