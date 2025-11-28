#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    // State: [x, y, z, vx, vy, vz]^T
    static constexpr int STATE_DIM = 6;
    static constexpr int MEAS_DIM  = 3;

    KalmanFilter();

    // Initialize with first position measurement
    void init(const Eigen::Vector3f& pos, float dt_seconds);

    // Predict step (called at fixed dt)
    void predict();

    // Update with position measurement (x, y, z)
    void update(const Eigen::Vector3f& meas);

    // Get full state
    Eigen::Matrix<float, STATE_DIM, 1> getState() const { return x_; }

    // Get just position
    Eigen::Vector3f getPosition() const {
        return Eigen::Vector3f(x_(0), x_(1), x_(2));
    }

    // Get just velocity
    Eigen::Vector3f getVelocity() const {
        return Eigen::Vector3f(x_(3), x_(4), x_(5));
    }

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    float dt_ = 0.05f; // 20 Hz default

    // State vector and covariance
    Eigen::Matrix<float, STATE_DIM, 1> x_;
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> P_;

    // System matrices
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> F_; // State transition
    Eigen::Matrix<float, MEAS_DIM, STATE_DIM>  H_; // Measurement
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> Q_; // Process noise
    Eigen::Matrix<float, MEAS_DIM, MEAS_DIM>   R_; // Measurement noise
};

#endif // KALMAN_FILTER_H
