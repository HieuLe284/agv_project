/**
 * @file math_utils.h
 * @brief Các hàm tiện ích toán học dùng chung cho toàn bộ project AGV_Robot
 */
#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <cmath>

/**
 * @brief Chuẩn hóa góc radian về miền [-π, π]
 * @param a Góc đầu vào [rad]
 * @return Góc đã chuẩn hóa trong [-π, π]
 */
inline double normalizeAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

/**
 * @brief Chuyển đổi quaternion (x, y, z, w) sang góc yaw [rad]
 * @param qx Thành phần x của quaternion
 * @param qy Thành phần y của quaternion
 * @param qz Thành phần z của quaternion
 * @param qw Thành phần w của quaternion
 * @return Góc yaw [rad]
 */
inline double quatToYaw(double qx, double qy, double qz, double qw) {
    return std::atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz));
}

#endif  // MATH_UTILS_H
