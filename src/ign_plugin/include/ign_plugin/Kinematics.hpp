/// @brief Simple Kinematic library
#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using Matrix3d = Eigen::Matrix3d;

/*

  Make sure that DCM(Direction Cosine Matrix) and rotation matrix are different rotation method
  which have transpose relation to each other.
  DCM => vector value rotation
  rotation matrix => axis rotation

  Default Unit
  Angle : [radian]
  quaternion : [1/z]
*/

class Kinematics {
  public:
    // Skew-symmetric cross product matrix
    Matrix3d skew(const Vector3d& w) {
    /**
     * @brief Generates a skew-symmetric matrix from a 3-element vector.
     *
     * This function constructs a 3×3 skew-symmetric matrix from the given
     * 3D vector. The skew-symmetric matrix is useful for cross-product
     * computations in matrix form.
     *
     * @param w A 3-element Eigen::Vector3d representing the input vector.
     * @return A 3×3 Eigen::Matrix3d representing the skew-symmetric matrix.
     */
        Matrix3d cross_mat;
        cross_mat << 0.0, -w.z(), w.y(),
                     w.z(), 0.0, -w.x(),
                    -w.y(), w.x(), 0.0;
        return cross_mat;
    }

    // Rotation matrix around x-axis
    Matrix3d rotm_x(double angle) {
    /**
     * @brief Generates a x-axis rotation matrix from given angle.
     *
     * This function constructs a 3×3 rotation matrix from the given
     * angle. The matrix is for x-axis rotation.
     *
     * @param angle A scalar double value with radian unit.
     * @return A 3×3 Eigen::Matrix3d representing x-axis rotation matrix.
     */
        Matrix3d rot_mat;
        rot_mat << 1.0,    0.0    ,  0.0,
                   0.0, cos(angle), -sin(angle),
                   0.0, sin(angle),  cos(angle);
        return rot_mat;
    }

    // Rotation matrix around y-axis
    Matrix3d rotm_y(double angle) {
    /**
     * @brief Generates a y-axis rotation matrix from given angle.
     *
     * This function constructs a 3×3 rotation matrix from the given
     * angle. The matrix is for y-axis rotation.
     *
     * @param angle A scalar double value with radian unit.
     * @return A 3×3 Eigen::Matrix3d representing y-axis rotation matrix.
     */
        Matrix3d rot_mat;
        rot_mat << cos(angle), 0.0, sin(angle),
                    0.0      , 1.0,        0.0,
                  -sin(angle), 0.0, cos(angle);
        return rot_mat;
    }

    // Rotation matrix around z-axis
    Matrix3d rotm_z(double angle) {
    /**
     * @brief Generates a z-axis rotation matrix from given angle.
     *
     * This function constructs a 3×3 rotation matrix from the given
     * angle. The matrix is for z-axis rotation.
     *
     * @param angle A scalar double value with radian unit.
     * @return A 3×3 Eigen::Matrix3d representing z-axis rotation matrix.
     */
        Matrix3d rot_mat;
        rot_mat << cos(angle), -sin(angle),  0.0,
                   sin(angle), cos(angle) ,  0.0,
                   0.0       ,    0.0     ,  1.0;
        return rot_mat;
    }

    // DCM matrix around x-axis
    Matrix3d dcm_x(double angle) {
    /**
     * @brief Generates a x-axis DCM matrix from given angle.
     *
     * This function constructs a 3×3 rotation matrix from the given
     * angle. The matrix is for x-axis rotation.
     *
     * @param angle A scalar double value with radian unit.
     * @return A 3×3 Eigen::Matrix3d representing x-axis DCM matrix.
     */
        Matrix3d dcm_mat;
        dcm_mat << 1.0,    0.0,        0.0,
                   0.0,   cos(angle), sin(angle),
                   0.0,  -sin(angle), cos(angle);
        return dcm_mat;
    }

    // DCM matrix around y-axis
    Matrix3d dcm_y(double angle) {
    /**
     * @brief Generates a y-axis DCM matrix from given angle.
     *
     * This function constructs a 3×3 rotation matrix from the given
     * angle. The matrix is for y-axis rotation.
     *
     * @param angle A scalar double value with radian unit.
     * @return A 3×3 Eigen::Matrix3d representing y-axis DCM matrix.
     */
        Matrix3d dcm_mat;
        dcm_mat << cos(angle), 0.0, -sin(angle),
                   0.0       , 1.0,  0.0,
                   sin(angle), 0.0,  cos(angle);
        return dcm_mat;
    }

    // DCM matrix around z-axis
    Matrix3d dcm_z(double angle) {
    /**
     * @brief Generates a z-axis DCM matrix from given angle.
     *
     * This function constructs a 3×3 rotation matrix from the given
     * angle. The matrix is for z-axis rotation.
     *
     * @param angle A scalar double value with radian unit.
     * @return A 3×3 Eigen::Matrix3d representing z-axis DCM matrix.
     */
        Matrix3d dcm_mat;
        dcm_mat <<  cos(angle), sin(angle), 0.0,
                   -sin(angle), cos(angle), 0.0,
                        0.0   ,    0.0    , 1.0;
        return dcm_mat;
    }

    // Rodrigues rotation formula
    Matrix3d euler2rotm(double euler_angle, Vector3d euler_vector) {
    /**
     * @brief Generates a rotation matrix from euler angle & vector.
     *
     * This function constructs a 3×3 rotation matrix from the given
     * angle & vector.
     *
     * @param angle        A scalar double value with radian unit.
     * @param euler_vector A Eigen::Vector3d vector
     * @return A 3×3 Eigen::Matrix3d representing rotation matrix.
     */
    euler_vector.normalize(); // Ensure the vector is a unit vector

    double cosine_0 = cos(euler_angle); // cos(theta)
    double sine = sin(euler_angle);     // sin(theta)
    double cosine_1 = 1 - cosine_0;     // (1 - cos(theta))
    Matrix3d skew = Kinematics::skew(euler_vector); // Skew-symmetric matrix

    return cosine_0 * Matrix3d::Identity()
           + cosine_1 * (euler_vector * euler_vector.transpose())
           + sine * skew;
    }

    // Extract Euler angles from a rotation matrix (x-y-z)
    Vector3d rotm2xyz(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts euler angles from rotation matrix in sequence of x-y-z.
     *
     * This function constructs an euler angle vector from the given
     * rotation matrix in sequence of x-y-z.
     *
     * @param rot_mat      A 3×3 Eigen::Matrix3d double value with radian unit.
     * @return             A 1×3 Eigen::Vector3d representing euler angle sequence.
     */
    // Clamp value to valid range for asin
    double eul2 = asin(std::clamp(rot_mat(0, 2), -1.0, 1.0));
    double eul1 = atan2(-rot_mat(1, 2), rot_mat(2, 2));
    double eul3 = atan2(-rot_mat(0, 1), rot_mat(0, 0));

    return Vector3d(eul1, eul2, eul3);
    }

    // Extract Euler angles from a rotation matrix (z-y-x)
    Vector3d rotm2zyx(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts euler angles from rotation matrix in sequence of z-y-x.
     *
     * This function constructs an euler angle vector from the given
     * rotation matrix in sequence of z-y-x.
     *
     * @param rot_mat      A 3×3 Eigen::Matrix3d double value with radian unit.
     * @return             A 1×3 Eigen::Vector3d representing euler angle sequence.
     */
    double eul2 = asin(std::clamp(-rot_mat(0, 2), -1.0, 1.0));
    double eul1 = atan2(rot_mat(1, 2), rot_mat(2, 2));
    double eul3 = atan2(rot_mat(0, 1), rot_mat(0, 0));

    return Vector3d(eul1, eul2, eul3);
    }

    // Extract Euler angles from a rotation matrix (z-x-z)
    Vector3d rotm2zxz(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts euler angles from rotation matrix in sequence of z-x-z.
     *
     * This function constructs an euler angle vector from the given
     * rotation matrix in sequence of z-x-z.
     *
     * @param rot_mat      A 3×3 Eigen::Matrix3d double value with radian unit.
     * @return             A 1×3 Eigen::Vector3d representing euler angle sequence.
     */
    double eul2 = acos(std::clamp(rot_mat(2, 2), -1.0, 1.0));
    double eul1 = atan2(rot_mat(1, 2), rot_mat(0, 2));
    double eul3 = atan2(rot_mat(2, 1), -rot_mat(2, 0));

    return Vector3d(eul1, eul2, eul3);
    }

    // Extract Euler angles from a rotation matrix (z-y-z)
    Vector3d rotm2zyz(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts euler angles from rotation matrix in sequence of z-y-z.
     *
     * This function constructs an euler angle vector from the given
     * rotation matrix in sequence of z-y-z.
     *
     * @param rot_mat      A 3×3 Eigen::Matrix3d double value with radian unit.
     * @return             A 1×3 Eigen::Vector3d representing euler angle sequence.
     */
    double eul2 = acos(std::clamp(rot_mat(2, 2), -1.0, 1.0));
    double eul1 = atan2(rot_mat(1, 2), rot_mat(0, 2));
    double eul3 = atan2(rot_mat(2, 1), -rot_mat(2, 0));

    return Vector3d(eul1, eul2, eul3);
    }

    // Extract Euler angles from a rotation matrix (x-y-x)
    Vector3d rotm2xyx(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts euler angles from rotation matrix in sequence of x-y-x.
     *
     * This function constructs an euler angle vector from the given
     * rotation matrix in sequence of x-y-x.
     *
     * @param rot_mat      A 3×3 Eigen::Matrix3d double value with radian unit.
     * @return             A 1×3 Eigen::Vector3d representing euler angle sequence.
     */
    double eul2 = acos(std::clamp(rot_mat(0, 0), -1.0, 1.0));
    double eul1 = atan2(rot_mat(0, 1), -rot_mat(0, 2));
    double eul3 = atan2(rot_mat(1, 0), rot_mat(2, 0));
    return Vector3d(eul1, eul2, eul3);
    }

    // Extract Euler angles from a rotation matrix (x-z-x)
    Vector3d rotm2xzx(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts euler angles from rotation matrix in sequence of x-z-x.
     *
     * This function constructs an euler angle vector from the given
     * rotation matrix in sequence of x-z-x.
     *
     * @param rot_mat      A 3×3 Eigen::Matrix3d double value with radian unit.
     * @return             A 1×3 Eigen::Vector3d representing euler angle sequence.
     */
    double eul2 = acos(std::clamp(rot_mat(0, 0), -1.0, 1.0));
    double eul1 = atan2(rot_mat(0, 2), rot_mat(0, 1));
    double eul3 = atan2(rot_mat(2, 0), -rot_mat(1, 0));
    return Vector3d(eul1, eul2, eul3);
    }

    // Extract Euler angles from a rotation matrix (y-x-y)
    Vector3d rotm2yxy(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts euler angles from rotation matrix in sequence of y-x-y.
     *
     * This function constructs an euler angle vector from the given
     * rotation matrix in sequence of y-x-y.
     *
     * @param rot_mat      A 3×3 Eigen::Matrix3d double value with radian unit.
     * @return             A 1×3 Eigen::Vector3d representing euler angle sequence.
     */
    double eul2 = acos(std::clamp(rot_mat(1, 1), -1.0, 1.0));
    double eul1 = atan2(rot_mat(1, 0), rot_mat(1, 2));
    double eul3 = atan2(rot_mat(0, 1), -rot_mat(2, 1));
    return Vector3d(eul1, eul2, eul3);
    }

    // Extract Euler angles from a rotation matrix (y-z-y)
    Vector3d rotm2yzy(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts euler angles from rotation matrix in sequence of y-z-y.
     *
     * This function constructs an euler angle vector from the given
     * rotation matrix in sequence of y-z-y.
     *
     * @param rot_mat      A 3×3 Eigen::Matrix3d double value with radian unit.
     * @return             A 1×3 Eigen::Vector3d representing euler angle sequence.
     */
    double eul2 = acos(std::clamp(rot_mat(1, 1), -1.0, 1.0));
    double eul1 = atan2(rot_mat(1, 2), rot_mat(1, 0));
    double eul3 = atan2(rot_mat(2, 1), -rot_mat(0, 1));
    return Vector3d(eul1, eul2, eul3);
    }

    // Extract Euler angles from a rotation matrix (x-z-y)
    Vector3d rotm2xzy(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts euler angles from rotation matrix in sequence of x-z-y.
     *
     * This function constructs an euler angle vector from the given
     * rotation matrix in sequence of x-z-y.
     *
     * @param rot_mat      A 3×3 Eigen::Matrix3d double value with radian unit.
     * @return             A 1×3 Eigen::Vector3d representing euler angle sequence.
     */
    double eul2 = asin(std::clamp(-rot_mat(0, 1), -1.0, 1.0));
    double eul1 = atan2(rot_mat(0, 2), rot_mat(0, 0));
    double eul3 = atan2(rot_mat(2, 1), rot_mat(1, 1));
    return Vector3d(eul1, eul2, eul3);
    }

    // Extract Euler angles from a rotation matrix (y-x-z)
    Vector3d rotm2yxz(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts euler angles from rotation matrix in sequence of y-x-z.
     *
     * This function constructs an euler angle vector from the given
     * rotation matrix in sequence of y-x-z.
     *
     * @param rot_mat      A 3×3 Eigen::Matrix3d double value with radian unit.
     * @return             A 1×3 Eigen::Vector3d representing euler angle sequence.
     */
    double eul2 = asin(std::clamp(-rot_mat(1, 0), -1.0, 1.0));
    double eul1 = atan2(rot_mat(1, 2), rot_mat(1, 1));
    double eul3 = atan2(rot_mat(2, 0), rot_mat(0, 0));
    return Vector3d(eul1, eul2, eul3);
    }

    // Extract Euler angles from a rotation matrix (y-z-x)
    Vector3d rotm2yzx(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts euler angles from rotation matrix in sequence of y-z-x.
     *
     * This function constructs an euler angle vector from the given
     * rotation matrix in sequence of y-z-x.
     *
     * @param rot_mat      A 3×3 Eigen::Matrix3d double value with radian unit.
     * @return             A 1×3 Eigen::Vector3d representing euler angle sequence.
     */
    double eul2 = asin(std::clamp(rot_mat(1, 2), -1.0, 1.0));
    double eul1 = atan2(-rot_mat(1, 0), rot_mat(1, 1));
    double eul3 = atan2(-rot_mat(0, 2), rot_mat(2, 2));
    return Vector3d(eul1, eul2, eul3);
    }

    // Extract Euler angles from a rotation matrix (z-x-y)
    Vector3d rotm2zxy(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts euler angles from rotation matrix in sequence of z-x-y.
     *
     * This function constructs an euler angle vector from the given
     * rotation matrix in sequence of z-x-y.
     *
     * @param rot_mat      A 3×3 Eigen::Matrix3d double value with radian unit.
     * @return             A 1×3 Eigen::Vector3d representing euler angle sequence.
     */
    double eul2 = asin(std::clamp(-rot_mat(2, 0), -1.0, 1.0));
    double eul1 = atan2(rot_mat(1, 0), rot_mat(0, 0));
    double eul3 = atan2(rot_mat(2, 1), rot_mat(2, 2));

    return Vector3d(eul1, eul2, eul3);
    }

    // Normalize a quaternion (Scalar part first: [w, x, y, z])
    Vector4d quat_norm(const Vector4d& q) {
    /**
     * @brief Normalizes a quaternion represented as a 4D vector.
     *
     * If the quaternion has zero norm, it returns
     * the identity quaternion (1, 0, 0, 0).
     *
     * @param q A 4D Eigen::Vector4d representing a quaternion.
     * @return A normalized Eigen::Vector4d quaternion.
     */
        double norm = q.norm();
        if (norm < std::numeric_limits<double>::epsilon()) {
            return Vector4d(1, 0, 0, 0); // Return identity quaternion
        }

        return q / norm;
    }

    // Quaternion conjugate
    Vector4d quat_conj(const Vector4d& q) {
    /**
     * @brief Normalizes a quaternion represented as a 4D vector.
     *
     * If the quaternion has zero norm, it returns
     * the identity quaternion (1, 0, 0, 0).
     *
     * @param q A 4D Eigen::Vector4d representing a quaternion.
     * @return A conjugated Eigen::Vector4d quaternion.
     */
        return Vector4d(q.w(), -q.x(), -q.y(), -q.z());
    }

    // Quaternion product
    Vector4d quat_prod(const Vector4d& q1, const Vector4d& q2) {
    /**
     * @brief Computes the Hamilton product of two quaternions.
     *
     * The function multiplies two quaternions represented as 4D vectors.
     * The quaternion multiplication follows the Hamilton product rule.
     *
     * @param q1 First quaternion (Eigen::Vector4d).
     * @param q2 Second quaternion (Eigen::Vector4d).
     * @return The resulting quaternion (Eigen::Vector4d).
     */
        double w1 = q1(0), w2 = q2(0);   // Scalar parts
        Vector3d v1 = q1.tail<3>(), v2 = q2.tail<3>(); // Vector parts

        double w = w1 * w2 - v1.dot(v2); // Scalar part of the result
        Vector3d v = w1 * v2 + w2 * v1 + v1.cross(v2); // Vector part

        return Vector4d(w, v.x(), v.y(), v.z());
    }

    // Quaternion to Rotation Matrix
    Matrix3d quat2rotm(const Vector4d& q) {
    /**
     * @brief Contructs rotation matrix from quaternion.
     *
     * The function constructs rotation matrix represented as 4D vectors.
     * The quaternion multiplication follows the Hamilton product rule.
     *
     * @param q  quaternion (Eigen::Vector4d).
     * @return   A 3x3 rotaion matrix from quaternion
     */
        double w = q.w();
        double x = q.x();
        double y = q.y();
        double z = q.z();

        Matrix3d rot_mat;
        rot_mat << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
                   2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                   2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y);
        return rot_mat;
    }

    // Rotation Matrix to Quaternion
    Vector4d rotm2quat(const Matrix3d& rot_mat) {
    /**
     * @brief Extracts quaternion from rotation matrix.
     *
     * The function extracts a quaternion from rotation matrix.
     *
     * @param rot_mat  A 3x3 Eigen::Matrix3d rotation matrix
     * @return         A 1x4 Eigen::Vector4d quaternion
     */
        double trace = rot_mat.trace();
        Vector4d q;
        if (trace > 0.0) {
            double s = 0.5 / sqrt(trace + 1.0);
            q.w() = 0.25 / s;
            q.x() = (rot_mat(2, 1) - rot_mat(1, 2)) * s;
            q.y() = (rot_mat(0, 2) - rot_mat(2, 0)) * s;
            q.z() = (rot_mat(1, 0) - rot_mat(0, 1)) * s;
        } else {
            if (rot_mat(0, 0) > rot_mat(1, 1) && rot_mat(0, 0) > rot_mat(2, 2)) {
                double s = 2.0 * sqrt(1.0 + rot_mat(0, 0) - rot_mat(1, 1) - rot_mat(2, 2));
                q.w() = (rot_mat(2, 1) - rot_mat(1, 2)) / s;
                q.x() = 0.25 * s;
                q.y() = (rot_mat(0, 1) + rot_mat(1, 0)) / s;
                q.z() = (rot_mat(0, 2) + rot_mat(2, 0)) / s;
            } else if (rot_mat(1, 1) > rot_mat(2, 2)) {
                double s = 2.0 * sqrt(1.0 + rot_mat(1, 1) - rot_mat(0, 0) - rot_mat(2, 2));
                q.w() = (rot_mat(0, 2) - rot_mat(2, 0)) / s;
                q.x() = (rot_mat(0, 1) + rot_mat(1, 0)) / s;
                q.y() = 0.25 * s;
                q.z() = (rot_mat(1, 2) + rot_mat(2, 1)) / s;
            } else {
                double s = 2.0 * sqrt(1.0 + rot_mat(2, 2) - rot_mat(0, 0) - rot_mat(1, 1));
                q.w() = (rot_mat(1, 0) - rot_mat(0, 1)) / s;
                q.x() = (rot_mat(0, 2) + rot_mat(2, 0)) / s;
                q.y() = (rot_mat(1, 2) + rot_mat(2, 1)) / s;
                q.z() = 0.25 * s;
            }
        }
        return q;
    }

    // Euler Angles from Quaternion
    Vector3d quat2euler(const Vector4d& q) {
    /**
     * @brief Extracts RPY from quaternion.
     *
     * The function extracts roll/pitch/yaw from quaternion.
     *
     * @param q  A 1x4 Eigen::Vector4d quaternion
     * @return   A 1x3 Eigen::Vector3d RPY vector
     */
        double w = q.w();
        double x = q.x();
        double y = q.y();
        double z = q.z();

        double roll = atan2(2.0 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        double pitch = asin(2.0 * (w * y - z * x));
        double yaw = atan2(2.0 * (w * z + x * y), 1 - 2 * (y * y + z * z));

        return Vector3d(roll, pitch, yaw);
    }

    // Quaternion difference Calculation
    Vector4d quat_diff(const Vector4d& q1, const Vector4d& q2) {
    /**
     * @brief Computes quaternion difference between given two quaternions.
     *
     * This function calculates quaternion difference.
     *
     * @param q1 The quaternion (Eigen::Vector4d).
     * @param q2 The quaternion (Eigen::Vector4d).
     * @return   A 1x4 Eigen::Vector4d quaternion difference.
     */
        Vector4d q_err = quat_prod(quat_conj(q1), q2);
        return q_err;
    }

    Vector4d quat_dot(const Vector4d& quat, const Vector3d& w) {
    /**
     * @brief Computes the time derivative of a quaternion given angular velocity.
     *
     * This function calculates `q_dot = 0.5 * (q ⊗ w)`, where `⊗` represents quaternion-vector multiplication.
     *
     * @param quat The quaternion (Eigen::Vector4d).
     * @param w The angular velocity vector (Eigen::Vector3d).
     * @return The quaternion derivative (Eigen::Vector4d).
     */
        Vector3d qv = quat.tail<3>(); // Extract vector part
        double qs = -0.5 * qv.dot(w); // Scalar part derivative
        Vector3d qv_dot = 0.5 * (quat(0) * w + qv.cross(w)); // Vector part derivative

        return Vector4d(qs, qv_dot.x(), qv_dot.y(), qv_dot.z());
    }

    // Runge-Kutta 4th Order Integration for Quaternion Update
    Vector4d quat_update(const Vector4d& qd, const Vector3d& w, double dt, double tol = 1e-6) {
    /**
     * @brief Computes the integration of a quaternion derivative.
     *
     * This function calculates integration of quaternion derivation.
     *
     * @param qd The quaternion derivative   (Eigen::Vector4d).
     * @param w The angular velocity vector  (Eigen::Vector3d).
     * @param dt a time step for integration (double).
     * @return  integrated quaternion        (Eigen::Vector4d).
     */
        auto quatDeriv = [&](const Vector4d& q, const Vector3d& w) {
            return quat_dot(qd,w);
        };

        Vector4d k1 = quatDeriv(qd, w);
        Vector4d k2 = quatDeriv(qd + 0.5 * dt * k1, w);
        Vector4d k3 = quatDeriv(qd + 0.5 * dt * k2, w);
        Vector4d k4 = quatDeriv(qd + dt * k3, w);

        Vector4d q_next = qd + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

        // Error estimation for adaptive step size
        double error = (k1 - k4).norm();
        if (error > tol) {
            dt *= 0.5;
            return quat_update(qd, w, dt, tol);
        } else if (error < tol / 10) {
            dt *= 2.0;
        }

        return quat_norm(q_next);
    }
};
#endif // KINEMATICS_HPP_
