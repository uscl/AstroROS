#ifndef ORBIT_MECHANICS_HPP_
#define ORBIT_MECHANICS_HPP_

#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <filesystem>

#include "ign_plugin/Kinematics.hpp"


// const auto package_share_path = ament_index_cpp::get_package_share_directory("pinocchio_ros_example");
// const auto urdf_path = std::filesystem::path(package_share_path) / "ur_robot_model" / "ur5_gripper.urdf";
// const auto srdf_path = std::filesystem::path(package_share_path) / "ur_robot_model" / "ur5_gripper.srdf";

using namespace std;
using namespace Eigen;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using Matrix3d = Eigen::Matrix3d;

typedef Matrix<double, 6, 1> Vector6d;

class OrbitMechanics {
public:
    // Constructor & Deconstructor
    OrbitMechanics(double semi,   double eccen,   double incli,
                   double ascend, double perigee, double anomaly,
                   int year, int month,  int day,
                   int hour, int minute, int second)
    {
        semi_   = semi; eccen_  = eccen;
        perigee_ = perigee * deg2rad_; ascend_  = ascend * deg2rad_;
        incli_  = incli * deg2rad_;    anomaly_ = anomaly * deg2rad_;
        keplerian_ << semi_, eccen_, incli_, ascend_, perigee_, anomaly_;

        year_ = year; month_ = month;   day_ = day;
        hour_ = hour; minute_ = minute; second_ = second;
        time_ << year, month, day, hour, minute, second;

        cartestian_state_ = kepel2state(keplerian_);
        // cout << "cartesian_state : " << cartestian_state_ << endl;
        x_ = cartestian_state_(0);  y_ = cartestian_state_(1);  z_ = cartestian_state_(2);
        vx_ = cartestian_state_(3); vy_ = cartestian_state_(4); vz_ = cartestian_state_(5);
    }

    ~OrbitMechanics() {}

    // Time update
    // Time update code will be added. TBD

    // Orbit propagation
    pair<Vector6d, Matrix3d> calcOrbitPropagation (const Vector6d& initial_condition,
                                                   Vector3d& sat_accel)
    {
        Vector6d dfdt;
        Matrix3d eci2lvlh;

        double x  = initial_condition(0);
        double y  = initial_condition(1);
        double z  = initial_condition(2);
        double x1 = initial_condition(3);
        double y1 = initial_condition(4);
        double z1 = initial_condition(5);

        double r = sqrt(x*x + y*y + z*z);

        // J2 Perturbation
        double term1 = 3 * J2_ * u_ * re_ * re_ * x;
        double term2 = 2 * pow(r, 5);
        double term3 = (1 - ((5 * z * z) / (r * r)));
        double J2_a_I = -(term1 / term2) * term3;
        term1 = 3 * J2_ * u_ * re_ * re_ * y;
        double J2_a_J = -(term1 / term2) * term3;
        term1 = 3 * J2_ * u_ * re_ * re_ * z;
        term3 = (3 - ((5 * z * z) / (r * r)));
        double J2_a_K = -(term1 / term2) * term3;
        Vector3d J2_vec(J2_a_I, J2_a_J, J2_a_K);

        // J4 Perturbation
        term1 = (15 * J4_ * u_ * re_ * re_ * re_ * re_ * x) / (8 * pow(r, 7));
        term2 = (14 * z * z) / (r * r);
        term3 = (21 * pow(z, 4)) / (pow(r, 4));
        double J4_a_I = term1 * (1 - term2 + term3);
        term1 = (15 * J4_ * u_ * re_ * re_ * re_ * re_ * y) / (8 * pow(r, 7));
        double J4_a_J = term1 * (1 - term2 + term3);
        term1 = (15 * J4_ * u_ * re_ * re_ * re_ * re_ * z) / (8 * pow(r, 7));
        term2 = (70 * z * z) / (3 * r * r);
        term3 = (21 * pow(z, 4)) / (pow(r, 4));
        double J4_a_K = term1 * (5 - term2 + term3);
        Vector3d J4_vec(J4_a_I, J4_a_J, J4_a_K);

        // Kepler Motion Equations
        dfdt(0) = x1;
        dfdt(1) = y1;
        dfdt(2) = z1;
        dfdt(3) = -u_ * x / pow(r, 3) + sat_accel(0);
        dfdt(4) = -u_ * y / pow(r, 3) + sat_accel(1);
        dfdt(5) = -u_ * z / pow(r, 3) + sat_accel(2);
        // dfdt(3) = -u_ * x / pow(r, 3) + sat_accel(0)
        //           +J2_vec(0) + J2_vec(0);
        // dfdt(4) = -u_ * y / pow(r, 3) + sat_accel(1)
        //           + J2_vec(1) + J2_vec(1);
        // dfdt(5) = -u_ * z / pow(r, 3) + sat_accel(2)
        //           + J2_vec(2) + J2_vec(2);

        // LVLH Frame Calculation
        Vector3d lvlh_z = -Vector3d(x, y, z).normalized();
        Vector3d rcv(x1, y1, z1);
        Vector3d lvlh_x = rcv.normalized();
        Vector3d lvlh_y = lvlh_z.cross(lvlh_x);

        eci2lvlh.col(0) = lvlh_x;
        eci2lvlh.col(1) = lvlh_y;
        eci2lvlh.col(2) = lvlh_z;

        return make_pair(dfdt, eci2lvlh);
    }

    Vector6d derivativeFunction(const Vector6d& state, Vector3d& sat_accel)
    {
        Vector6d tmp_state = state;
        auto result = calcOrbitPropagation(tmp_state, sat_accel);
        // result.first is the Vector6d derivative
        return result.first;
    }

    Vector6d integrateStateRK4(const Vector6d& current_state,
                               double dt,
                               Vector3d& sat_accel)
    {
        Vector6d temp_state = current_state;
        // k1
        Vector6d k1 = derivativeFunction(temp_state, sat_accel);
        // k2
        Vector6d state_k2 = temp_state + 0.5 * dt * k1;
        Vector6d k2       = derivativeFunction(state_k2, sat_accel);
        // k3
        Vector6d state_k3 = temp_state + 0.5 * dt * k2;
        Vector6d k3       = derivativeFunction(state_k3, sat_accel);
        // k4
        Vector6d state_k4 = temp_state + dt * k3;
        Vector6d k4       = derivativeFunction(state_k4, sat_accel);
        // next_state
        Vector6d next_state = temp_state
                              + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        x_ = next_state(0); y_ = next_state(1); z_ = next_state(2);
        vx_ = next_state(3); vy_ = next_state(4); vz_ = next_state(5);
        return next_state;
    }

    // Orbit state descriptor
    double findKepler(double mean_anomaly, double eccentricity)
    {
        double exc2 = eccentricity * eccentricity;
        double am1 = fmod(mean_anomaly, 2*M_PI);
        double am2 = am1 + am1;
        double am3 = am2 + am1;
        double shoot = am1 + eccentricity*(1.0 - 0.125*exc2)*sin(am1) +
                       0.5*exc2*(sin(am2) + 0.75*eccentricity*sin(am3));
        shoot = fmod(shoot, 2*M_PI);

        double e1 = 1.0;
        double ic = 0.0;

        while ((abs(e1) > 1.e-12) && (ic <= 10)){
            e1 = (shoot - am1 - eccentricity*sin(shoot)) /
                 (1.0 -eccentricity*cos(shoot));
            shoot = shoot - e1;
            ic = ic + 1;
        }

        if (ic >= 10){
            cout << "[WARN] subroutine kepler did not converge in 10 iteration" << endl;
        }

        return shoot; // eccentric
    }

    Vector6d kepel2state(Vector6d& kepel)
    {
        double a = kepel(0);
        double exc = kepel(1);

        double c1 = sqrt(1.0 - exc*exc);

        Matrix3d orb2iner = kin_.rotm_z(-kepel(3)) *
                            kin_.rotm_x(-kepel(2)) *
                            kin_.rotm_z(-kepel(4));

        double E = findKepler(kepel(5), exc);

        double sE = sin(E);
        double cE = cos(E);
        double c3 = sqrt(u_/a) / (1.0 -exc*cE);

        Vector3d x(a * (cE - exc), a*c1*sE, 0.0);
        x = orb2iner.transpose() * x;
        Vector3d v(-c3*sE, c1*c3*cE, 0.0);
        v = orb2iner.transpose() * v;

        Vector6d statevec(x(0),x(1),x(2),v(0),v(1),v(2));

        return statevec;
    }

    Vector6d state2kepel(Vector6d& statevec){
        Vector3d xp;
        xp << statevec(0), statevec(1), statevec(2);
        Vector3d xv;
        xv << statevec(3), statevec(4), statevec(5);

        double r = xp.norm();
        double vq = xv.squaredNorm();
        double ainv = 2.0 / r - vq / u_;

        Vector3d h = kin_.skew(xp) * xv;
        double hm = h.norm();
        Vector3d an(0.0, 0.0, 0.0);

        if (hm < 1.e-10){
            clog << "[WARN] Subroutine kepler did not converge in 10 iterations." << endl;
            return Vector6d::Zero();
        }
        h = h / hm;
        double incl = acos(h(2));
        double raan = atan2(h(0), -h(1));
        double d = xp.dot(xv) / u_;
        double esene = d * sqrt(u_ * ainv);
        double ecose = 1 - r * ainv;
        double exc = sqrt(esene*esene + ecose*ecose);
        double E = atan2(esene, ecose);
        double mean = fmod(E - esene, 2 * M_PI);

        if (mean < 0) mean = mean + 2 * M_PI;

        double arpe = 0.0;
        if (exc < 1.e-10) {
            arpe = 0.0;
        }
        else{
            double dp = 1.0/r - ainv;
            Vector3d ev = dp*xp - d*xv;
            double abev = ev.norm();
            ev = ev / abev;
            an(0) = cos(raan);
            an(1) = sin(raan);
            an(2) = 0;
            Vector3d tmp = kin_.skew(h) * an;
            double fi = ev.dot(tmp);
            double arpe = acos(ev.dot(an));
            if (fi < 0){
                arpe = -arpe + 2 * M_PI;
            }
        }
        Vector6d kepel;
        kepel << 1.0 / ainv, exc, incl, raan, arpe, mean;

        return kepel;
    }

    Vector6d getKeplerian(){
        return Vector6d(semi_, eccen_, incli_, ascend_, perigee_, anomaly_);
    }

    Vector6d getCartesian(){
        return Vector6d(x_, y_, z_,vx_, vy_, vz_);
    }

private:
    Kinematics kin_;
    // Constant parameter
    double rad2deg_ = 180.0 / M_PI;
    double deg2rad_ = M_PI / 180.0;
    double u_ = 3.9865e14;
    double re_ = 6378137.0;
    double J2_ = 0.0010826267;
    double J4_ = -0.0000016196;

    // day time parameter
    Vector6d time_;
    int year_;
    int month_;
    int day_;
    int hour_;
    int minute_;
    int second_;

    // keplerian element
    Vector6d keplerian_;
    double semi_;    // semi-major axis
    double eccen_;   // eccentricity
    double incli_;   // inclination
    double ascend_;  // ascending node
    double perigee_; // perigee
    double anomaly_; // mean anomaly

    // cartesian element
    Vector6d cartestian_state_;
    double x_;
    double y_;
    double z_;
    double vx_;
    double vy_;
    double vz_;
};
#endif // ORBIT_MECHANICS_HPP_
