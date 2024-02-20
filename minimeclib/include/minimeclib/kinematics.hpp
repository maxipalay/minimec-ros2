#ifndef MINIMEC_KINEMATICS_INCLUDE_GUARD_HPP
#define MINIMEC_KINEMATICS_INCLUDE_GUARD_HPP
/// @file
/// @brief Kinematics for a mecanum wheel base.

namespace minimeclib {

    /// @brief struct containing the wheel speeds
    struct WheelSpeeds {
        /// @brief front-left
        double fl{};
        /// @brief front-right
        double fr{};
        /// @brief rear-right
        double rr{};
        /// @brief rear-left
        double rl{};
    };

    class MecanumDrive {
        private:
            /// @brief distance between center of wheels (left/right)/2 (meters)
            double w;
            /// @brief distance between front and rear axes/2 (meters)
            double l;
            /// @brief wheel radius (meters)
            double r;

        public:
            /// @brief default constructor
            MecanumDrive(){};

            /// @brief constructor
            /// @param w - distance between center of wheels (width)/2
            /// @param l - distance between fron and rear axles/2 
            /// @param r - wheel radius
            MecanumDrive(const double w, const double l, const double r);

            /// @brief Get wheel speeds to achieve desired twist
            /// @param wz - twist rotational z
            /// @param vx - twist x velocity
            /// @param vy - twist y velocity
            /// @return wheel speeds
            WheelSpeeds IKin(const double wz, const double vx, const double vy);
    };



}

#endif