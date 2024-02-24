#ifndef MINIMEC_KINEMATICS_INCLUDE_GUARD_HPP
#define MINIMEC_KINEMATICS_INCLUDE_GUARD_HPP
/// @file
/// @brief Kinematics for a mecanum wheel base.

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include <iostream>

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

    /// @brief struct representing wheel positions
    struct WheelPositions {
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
            /// @brief keep track of wheel positions between calls
            WheelPositions wheel_positions;

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

            /// @brief Get the relative transform form the previous wheel angles
            /// @param positions - wheel positions (radians)
            /// @return the relative transform
            tf2::Transform FKin(const WheelPositions new_wheel_positions);

            /// @brief Set the initial position of the wheels
            /// @param initial_positions - WheelPositions
            void setWheelOffsets(const WheelPositions initial_positions);


    };



}

#endif