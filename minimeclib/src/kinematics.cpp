#include "minimeclib/kinematics.hpp"

namespace minimeclib {

    MecanumDrive::MecanumDrive(const double w, const double l, const double r)
    {
        this->w = w;
        this->l = l;
        this->r = r;
    }

    WheelSpeeds MecanumDrive::IKin(const double wz, const double vx, const double vy)
    {
        double fl = 1.0/r*((-l-w)*wz+vx-vy);
        double fr = 1.0/r*((l+w)*wz+vx+vy);
        double rr = 1.0/r*((l+w)*wz+vx-vy);
        double rl = 1.0/r*((-l-w)*wz+vx+vy);
        return {fl, fr, rr, rl};
    }

    tf2::Transform MecanumDrive::FKin(const WheelPositions new_positions)
    {
        auto yaw = r/4.0*
                    (-1.0/(l+w)*(new_positions.fl-wheel_positions.fl)+
                      1.0/(l+w)*(new_positions.fr-wheel_positions.fr)+
                      1.0/(l+w)*(new_positions.rr-wheel_positions.rr)-
                      1.0/(l+w)*(new_positions.rl-wheel_positions.rl));
        
        auto dx = r/4.0*
                    ((new_positions.fl-wheel_positions.fl)+
                     (new_positions.fr-wheel_positions.fr)+
                     (new_positions.rr-wheel_positions.rr)+
                     (new_positions.rl-wheel_positions.rl));
        
        auto dy = r/4.0*
                    (-(new_positions.fl-wheel_positions.fl)+
                      (new_positions.fr-wheel_positions.fr)-
                      (new_positions.rr-wheel_positions.rr)+
                      (new_positions.rl-wheel_positions.rl));

        auto translation = tf2::Vector3(dx, dy, 0.0);
        auto rotation = tf2::Quaternion{};
        rotation.setRPY(0.0, 0.0, yaw);
        wheel_positions = new_positions;
        return tf2::Transform(rotation, translation);
    }

    void MecanumDrive::setWheelOffsets(const WheelPositions new_positions)
    {
        wheel_positions = new_positions;
    }

}

