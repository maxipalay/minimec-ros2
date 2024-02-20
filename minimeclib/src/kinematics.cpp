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
        double fl = 1/r*((-l-w)*wz+vx-vy);
        double fr = 1/r*((l+w)*wz+vx+vy);
        double rr = 1/r*((l+w)*wz+vx-vy);
        double rl = 1/r*((-l-w)*wz+vx+vy);
        return {fl, fr, rr, rl};
    }

}

