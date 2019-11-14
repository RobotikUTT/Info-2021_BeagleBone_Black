#include "pathfinder/pos_convertor.h"

using namespace std;

Pose2D PosConvertor::internalPose(Pose2D rosPos) const {
    Pose2D res;
    res.x = internalX(rosPos.x);
    res.y = internalY(rosPos.y);

    return res;
}


Pose2D PosConvertor::externalPose(Pose2D mapPos) const{
    Pose2D res;
    res.x = externalX(mapPos.x);
    res.y = externalY(mapPos.y);
    return res;
}

double PosConvertor::internalX(double externalX) const {
    this->assertReady();
    return std::ceil(externalX * _sizeMap.x / _sizeRos.x);
}

double PosConvertor::internalY(double externalY) const {
    this->assertReady();
    return std::ceil(externalY * _sizeMap.y / _sizeRos.y);
}

double PosConvertor::externalX(double externalX) const {
    this->assertReady();
    return externalX * _sizeRos.x / _sizeMap.x;
}

double PosConvertor::externalY(double externalY) const {
    this->assertReady();
    return externalY * _sizeRos.y / _sizeMap.y;
}

double PosConvertor::externalDistance(double internalDist) const {
    // We assume that the scale on x and y is the same, we take the linear average to have a better precision.
    double coef = (externalX(1) + externalY(1)) / 2;
    return internalDist * coef;
}

double PosConvertor::internalDistance(double externalDist) const {
    // We assume that the scale on x and y is the same, we take the linear average to have a better precision.
    double coef = (internalX(1) + internalY(1)) / 2;
    return externalDist * coef;
}

void PosConvertor::assertReady() const {
    if (_sizeMap.x == 0 || _sizeMap.y == 0 || _sizeRos.x == 0 || 0 == _sizeRos.y) {
        throw ConversionException();
    }
}
