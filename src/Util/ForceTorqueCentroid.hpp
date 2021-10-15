#pragma once
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <Mahi/Robo.hpp>

using namespace mahi::robo;
using Eigen::Vector3d;

namespace ContactMechanics{

class FTC {
public:

    FTC(const double* ch0, const double* ch1, const double* ch2, const double* ch3, 
    const double* ch4, const double* ch5, const std::string& filepath);

    FTC(std::vector<double> forces, std::vector<double> torques);

    ~FTC(){};

    std::vector<double> getCentroid();

private:

    void update();

    void setSigmaPrime();

    void setK();

    void setCentroid();

    double sgn(double val);

private:
    Eigen::Vector3d f;
    Eigen::Vector3d m;
    Eigen::Vector3d c;

    double K;
    double sigPrime;
    double R = 30; // [mm] - radius of the sphere

    bool withAti;

    AtiSensor ati;


}; // class
} // namespace
