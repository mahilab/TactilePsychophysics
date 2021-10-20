#include "Util/ForceTorqueCentroid.hpp"

using namespace mahi::robo;
using Eigen::Vector3d;

namespace ContactMechanics{

FTC::FTC(const double* ch0, const double* ch1, const double* ch2, const double* ch3,
const double* ch4, const double* ch5, const std::string& filepath):
ati(ch0, ch1, ch2, ch3, ch4, ch5, filepath)
{
    withAti = 1;
    FTC::update();
}

FTC::FTC(std::vector<double> forces, std::vector<double> torques){
    withAti = 0;
    f << forces[0], forces[1], forces[2];
    m << torques[0], torques[1], torques[2];
    FTC::update();
}

std::vector<double> FTC::getCentroid(){
    FTC::update();
    std::vector<double> C(c.data(), c.data() + c.size());
    return C;
}

// private

void FTC::update(){
    if(withAti){
        std::vector<double> forces = ati.get_forces();
        std::vector<double> torques = ati.get_torques();

        f << forces[0], forces[1], forces[2];
        m << torques[0], torques[1], torques[2];
    }

    FTC::setSigmaPrime();
    FTC::setK();
    FTC::setCentroid();    
}

void FTC::setSigmaPrime(){
    sigPrime = m.squaredNorm() - pow(R,2)*f.squaredNorm();
}

void FTC::setK(){
    K = (-sgn(f.transpose()*m))*sqrt(sigPrime + sqrt(pow(sigPrime,2) + 4*pow(R,2)*pow(f.transpose()*m,2)))/(sqrt(2)*R);
}

void FTC::setCentroid(){
    c = (pow(K,2)*m + K*f.cross(m) + (f.transpose()*m)*f)/(K*(pow(K,2) + f.squaredNorm()));
}

double FTC::sgn(double val) {
    return (0.0 < val) - (val < 0.0);
}

}
