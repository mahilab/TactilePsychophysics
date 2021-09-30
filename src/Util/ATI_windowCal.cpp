#include <Util/ATI_windowCal.hpp>
#include <fstream>

using namespace mahi::util;
using namespace mahi::robo;

AtiWindowCal::AtiWindowCal()
{
    FTbias_.fill(0);
    FTbSTG_.fill(0);
}

AtiWindowCal::AtiWindowCal(const double* ch0, const double* ch1, const double* ch2, const double* ch3,
                     const double* ch4, const double* ch5, const std::string& filepath)
{
    set_channels(ch0, ch1, ch2, ch3, ch4, ch5);
    load_calibration(filepath);
}

void AtiWindowCal::set_channels(const double* ch0, const double* ch1, const double* ch2,
                             const double* ch3, const double* ch4, const double* ch5) {
    m_ati.set_channels(ch0, ch1, ch2, ch3, ch4, ch5);
}

bool AtiWindowCal::load_calibration(const std::string& filepath) {
    bool ans = m_ati.load_calibration(filepath);
    return ans;
}

void AtiWindowCal::set_calibration(mahi::robo::AtiSensor::Calibration& calibration_matrix) {
    m_ati.set_calibration(calibration_matrix);
}

void AtiWindowCal::zero() {
    FTbias_[0] = mean(forceAxisX.get_vector());
    FTbias_[1] = mean(forceAxisY.get_vector());
    FTbias_[2] = mean(forceAxisZ.get_vector());
    FTbias_[3] = mean(torqueAxisX.get_vector());
    FTbias_[4] = mean(torqueAxisY.get_vector());
    FTbias_[5] = mean(torqueAxisZ.get_vector());
}

double AtiWindowCal::get_force(mahi::robo::Axis axis) {
    update_biased_forcetorques();
    switch (axis) {
        case AxisX: return FTbSTG_[0];
        case AxisY: return FTbSTG_[1];
        case AxisZ: return FTbSTG_[2];
        default: return 0.0;
    }
}

std::vector<double> AtiWindowCal::get_forces() {
    update_biased_forcetorques();
    forces_[0] = FTbSTG_[0];
    forces_[1] = FTbSTG_[1];
    forces_[2] = FTbSTG_[2];
    return forces_;
}

double AtiWindowCal::get_torque(mahi::robo::Axis axis) {
    update_biased_forcetorques();
    switch (axis) {
        case AxisX: return FTbSTG_[3];
        case AxisY: return FTbSTG_[4];
        case AxisZ: return FTbSTG_[5];
        default: return 0.0;
    }
}

std::vector<double> AtiWindowCal::get_torques() {
    update_biased_forcetorques();
    torques_[0] = FTbSTG_[3];
    torques_[1] = FTbSTG_[4];
    torques_[2] = FTbSTG_[5];
    return torques_;
}

void AtiWindowCal::update_biased_forcetorques() {

    double Fx = m_ati.get_force(Axis::AxisX);
    double Fy = m_ati.get_force(Axis::AxisY);
    double Fz = m_ati.get_force(Axis::AxisZ);
    double Tx = m_ati.get_torque(Axis::AxisX);
    double Ty = m_ati.get_torque(Axis::AxisY);
    double Tz = m_ati.get_torque(Axis::AxisZ);

    FTbSTG_[0] = Fx - FTbias_[0];
    FTbSTG_[1] = Fy - FTbias_[1];
    FTbSTG_[2] = Fz - FTbias_[2];
    FTbSTG_[3] = Tx - FTbias_[3];
    FTbSTG_[4] = Ty - FTbias_[4];
    FTbSTG_[5] = Tz - FTbias_[5];

    forceAxisX.push_back(Fx);
    forceAxisY.push_back(Fy);
    forceAxisZ.push_back(Fz);
    torqueAxisX.push_back(Tx);
    torqueAxisY.push_back(Ty);
    torqueAxisZ.push_back(Tz);
}

