// MIT License
//
// Copyright (c) 2020 Mechatronics and Haptic Interfaces Lab - Rice University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// Author(s): Evan Pezent (epezent@rice.edu)

#pragma once
#include <Mahi/Robo/Mechatronics/ForceSensor.hpp>
#include <Mahi/Robo/Mechatronics/TorqueSensor.hpp>
#include <Mahi/Robo/Mechatronics/AtiSensor.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Util/System.hpp>
#include <array>
#include <string>

/// Implements an ATI force/torque transducer
class AtiWindowCal : public mahi::robo::ForceSensor, public mahi::robo::TorqueSensor {
public:
    /// Constucts AtiWindowCal with unspecified channels and no calibration
    AtiWindowCal();
    /// Constucts AtiWindowCal with specified channels and calibration matrix
    AtiWindowCal(const double* ch0, const double* ch1, const double* ch2, const double* ch3,
                     const double* ch4, const double* ch5, const std::string& filepath);
    /// Sets the voltages channels associated with this ATI sensor
    void set_channels(const double* ch0, const double* ch1, const double* ch2, const double* ch3,
                      const double* ch4, const double* ch5);
    /// Loads calibration from ATI calibration file (e.g. "FTXXXXX.cal")
    bool load_calibration(const std::string& filepath);
    /// Allows for manually setting calibration
    void set_calibration(mahi::robo::AtiSensor::Calibration& calibration);
    /// Returns force along specified axis
    double get_force(mahi::robo::Axis axis) override;
    /// Returns forces along X, Z, and Z axes
    std::vector<double> get_forces() override;
    /// Returns torque along specifed axis
    double get_torque(mahi::robo::Axis axis) override;
    /// Returns torque along X, Z, and Z axes
    std::vector<double> get_torques() override;
    /// Zeros all forces and torques at current preload
    void zero() override;

private:
    /// Updates biased voltages
    void update_biased_forcetorques();

private:
    mahi::robo::AtiSensor m_ati;
    std::array<double, 6>      FTbias_;         ///< bias vector
    std::array<double, 6>      FTbSTG_;         ///< biased strain gauge voltages
    mahi::util::RingBuffer<double> forceAxisX{50};
    mahi::util::RingBuffer<double> forceAxisY{50};
    mahi::util::RingBuffer<double> forceAxisZ{50};
    mahi::util::RingBuffer<double> torqueAxisX{50};
    mahi::util::RingBuffer<double> torqueAxisY{50};
    mahi::util::RingBuffer<double> torqueAxisZ{50};
};