// MIT License
//
// AtiWindowCal - Mechatronics Engine & Library
// Copyright (c) 2021 Mechatronics and Haptic Interfaces Lab - Rice University
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
// Author(s): Janelle Clark (janelle.clark@rice.edu)

// _________________________________________________________________________
// Given the 3D force and moment vectors at the centerpoint of a sphere, the 
// following code gives the location of the contact centroid.
// From "Contact Sensing from Force Measurements" by  Bicchi, et al. 1993

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
