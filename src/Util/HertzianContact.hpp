// MIT License
//
// ContactMechanics - Mechatronics Engine & Library
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

// Hertzian contact class for contact mechanics of a rigid, spherical indentor on 
// an infinite plane based on (Johnson, 1985), (Barber, 2018), and (Joodaki, 2018). 

#pragma once
#include <vector>
#include <iostream>

namespace ContactMechanics{

class HertzianContact {
public:

    struct QueryHZ {
        double R = 0;           // [mm] Radius of the indentor
        double a = 0;           // [mm] radius of the contact area of the indentor
        double planarA = 0;     // [mm^2] contact area calculated as if 2D
        double sphericalA = 0;  // [mm^2] contact area calculated as if 3D
        double Fn = 0;          // [N] Normal Load
        double Ft = 0;          // [N] Tangential Load
        double deltaN = 0;      // [mm] Normal Displacement, indentation depth
        double deltaT = 0;      // [mm] Tangential Displacement
        double combinedE = 0;   // [N/mm^2] [MPa] combined Young's modulus, E* of the two interacting materials
        double Wn = 0;          // [??? N^(7/3)/mm^(5/3)] Normal elastic strain energy. Johnson (6.42)
        double couplingP = 0;   // [-] beta - coupling parameter between normal and tangential load
        double E = 0;           // [N/mm^2] [MPa] Young's Modulus for elasticity for the infinite plane
        double v = 0;           // [-] nu - Poisson's ratio for the infinite plane
        double G = 0;           // [N/mm^2] [MPa] Shear Modulus of Rigidity of the two interfacing materials
        double meanStress = 0;  // [N/mm^2] mean stress, sigma*
        double meanStrain = 0;  // [-] mean strain, epsilon*
        double Wt = 0;          // Work done by the tangential force during a complete cycle with microslip. Johnson (7.60)
        double mu = 0;          // [-] mu - frictional coefficient
        double complianceN = 0; // [mm/N] normal compliance (d deltaN/ d Fn)
        double complianceT = 0; // [mm/N] tangential compliance (d deltaT/ d Ft)
    };

    ////////////////////////////////////////////////////////////////////
    //    Make Query with measurement data
    ////////////////////////////////////////////////////////////////////
        
    QueryHZ makeQuery_Normal(double R, double Fn, double deltaN);
    QueryHZ makeQuery_TanNoSlip(double R, double Fn, double Ft, double deltaN, double deltaT);
    QueryHZ makeQuery_TanPartialSlip(double R, double Fn, double Ft, double deltaN, double deltaT, double mu);

    ////////////////////////////////////////////////////////////////////
    //    Print Query
    ////////////////////////////////////////////////////////////////////
        
    void printQuery();

    ////////////////////////////////////////////////////////////////////
    //    Direct Calculations
    ////////////////////////////////////////////////////////////////////

    // Normal Loading
    double getCombinedE_Normal(double R, double Fn, double deltaN);
    double getContactRadius_Normal(double R, double deltaN);
    double getPlanarA_Normal(double R, double deltaN);
    double getSphericalA_Normal(double R, double deltaN);
    double getMeanStress_Normal(double R, double Fn, double deltaN);
    double getMeanStrain_Normal(double R, double deltaN);
    double getElasticStrainEnergy_Normal(double R, double Fn, double deltaN);

    // Tangential Loading
    double getCouplingParameter_Tan(double v);
    std::vector<double> getYoungAndPoisson_TanNoSlip(double R, double Fn, double Ft, double deltaN, double deltaT);
    std::vector<double> getYoungAndPoisson_TanPartialSlip(double R, double Fn, double Ft, double deltaN, double deltaT, double mu);
    double getShearModulus_Tan(double E, double v);
    std::vector<double> getCompliance_Tan(double E, double v, double a);
    double getWorkofCycle_Tan(double R, double Fn, double Ft, double deltaN, double deltaT, double mu); // valid at peak Ft


private:
    ////////////////////////////////////////////////////////////////////
    //    Fill Query with measurement data
    ////////////////////////////////////////////////////////////////////
        
    void fillQuery_Normal();
    void fillQuery_TanNoSlip();
    void fillQuery_TanPartialSlip();

    ////////////////////////////////////////////////////////////////////
    //    Normal Indentation Functions
    ////////////////////////////////////////////////////////////////////

    void getCombinedE_Normal();
    void getContactRadius_Normal();
    void getPlanarA_Normal();
    void getSphericalA_Normal();
    void getMeanStress_Normal();
    void getMeanStrain_Normal();
    void getElasticStrainEnergy_Normal();

    /////////////////////////////////////////////////////////////////////////////////
    //    Tangential Loading Functions - Also assume isotropic and Goodman Approximation
    /////////////////////////////////////////////////////////////////////////////////

    void getCouplingParameter_Tan();
    void getYoungAndPoisson_TanNoSlip();
    void getYoungAndPoisson_TanPartialSlip();
    void getShearModulus_Tan();
    void getCompliance_Tan();
    void getWorkofCycle_Tan();  // valid at peak Ft

    /////////////////////////////////////////////////////////////////////////////////
    //    Member Variables
    /////////////////////////////////////////////////////////////////////////////////
    QueryHZ m_q;
    double m_pi = 3.141592653589793238462643383279502884;

}; // class
} // namespace