#pragma once
#include <filesystem>
#include <fstream>
#include <Mahi/Util.hpp>

// Written by Janelle Clark 


using namespace mahi::util;
namespace fs = std::filesystem;

class UserParams;

class UserParams : public mahi::util::NonCopyable {

public:

struct Params {
        int subnum                  = 0;       
        std::string sex             = "NoGender";
        std::string handedness      = "NA";
        int age                     = 0;
        double bmi                  = 0;
        double skinfold             = 0;
        double armCircum            = 0;
        double hairLength           = 0;
        double hairThick            = 0;
        double hairSqInch           = 0;
        double positionMin_n        = 0;
        double positionMax_n        = 8;
        double positionCont_n       = 1;
        double positionMin_t        = 0;
        double positionMax_t        = 8;
        double positionCont_t       = 0;
        double forceMin_n           = 1;
        double forceMax_n           = 12;
        double forceCont_n          = 1;
        double forceMin_t           = 1;
        double forceMax_t           = 12;
        double forceCont_t          = 0;
    };

    /// Constructor
    UserParams();
    /// Destructor
    ~UserParams();
    /// Exports configuration to JSON 
    bool exportParams(const std::string& filepath);
    /// Imports configuration from JSON 
    bool importParams(const std::string& filepath);
    /// Gets a PsychTest configuration 
    Params getParams() const;

    // Experiment Settings
    Params      m_params;    ///< parameters
};