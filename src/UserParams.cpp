#include "UserParams.hpp"

// Written by Janelle Clark

/// Constructor
UserParams::UserParams(){};
/// Destructor
UserParams::~UserParams(){};

bool UserParams::exportParams(const std::string& filepath) {
    fs::path path(filepath);
    UserParams::Params params = getParams();
    Timestamp ts;
    json j;
    j["subnum"]                 = params.subnum;
    j["sex"]                    = params.sex;
    j["handedness"]             = params.handedness;
    j["age"]                    = params.age;
    j["bmi"]                    = params.bmi;
    j["skinfold"]               = params.skinfold;
    j["armCircum"]              = params.armCircum;
    j["hairLength"]             = params.hairLength;
    j["hairThick"]              = params.hairThick;
    j["hairSqInch"]             = params.hairSqInch;
    j["positionMin_n"]          = params.positionMin_n;
    j["positionMax_n"]          = params.positionMax_n;
    j["positionCont_n"]         = params.positionCont_n;
    j["positionMin_t"]          = params.positionMin_t;
    j["positionMax_t"]          = params.positionMax_t;
    j["positionCont_t"]         = params.positionCont_t;
    j["forceMin_n"]             = params.forceMin_n;
    j["forceMax_n"]             = params.forceMax_n;
    j["forceCont_n"]            = params.forceCont_n;
    j["forceMin_t"]             = params.forceMin_t;
    j["forceMax_t"]             = params.forceMax_t;
    j["forceCont_t"]            = params.forceCont_t;

    std::ofstream file(path);
    if (file.is_open()) {
        file << std::setw(10) << j;
        LOG(Info) << "Exported Subject " << m_params.subnum << " parameters to " << path.generic_string();
        file.close();
        return true;
    }
    LOG(Error) << "Failed to export Subject " << m_params.subnum << " parameters because because " << path << " could not be created or opened.";
    return false;
}

bool UserParams::importParams(const std::string& filepath) {
    fs::path path(filepath);
    if (fs::exists(path)) {

        try {
            std::ifstream file(path);
            json j;
            file >> j;

            m_params.subnum               = j["subnum"].get<int>();
            m_params.sex                  = j["sex"].get<std::string>();
            m_params.handedness           = j["handedness"].get<std::string>();
            m_params.age                  = j["age"].get<int>();
            m_params.bmi                  = j["bmi"].get<double>();
            m_params.skinfold             = j["skinfold"].get<double>();
            m_params.armCircum            = j["armCircum"].get<double>();
            m_params.hairLength           = j["hairLength"].get<double>();
            m_params.hairThick            = j["hairThick"].get<double>();
            m_params.hairSqInch           = j["hairSqInch"].get<double>();
            m_params.positionMin_n        = j["positionMin_n"].get<double>();
            m_params.positionMax_n        = j["positionMax_n"].get<double>();
            m_params.positionCont_n       = j["positionCont_n"].get<double>();
            m_params.positionMin_t        = j["positionMin_t"].get<double>();
            m_params.positionMax_t        = j["positionMax_t"].get<double>();
            m_params.positionCont_t       = j["positionCont_t"].get<double>();
            m_params.forceMin_n           = j["forceMin_n"].get<double>();
            m_params.forceMax_n           = j["forceMax_n"].get<double>();
            m_params.forceCont_n          = j["forceCont_n"].get<double>();
            m_params.forceMin_t           = j["forceMin_t"].get<double>();
            m_params.forceMax_t           = j["forceMax_t"].get<double>();
            m_params.forceCont_t          = j["forceCont_t"].get<double>();
                
            LOG(Info) << "Imported Subject " << m_params.subnum << " parameters from " << path.generic_string();
        }
        catch(...) {
            LOG(Error) << "Failed to import Subject " << m_params.subnum << " parameters.";
        }

        return true;
    }
    LOG(Error) << "Failed to import Subject " << m_params.subnum << " parameters because " << path << " does not exist.";
    return false;
}

UserParams::Params UserParams::getParams() const {
    return m_params;
}