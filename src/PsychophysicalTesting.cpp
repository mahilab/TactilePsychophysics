#include <Mahi/Util/Math/Functions.hpp>
#include "PsychophysicalTesting.hpp"
#include <filesystem>
#include <fstream>
#include <random>

using namespace mahi::daq;
using namespace mahi::util;
namespace fs = std::filesystem;

PsychTest::PsychTest(int subnum, Params config, WhichDof whichdof, ControlType controller) :
    m_Q(10000),
    m_jnd_stim_levels(m_params.n_msc_comparisons, 0)
{
    m_subnum = subnum;
    m_whichdof = whichdof;
    m_controller = controller;
    setParams(config);
    LOG(Info) << "Opened PsychTest for subject " << m_subnum << ". Set to default settings";
}

PsychTest::~PsychTest(){};

//=============================================================================
// PUBLIC (THREAD SAFE)
//=============================================================================

void PsychTest::update(const Time &t) {
    // update fixed query
    fillQuery(m_q);
    m_q.time = t.as_microseconds();
    m_Q.push_back(m_q);
}

void PsychTest::setParams(Params config) {
    m_params = config;

    n_jnd_trials_window = m_params.n_msc_comparisons * m_params.n_msc_reps * 2; // 180 trials per window
    n_jnd_trials_total  = n_jnd_trials_window * m_params.n_msc_windows; // 900 trials total
    m_jnd_ref_index     = int(floor(double(m_params.n_msc_comparisons)/2)); // this is the index since indexing starts at 0

    if (m_whichdof == Shear){
        if (m_controller == Position)
            m_jnd_stimulus_min  = m_params.positionCont_t; // ??????????????????? contact or abs threshold for force/position (from calibration?) 
        else if (m_controller == Force)
            m_jnd_stimulus_min  = m_params.forceCont_t; // ??????????????????? contact or abs threshold for force/position (from calibration?) 
    }else if (m_whichdof == Normal){
        if (m_controller == Position)
            m_jnd_stimulus_min  = m_params.positionCont_n; // ??????????????????? contact or abs threshold for force/position (from calibration?) 
        else if (m_controller == Force)
            m_jnd_stimulus_min  = m_params.forceCont_n; // ??????????????????? contact or abs threshold for force/position (from calibration?) 
    }

    buildComparisonVector();
    
}

bool PsychTest::exportParams(const std::string& filepath) {
    fs::path path(filepath);
    PsychTest::Params params = getParams();
    Timestamp ts;
    json j;
    j["subnum"]                 = params.subnum;
    j["date"]                   = ts.yyyy_mm_dd();
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
    j["n_msc_comparisons"]      = params.n_msc_comparisons;
    j["n_msc_reps"]             = params.n_msc_reps;
    j["n_msc_windows"]          = params.n_msc_windows;
    j["msc_stimulus_time"]      = params.msc_stimulus_time;

    std::ofstream file(path);
    if (file.is_open()) {
        file << std::setw(10) << j;
        LOG(Info) << "Exported PsychTest Subject " << m_subnum << " parameters to " << path.generic_string();
        file.close();
        return true;
    }
    LOG(Error) << "Failed to export PsychTest Subject " << m_subnum << " parameters because because " << path << " could not be created or opened.";
    return false;
}

bool PsychTest::importParams(const std::string& filepath) {
    fs::path path(filepath);
    if (fs::exists(path)) {

        try {
            std::ifstream file(path);
            json j;
            file >> j;
            Params params;

            params.subnum               = j["subnum"].get<int>();
            params.sex                  = j["sex"].get<std::string>();
            params.handedness           = j["handedness"].get<std::string>();
            params.age                  = j["age"].get<int>();
            params.bmi                  = j["bmi"].get<double>();
            params.skinfold             = j["skinfold"].get<double>();
            params.armCircum            = j["armCircum"].get<double>();
            params.hairLength           = j["hairLength"].get<double>();
            params.hairThick            = j["hairThick"].get<double>();
            params.hairSqInch           = j["hairSqInch"].get<double>();
            params.positionMin_n        = j["positionMin_n"].get<double>();
            params.positionMax_n        = j["positionMax_n"].get<double>();
            params.positionCont_n       = j["positionCont_n"].get<double>();
            params.positionMin_t        = j["positionMin_t"].get<double>();
            params.positionMax_t        = j["positionMax_t"].get<double>();
            params.positionCont_t       = j["positionCont_t"].get<double>();
            params.forceMin_n           = j["forceMin_n"].get<double>();
            params.forceMax_n           = j["forceMax_n"].get<double>();
            params.forceCont_n          = j["forceCont_n"].get<double>();
            params.forceMin_t           = j["forceMin_t"].get<double>();
            params.forceMax_t           = j["forceMax_t"].get<double>();
            params.forceCont_t          = j["forceCont_t"].get<double>();
            params.n_msc_comparisons    = j["n_msc_comparisons"].get<int>();
            params.n_msc_reps           = j["n_msc_reps"].get<int>();
            params.n_msc_windows        = j["n_msc_windows"].get<int>();
            params.msc_stimulus_time    = j["msc_stimulus_time"].get<double>();
                
            setParams(params);
            LOG(Info) << "Imported PsychTest Subject " << m_subnum << " parameters from " << path.generic_string();
            if (m_subnum != params.subnum)
                LOG(Error) << "Subject number at construction, " << m_subnum << "is not the same as in the imported parameters.";

        }
        catch(...) {
            LOG(Error) << "Failed to import PsychTest Subject " << m_subnum << " parameters.";
        }

        return true;
    }
    LOG(Error) << "Failed to import PsychTest Subject " << m_subnum << " parameters because " << path << " does not exist.";
    return false;
}

PsychTest::Params PsychTest::getParams() const {
    return m_params;
}

PsychTest::Query PsychTest::getQuery(bool immediate) {
    if (immediate) {
        Query q;
        fillQuery(q);
        return q;
    }
    return m_q;
}

void PsychTest::dumpQueries(const std::string& filepath) {
    Csv csv(filepath);
    if (csv.is_open()) {
        csv.write_row("time",
                      "testmode"
                      "controller"
                      "trialnum",
                      "generated_num",
                      "window",
                      "level",
                      "stimulus1",
                      "stimulus2",
                      "standard",
                      "comparison",
                      "correct",
                      "answer",
                      "greater");
        for (int i = 0; i < m_Q.size(); ++i) {
            Query& q = m_Q[i];
            csv.write_row(q.time,
                          q.testmode,
                          q.controller,          
                          q.trialnum,            
                          q.generated_num,            
                          q.window,   
                          q.level,     
                          q.stimulus1,     
                          q.stimulus2,
                          q.standard,     
                          q.comparison,     
                          q.correct,             
                          q.answer,     
                          q.greater);
        }
        csv.close();
    }
}

void PsychTest::buildComparisonVector(){

    if (m_whichdof == Shear){
        if (m_controller == Position){
            m_userStimulusMin = m_params.positionMin_t;
            m_userStimulusMax = m_params.positionMax_t;
        }else if (m_controller == Force){
            m_userStimulusMin = m_params.forceMin_t;
            m_userStimulusMax = m_params.forceMax_t;
        }
    }else if (m_whichdof == Normal){
        if (m_controller == Position){
            m_userStimulusMin = m_params.positionMin_n;
            m_userStimulusMax = m_params.positionMax_n;
        }else if (m_controller == Force){
            m_userStimulusMin = m_params.forceMin_n;
            m_userStimulusMax = m_params.forceMax_n;
        }
    }
    
    m_jnd_stimulus_reference = 0.75*(m_userStimulusMax - m_userStimulusMin) + m_userStimulusMin;
    m_jnd_stimulus_interval  = 0.02*(m_userStimulusMax - m_userStimulusMin); /// ????????????? base this value on lit/pilots
    double min_comparison = m_jnd_stimulus_reference - double(m_jnd_ref_index) * m_jnd_stimulus_interval;
    
    
    for (int i = 0; i < m_params.n_msc_comparisons; ++i) { // this writes a vector of length "m_params.n_msc_comparisons" centered at "m_jnd_stimulus_reference"
        m_jnd_stim_levels[i] = min_comparison + double(i) * m_jnd_stimulus_interval;
    std::cout << "m_jnd_stim_levels[i] " << m_jnd_stim_levels[i] << " i " << i << " ref " << m_jnd_stimulus_reference << std::endl;
    }
    std::cout << "min " << m_userStimulusMin << " max " << m_userStimulusMax << " interval " << m_jnd_stimulus_interval << std::endl;
}

void PsychTest::buildStimTrials() {
        std::random_device rd;
        std::mt19937 g(rd());
        g.seed(m_subnum);
        int i = 0;
        for (int w = 0; w < m_params.n_msc_windows; ++w) {
            std::vector<Query> window_trials;
            window_trials.reserve(n_jnd_trials_window);
            for (int r = 0; r < m_params.n_msc_reps; ++r) {
                for (int f = 0; f < m_params.n_msc_comparisons; ++f) {
                    Query trial;
                    //  standard first // ??????????????????? replace with psychtest set parameters
                    trial.generated_num     = i++;
                    trial.window      = w;
                    trial.level       = f-5; // minus m_jnd_stimulus_reference?
                    trial.stimulus1    = m_jnd_stimulus_reference;
                    trial.stimulus2    = m_jnd_stim_levels[f];
                    trial.standard   = 1;
                    trial.comparison = 2;
                    trial.correct    = trial.stimulus1 == trial.stimulus2 ? 0 : trial.stimulus1 > trial.stimulus2 ? 1 : 2;
                    trial.answer     = -1;
                    trial.greater     = -1;
                    window_trials.push_back(trial);
                    // standard second
                    trial.generated_num     = i++;
                    trial.window      = w;
                    trial.level       = f-5; // minus m_jnd_stimulus_reference?
                    trial.stimulus1    = m_jnd_stim_levels[f];
                    trial.stimulus2    = m_jnd_stimulus_reference;
                    trial.standard   = 2;
                    trial.comparison = 1;
                    trial.correct    =  trial.stimulus1 == trial.stimulus2 ? 0 : trial.stimulus1 > trial.stimulus2 ? 1 : 2;
                    trial.answer     = -1;
                    trial.greater    = -1;
                    window_trials.push_back(trial);
                }
            }
            // shuffle window trials
            std::shuffle(window_trials.begin(), window_trials.end(), g);
            // append
            m_stimulus_trials.push_back(window_trials);
        }
        // assign trial numbers
        i = 0;
        for (int w = 0; w < m_params.n_msc_windows; ++w) {
            for (auto& trial : m_stimulus_trials[w])
                trial.trialnum = i++;
        }
    }



void PsychTest::setNormalPositionRange(double min, double max) {
    m_params.positionMin_n = min;
    m_params.positionMax_n = max;
    LOG(Info) << "Set PsychTest Subject " << m_subnum << " normal position range to [ " << min << " , " << max << " mm].";
}

void PsychTest::setShearPositionRange(double min, double max) {
    m_params.positionMin_t = min;
    m_params.positionMax_t = max;
    LOG(Info) << "Set PsychTest Subject" << m_subnum << " shear position range to [ " << min << " , " << max << " mm].";
}

void PsychTest::setNormalForceRange(double min, double max) {
    m_params.forceMin_n = min;
    m_params.forceMax_n = max;
    LOG(Info) << "Set PsychTest Subject" << m_subnum << " normal force range to [ " << min << " , " << max << " N].";
}

void PsychTest::setShearForceRange(double min, double max) {
    m_params.forceMin_t = min;
    m_params.forceMax_t = max;
    LOG(Info) << "Set PsychTest Subject" << m_subnum << " shear force range to [ " << min << " , " << max << " N].";
}

//=============================================================================
// PRIVATE (NOT THREAD SAFE)
//=============================================================================

void PsychTest::fillQuery(PsychTest::Query &q) {
    
    q.testmode             = m_testmode;
    q.controller           = m_controller;
    q.trialnum             = m_trialnum;
    q.generated_num        = m_generated_num;
    q.window               = m_window;
    q.level                = m_level;
    q.stimulus1            = m_stimulus1;
    q.stimulus2            = m_stimulus2;
    q.standard             = m_standard;
    q.comparison           = m_comparison;
    q.correct              = m_correct;
    q.answer               = m_answer;
    q.greater              = m_greater;
}
