#pragma once

#include <Mahi/Daq.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include "Util/XboxController.hpp"

using namespace mahi::util;
using namespace xbox;


class PsychTest;

class PsychTest : public mahi::util::NonCopyable {
public:

    enum Mode       { Idle, MCS };
    enum Gender     { NoGender, Male, Female };
    enum Handedness { Left, Right, Other };
    enum ControlType{ Position, Force };
    enum WhichDof   { Shear, Normal } m_whichDOF;
    enum WhichStim  { First, Second, NA};

    /// PsychTest Parameter Configuration
    struct Params {
        int subnum                  = 0;       
        std::string sex             = "NoGender";
        std::string handedness      = "NA";
        double age                  = 0;
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
        double n_msc_comparisons    = 7;  // this must be an odd number, for a proper experiment design and for the code to make the comparison vector
        double n_msc_reps           = 10; // total = 90 -> 45 reference first, 45 reference second
        double n_msc_windows        = 3;
        double msc_stimulus_time    = 0.33;
    };

    /// PsychTest Query
    struct Query {
        int time = 0;
        Mode testmode = MCS;
        ControlType controller = Position;
        int trialnum = 0;
        int generated_num = 0;
        int window = 0;
        int level = 0;
        double stimulus1 = 0;
        double stimulus2 = 0;
        int standard = 0;   // 1 or 2
        int comparison = 0; // 1 or 2
        int correct = 0;     // 0 = same, 1 = first, 2 = second
        int answer = 0;     // 1 or 2
        int greater = 2;    // 0 or 1
    };

//----------------------------------------------------------------------------------
// THREAD SAFE FUNCTIONS 
//----------------------------------------------------------------------------------

    /// Constructor
    PsychTest(int subnum, Params params, WhichDof whichdof, ControlType controller);
    /// Destructor
    ~PsychTest();
    /// Updates PsychTest
    void update(const mahi::util::Time &t);
    /// Configures a PsychTest 
    void setParams(Params config);
    /// Exports configuration to JSON 
    bool exportParams(const std::string& filepath);
    /// Imports configuration from JSON 
    bool importParams(const std::string& filepath);
    /// Gets a PsychTest configuration 
    Params getParams() const;
    /// Queries PsychTest for full state information 
    Query getQuery(bool immediate = false);
    /// Get most recent 10k Queries.
    void dumpQueries(const std::string& filepath);
    /// Calculate comparison stimulus vector for MCS test based on ref, number of comparisons, and intervals between them
    void buildComparisonVector();
    /// build all the randomized trial settings and structure to record trial results
    void buildStimTrials();
    /// Sets allowable normal position range for this subject
    void setNormalPositionRange(double min, double max);
    /// Sets allowable shear position range for this subject
    void setShearPositionRange(double min, double max);
    /// Sets allowable normal force range for this subject
    void setNormalForceRange(double min, double max);
    /// Sets allowable shear force range for this subject
    void setShearForceRange(double min, double max);

//----------------------------------------------------------------------------------
// UNSAFE FUNCTIONS (ONLY CALL THESE FROM WITHIN A TASBI CONTROLLER UPDATE METHOD)
//----------------------------------------------------------------------------------

    void fillQuery(Query &q);
    
    
    double m_jnd_stimulus_min;
    double m_userStimulusMin;
    double m_userStimulusMax;
    double m_userStimulusContact;
    std::vector<std::vector<Query>> m_stimulus_trials;

protected:
    // Status and Congiguration
    Params      m_params;    ///< parameters
    Query       m_q;         ///< most recent Query point
    RingBuffer<Query> m_Q;   ///< 10k Query history
    int m_subnum;
    Gender m_sex;
    Handedness m_hand;
    WhichDof m_whichdof;
    ControlType m_controller;

    Mode m_testmode;
    int m_trialnum;
    int m_generated_num;
    int m_window;
    int m_level;
    double m_stimulus1;
    double m_stimulus2;
    int m_standard;
    int m_comparison;
    int m_correct;
    int m_answer;
    int m_greater;

    int    n_jnd_trials_window;
    int    n_jnd_trials_total;
    double m_jnd_ref_index;
    double m_jnd_stimulus_reference;
    double m_jnd_stimulus_interval;
    std::vector<double> m_jnd_stim_levels;
};