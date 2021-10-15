#pragma once
#include <random>
#include <algorithm>
#include <Mahi/Daq.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include "Util/XboxController.hpp"
#include "UserParams.hpp"

using namespace mahi::util;
using namespace xbox;


class PsychTest;

class PsychTest : public mahi::util::NonCopyable {
public:

    enum WhichExp   { MCS, SM, MA };
    enum Mode       { Idle, Run };
    enum Gender     { NoGender, Male, Female };
    enum Handedness { Left, Right, Other };
    enum ControlType{ Position, Force };
    enum WhichDof   { Shear, Normal };
    enum WhichStim  { First, Second, Choose, NA};
    enum Direction  { Up, Down, None};

    /// PsychTest Parameter Configuration
    struct Params {
        int    n_mcs_comparisons    = 7;  // this must be an odd number, for a proper experiment design and for the code to make the comparison vector
        int    n_mcs_reps           = 10; // total = 90 -> 45 reference first, 45 reference second
        int    n_mcs_windows        = 1;
        int    n_sm_staircases      = 4;
        int    n_sm_crossovers      = 4;
        double sm_pos_inc           = 0.3;
        double sm_force_inc         = 0.5;
        int    n_ma_trials          = 10;       
        double stimulus_time        = 0.33;
    };

    /// PsychTest QueryMCS
    struct QueryMCS {
        int time = 0;
        Mode testmode = PsychTest::Idle;
        WhichExp whichExp = PsychTest::MCS;
        WhichDof whichDof;
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
        int greater = 2;    // 0 or 1 // did they say the comparison was greater?
    };

    /// PsychTest QuerySM
    struct QuerySM {
        int time = 0;
        Mode testmode = PsychTest::Idle;
        WhichExp whichExp = PsychTest::SM;
        WhichDof whichDof;
        ControlType controller = Position;
        int trialnum = 0;
        int num_staircase = 0;
        Direction direction = PsychTest::None; // will be either moving up or down from the previous trial
        Direction lastSlope = PsychTest::None;
        int num_reversal = 0;
        double stimulus1 = 0;
        double stimulus2 = 0;
        int standard = 0;   // 1 or 2
        int comparison = 0; // 1 or 2
        int correct = 0;     // 0 = same, 1 = first, 2 = second
        int answer = 0;     // 1 or 2
        int greater = 2;    // 0 or 1 // did they say the comparison was greater?
    };

    /// PsychTest QueryMA
    struct QueryMA {
        int time                = 0;
        Mode testmode           = PsychTest::Idle;
        WhichExp whichExp       = PsychTest::SM;
        WhichDof whichDof;
        ControlType controller  = Position;
        int trialnum            = 0;
        double stimulus1        = 0;
        double stimulus2        = 0;
        int standard            = 1;   // always 1
        int comparison          = 2; // always 2
        int adjust              = 1;     // 0 = same, 1 = first, 2 = second
        Direction direction     = PsychTest::None;
        double change           = 0;     // amount the user changed the comparison for the next trial
        double finalVal       = -1;
    };

//----------------------------------------------------------------------------------
// THREAD SAFE FUNCTIONS 
//----------------------------------------------------------------------------------

    /// Constructor
    PsychTest(int subnum, Params params, PsychTest::WhichExp whichExp, WhichDof whichdof, ControlType controller);
    /// Destructor
    ~PsychTest();
    /// Configures a PsychTest 
    void setParams(Params config);
    /// Exports configuration to JSON 
    bool exportParams(const std::string& filepath);
    /// Imports configuration from JSON 
    bool importParams(const std::string& filepath);
    /// Gets a PsychTest configuration 
    Params getParams() const;
    /// Sets allowable normal position range for this subject
    void setNormalPositionRange(double min, double max);
    /// Sets allowable shear position range for this subject
    void setShearPositionRange(double min, double max);
    /// Sets allowable normal force range for this subject
    void setNormalForceRange(double min, double max);
    /// Sets allowable shear force range for this subject
    void setShearForceRange(double min, double max);

    //////////// Method of Constant Stimuli (MCS) Functions ///////////////
    
    /// QueriesMCS PsychTest for full state information 
    QueryMCS getQueryMCS(){return m_q_mcs;}
    /// Set subject response in corresponding query 
    void setResponseMCS(int answer, int greater);
    /// Print out query values with std::cout
    void printQueriesMCS(QueryMCS query);
    /// Calculate comparison stimulus vector for MCS test based on ref, number of comparisons, and intervals between them
    void buildComparisonVector();
    /// build all the randomized trial settings and structure to record trial results
    void buildStimTrials();

    //////////// Staircase Method (SM) Functions ///////////////

    /// QueriesSM PsychTest for full state information 
    QuerySM getQuerySM(){return m_q_sm;}
    /// Set subject response in corresponding query 
    void setResponseSM(int answer, int greater);
    /// Print out query values with std::cout
    void printQueriesSM(QuerySM query);
    /// determine comparison value and structure to record trial results
    void setNextTrialSM();
    /// Reinitialize member variables for new staircase
    void setNewStaircase();

    //////////// Method of Adjustments (MA) Functions ///////////////

    /// QueriesSM PsychTest for full state information 
    QueryMA getQueryMA(){return m_q_ma;}
    /// Set subject response in corresponding query 
    void setResponseMA(int adjust, double submittedVal);
    /// Print out query values with std::cout
    void printQueriesMA(QueryMA query);
    /// determine comparison value and structure to record trial results
    void setNextTrialMA();
//----------------------------------------------------------------------------------
// UNSAFE FUNCTIONS (ONLY CALL THESE FROM WITHIN A TASBI CONTROLLER UPDATE METHOD)
//----------------------------------------------------------------------------------

    // Experiment Settings
    int         m_subject;
    WhichExp    m_whichExp;
    WhichDof    m_whichDof;
    ControlType m_controller;
    Mode        m_testmode = Idle;

    std::string expchoice[3] = { "MCS", "SM", "MA" };
    std::array<std::string,3> method = { "Method of Constant Stimuli", "Staircase Method", "Method of Adjustments"};
    std::string controlChoice[2] = { "Position", "Force"};
    std::string dofChoice[2] = { "Shear", "Normal"};
    std::string currdirection[3] = {"Up", "Down", "None"};
    
    // General Experiment Variables
    double      m_jnd_stimulus_reference;
    double      m_jnd_stimulus_comparison = 0;
    double      m_jnd_stimulus_interval = 0;
    double      m_userStimulusMin;
    double      m_userStimulusMax;
    double      m_userStimulusContact; 
    QueryMCS    m_q_mcs;     ///< most recent QueryMCS point
    QuerySM     m_q_sm;      ///< most recent QuerySM point
    QueryMA     m_q_ma;      ///< most recent QueryMA point

    UserParams  m_up;
    UserParams::Params  m_userparams;    ///< parameters

    std::vector<std::vector<QueryMCS>> m_stim_trials_mcs;

protected:
    // Status and Configuration
    Params      m_params;    ///< parameters
    Gender      m_sex;
    Handedness  m_hand;

    // General Experiment Variables
    int         m_trialnum = 0;
    
    // Method of Constant Stimuli Variables
    int         n_jnd_trials_window;
    int         n_jnd_trials_total; //????????????
    double      m_jnd_ref_index;
    std::vector<double> m_jnd_stim_levels;

    //Staircase Method Variables
    std::vector<QuerySM> m_stim_trials_sm;
    bool        isNewStair = 1;
    int         m_num_staircase = 0;
    int         m_num_reversal = 0;
    Direction   m_direction = None; 
    Direction   m_lastSlope = None;

    //Method of Adjustments Variables
    std::vector<QueryMA> m_stim_trials_ma;
    int         m_adjust = 1;     // 0 = no, 1 = yes
    // Direction   m_direction = None; (redundant for staircase)
    double      m_change = 0;
    double      m_finalVal = -1;
};