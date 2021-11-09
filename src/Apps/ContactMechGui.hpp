#pragma once

#include <Mahi/Gui.hpp>
#include <Mahi/Robo.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <algorithm>
#include <random>
#include "CapstanModule.hpp"
#include "CMHub.hpp"
#include "UserParams.hpp"
#include "Util/HertzianContact.hpp"
#include "Util/ForceTorqueCentroid.hpp"

#include <Mahi/Daq.hpp>
#include <Mahi/Util.hpp>

using namespace ContactMechanics;


struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};

class ContactMechGui : public mahi::gui::Application {
public:

    enum WhichExp       { Ind, Creep, Relax, Cycle};
    enum Mode           { Idle, SetUp, Run};
    enum Gender         { NoGender, Male, Female};
    enum Handedness     { Left, Right };
    enum ControlType    { Position, Force};
    enum WhichDof       { Shear, Normal };
    enum WhichStim      { First, Second, Choose, NA};
    enum Direction      { Up, Down, None};
    enum PointInterest  { Start, Peak, Min, HoldInitial, HoldFinal, Other};
    enum TestLockDof    { Lock, Test};
    enum WhichSpeed     { Experiment, Calibration};

    /// ContactMechGui Parameter Configuration
    struct Params {
        double start_height         = -5.0; // [mm]
        double initial_force        = 1.0;  // [N]  contact, the pub says 1G of force
        double normalForceForTan    = 3;    // [N]
        double stimulus_velocity    = 0.5;  // [mm/s]
        double travel_velocity      = 4.0;  // [mm/s]
        double trial_break          = 30;   // [s]
        int    n_ind_trials         = 5;
        double ind_final_force      = 5.0;  // [N]  subject max, the pub says 5-10G of force
        int    n_creep_trials       = 5;
        double creep_hold           = 30;   // [s]
        double creep_final_force    = 5.0;  // [N]  subject max, the pub says 500N ...
        int    n_relax_trials       = 5;    
        double relax_hold           = 30;   // [s]
        double relax_final_force    = 5.0;  // [N] subject max, the pub says 500N ...
        int    n_cycle_cycles       = 5;
        double cycle_min_force      = -1;   // [N]  subject min
        double cycle_peak_force     = -1;  // [N]  subject max
    };

        /// ContactMechGui QueryMCS
    struct QueryContact {
        int time = 0;
        Mode testmode = ContactMechGui::Idle;
        WhichExp whichExp = ContactMechGui::Ind;
        WhichDof whichDof;
        ControlType controller = Position;
        PointInterest poi = Other;
        int cyclenum = 0;
        double Fn = 0;
        double Ft = 0;
        double deltaN = 0;
        double deltaT = 0;
    };

    // Indentation - Psychophysical Study
    // Done in either force or position control
    // Assumes a 2 DOF device

    ContactMechGui(int subject, ContactMechGui::WhichExp whichExp, ContactMechGui::WhichDof whichDOF);

    ~ContactMechGui();

    void update() override;

    void moveConstVel(double elapsed, TestLockDof isTest, WhichSpeed whichSpeed, double start, double stop);

    void updateQuery();
    
    void writeOutputVariables(Csv& csv);

    void writeOutputData(Csv& csv);

    void writeNormalHzOutputVariables(Csv& csv);

    void writeNormalHzOutputData(Csv& csv);

    void writeTanNoSlipHzOutputVariables(Csv& csv);

    void writeTanNoSlipHzOutputData(Csv& csv);

    void writeTanPartialSlipHzOutputVariables(Csv& csv);

    void writeTanPartialSlipHzOutputData(Csv& csv);

    
    // Experiment Functions

    Enumerator runICSRExperiment();
    
    Enumerator runCycleExperiment();

    // Hardware Specific Functions

    void connectToIO();

    void importUserHardwareParams();
    
    void stopExp();

    void calibrate();

    Enumerator findContact();

    Enumerator bringToStartPosition();

    Enumerator lockExtraDofs();

    Enumerator setControlDof();

    void setPositionControl(TestLockDof isTest);

    void setForceControl(TestLockDof isTest);

    void setTest(double N);

    void setLock(double N);

    void  getFNUpdate();

    void  userLimitsExceeded();

    double getTestPos();

    double getLockPos();

    double getTestForce();

    double getLockForce();

    void plotWindow();

private:
    // Status and Configuration
    int         m_subject;
    WhichExp    m_whichExp;
    WhichDof    m_whichDof;
    Mode        m_testmode = SetUp;
    Params      m_params;    ///< parameters
    Gender      m_sex;
    Handedness  m_hand;

    Timestamp ts;
    std::string filename_timeseries;
    Csv csv_timeseries;
    std::string filename;
    Csv csv;
    
    UserParams::Params m_userparams;
    UserParams m_up;

    std::array<std::string,4> expchoice = { "Ind", "Creep", "Relax", "Cycle"};
    std::array<std::string,4> method = { "Indentation", "Creep", "Stress Relaxation", "Cycle Between Bounds"};
    std::vector<std::string> dofChoice = { "Shear", "Normal"};
    std::array<std::string,3> currdirection = {"Up", "Down", "None"};
    std::array<std::string,6> currpoi  = { "Start", "Peak", "Min", "HoldInitial", "HoldFinal", "Other"};
    
    // General Experiment Variables
    PointInterest m_poi = Other;
    ControlType m_controller = Position;
    QueryContact  m_q;        //< most recent QueryInd point
    int     m_cyclenum                  = 0;
    double  m_Fn                        = 0;
    double  m_Ft                        = 0;
    double  m_deltaN                    = 0;
    double  m_deltaT                    = 0;
    bool    m_debug                     = false;
    bool    m_flag_presentStims         = false;
    bool    m_flag_reachedMAValue       = false;
    bool    m_flag_first_to_start;
    double  m_maxRangePercent           = 0.8;
    double  m_userStimulusForceMin      = 0;
    double  m_userStimulusForceMax      = 0;
    double  m_userStimulusForceContact  = 0;
    double  m_userStimulusPosMin        = 0;
    double  m_userStimulusPosMax        = 0;
    double  m_userStimulusPosContact    = 0;
    double  m_userShearTestNormPos      = 0;
    double  m_targetPosLock             = 0;
    double  m_targetPosTest             = 0;

    // Hertzian Contact
    HertzianContact             m_hz;
    HertzianContact::QueryHZ    m_q_hz_ns;  // Hertzian contact query - no slip
    double                      m_R = 30;       // [mm] - Radius of the spherical end effector

    // force-torque centroid
    FTC m_ftc;
    std::vector<double> m_ftc_centroid;

    // Plotting variables
    ScrollingBuffer lockForce, lockPosition, testForce, testPosition, testCmd, lockCmd;
    float t = 0;
    float m_history = 30.0f;

    // CM
    CMHub m_hub;
    std::shared_ptr<CM> m_cm_test;
    std::shared_ptr<CM> m_cm_lock;
    CM::Params m_paramsCMTest;
    CM::Params m_paramsCMLock;
};