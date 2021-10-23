#pragma once

#include <Mahi/Gui.hpp>
#include <Mahi/Robo.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <algorithm>
#include <random>
#include "CapstanModule.hpp"
#include "CMHub.hpp"
#include "UserParams.hpp"

#include <Mahi/Daq.hpp>
#include <Mahi/Util.hpp>


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
    enum Mode           { Idle, Run};
    enum Gender         { NoGender, Male, Female};
    enum Handedness     { Left, Right };
    enum ControlType    { Position, Force};
    enum WhichDof       { Shear, Normal };
    enum WhichStim      { First, Second, Choose, NA};
    enum Direction      { Up, Down, None};
    enum PointInterest  { Start, Peak, Min, HoldInitial, HoldFinal, Other};

    /// ContactMechGui Parameter Configuration
    struct Params {
        double start_height         = 10;   // [mm]
        double initial_force        = .5;    // [N]  Question - what is 1G of force ?????????????
        double normalForceForTan    = 3;    // [N]
        double stimulus_velocity    = 0.5;  // [mm/s]
        double trial_break          = 30;   // [s]
        int    n_ind_trials         = 5;
        double ind_final_force      = 2;    // [N]  Question - what is 5-10G of force ?????????
        int    n_creep_trials       = 5;
        double creep_hold           = 30;   // [s]
        double creep_final_force    = 500;  // [N]  Question - is this ok???????
        int    n_relax_trials       = 5;    
        double relax_hold           = 30;   // [s]
        double relax_final_force    = 500;  // [N]  Question - is this ok???????
        int    n_cycle_cycles       = 5;
        double relax_min_force      = 500;  // [N]  subject min
        double relax_peak_force     = 500;  // [N]  subject max
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

    void moveConstVel(double elapsed, bool isTest, double start, double stop);

    void updateQuery();
    
    void writeOutputVariables(Csv& csv);

    void writeOutputData(Csv& csv);

    
    // Experiment Functions

    Enumerator runICSRExperiment();
    
    Enumerator runCycleExperiment();

    // Hardware Specific Functions

    void connectToIO();

    void importUserHardwareParams();
    
    void stopExp();

    void calibrate();

    Enumerator bringToStartPosition();

    Enumerator bringToContact();

    void lockExtraDofs();

    void switchControllers(); // toggle between force and position control

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
    Mode        m_testmode = Idle;
    Params      m_params;    ///< parameters
    Gender      m_sex;
    Handedness  m_hand;
    
    UserParams::Params m_userparams;
    UserParams m_up;

    std::string expchoice[4] = { "Ind", "Creep", "Relax", "Cycle"};
    std::array<std::string,4> method = { "Indentation", "Creep", "Stress Relaxation", "Cycle Between Bounds"};
    std::string dofChoice[2] = { "Shear", "Normal"};
    std::string currdirection[3] = {"Up", "Down", "None"};
    std::string currpoi[6]  = { "Start", "Peak", "Min", "HoldInitial", "HoldFinal", "Other"};
    
    // General Experiment Variables
    PointInterest m_poi;
    ControlType m_controller = Position;
    int     m_cyclenum             = 0;
    double  m_Fn                   = 0;
    double  m_Ft                   = 0;
    double  m_deltaN               = 0;
    double  m_deltaT               = 0;
    bool    m_debug                = 0;
    bool    m_flag_presentStims    = 0;
    bool    m_flag_reachedMAValue  = 0;
    double  m_userStimulusMin      = 0;
    double  m_userStimulusMax      = 0;
    double  m_userStimulusContact  = 0;
    double  m_targetPosLock        = 0;
    double  m_targetPosTest        = 0;
    QueryContact    m_q;     ///< most recent QueryInd point

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