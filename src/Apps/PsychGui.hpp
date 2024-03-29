#pragma once

#include <Mahi/Gui.hpp>
#include <Mahi/Robo.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <algorithm>
#include <random>
#include "CapstanModule.hpp"
#include "CMHub.hpp"
#include "PsychophysicalTesting.hpp"
#include "Util/XboxController.hpp"
#include "Util/HertzianContact.hpp"

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

class PsychGui : public mahi::gui::Application {
public:

    // Method of Constant Stimuli - Psychophysical Study
    // Done in either force or position control
    // Assumes a 2 DOF device

    PsychGui(int subject, PsychTest::WhichExp whichExp, PsychTest::WhichDof whichDOF, PsychTest::ControlType controller);

    ~PsychGui();

    void update() override;

    int responseWindow(PsychTest::WhichStim whichStim);

    void rampStimulus(double start, double end, double ramptime, double elapsed);
    
    void rampLock(double start, double end, double ramptime, double elapsed);
    
    // Method of Constant Stimuli Functions

    Enumerator runMCSExperiment();

    void writeMCSOutputStimVariables(Csv& csv);

    void writeMCSOutputStimData(Csv& csv, PsychTest::QueryMCS trial);

    void writeMCSOutputTimeVariables(Csv& csv);

    void writeMCSOutputTimeData(Csv& csv);

    // Staircase Method Functions  

    Enumerator runSMExperiment();

    void writeSMOutputStimVariables(Csv& csv);

    void writeSMOutputStimData(Csv& csv, PsychTest::QuerySM trial);

    void writeSMOutputTimeVariables(Csv& csv);

    void writeSMOutputTimeData(Csv& csv);

    // Method of Adjustments Functions

    void responseWindowMA(PsychTest::WhichStim whichStim);

    Enumerator runMAExperiment();
    
    void writeMAOutputStimVariables(Csv& csv);

    void writeMAOutputStimData(Csv& csv, PsychTest::QueryMA trial);

    void writeMAOutputTimeVariables(Csv& csv);

    void writeMAOutputTimeData(Csv& csv);
    
    // Hardware Specific Functions

    void connectToIO();

    void importUserHardwareParams();
    
    void stopExp();

    void calibrate();

    Enumerator findContact();

    Enumerator bringToStartPosition();

    Enumerator lockExtraDofs();

    Enumerator setControlDof();

    void setPositionControl(bool isTest);

    void setForceControl(bool isTest);

    void setTest(double N);

    void setLock(double N);

    void  getFNUpdate();

    void userLimitsExceeded();

    void collectSensorData(PsychTest::WhichStim whichStim, int refOrder);

    void avgSensorData();

    void plotDebugExpInfo();

    // Experiment
    PsychTest m_pt;
    PsychTest::Params m_psychparams;
    bool m_debug            = 0;
    bool m_flag_is_JND = 0; // use this for SM and MCS too?
    bool m_flag_presentStims = 0;
    bool m_flag_reachedMAValue = 0;
    double m_whatChange = 0;
    int m_adjust = -1;
    double m_jnd_current_stimulus;
    bool m_flag_first_to_start = 0;

    Timestamp ts;
    std::string filename_timeseries;
    std::string filename;
    Csv csv_timeseries;
    Csv csv;

    // Create buffers for calculating the forces and positions during each cue
    mahi::util::RingBuffer<double> m_ref_normF{50};
    mahi::util::RingBuffer<double> m_ref_shearF{50};
    mahi::util::RingBuffer<double> m_ref_normP{50};
    mahi::util::RingBuffer<double> m_ref_shearP{50};

    mahi::util::RingBuffer<double> m_comp_normF{50};
    mahi::util::RingBuffer<double> m_comp_shearF{50};
    mahi::util::RingBuffer<double> m_comp_normP{50};
    mahi::util::RingBuffer<double> m_comp_shearP{50};

    mahi::util::RingBuffer<double> m_adjust_normF{50};
    mahi::util::RingBuffer<double> m_adjust_shearF{50};
    mahi::util::RingBuffer<double> m_adjust_normP{50};
    mahi::util::RingBuffer<double> m_adjust_shearP{50};
    
    double m_ref_avgNormF;
    double m_ref_avgShearF;
    double m_ref_avgNormP;
    double m_ref_avgShearP;

    double m_comp_avgNormF;
    double m_comp_avgShearF;
    double m_comp_avgNormP;
    double m_comp_avgShearP;

    double m_adjust_avgNormF;
    double m_adjust_avgShearF;
    double m_adjust_avgNormP;
    double m_adjust_avgShearP;

    PsychTest::WhichStim m_whichStim = PsychTest::NA;
    double m_NormF;
    double m_ShearF;
    double m_NormP;
    double m_ShearP;

    // Plotting variables
    ScrollingBuffer lockForce, lockPosition, testForce, testPosition, ref, comp, curr, torCmd;
    float t = 0;
    float m_history = 30.0f;

    // Hertzian Contact
    HertzianContact             m_hz;
    HertzianContact::QueryHZ    m_q_hz_ns;  // Hertzian contact query - no slip
    double                      m_R = 15;       // [mm] - Radius of the spherical end effector

    // CM
    CMHub m_hub;
    std::shared_ptr<CM> m_cm_test;
    std::shared_ptr<CM> m_cm_lock;
    CM::Params m_paramsTest;
    CM::Params m_paramsLock;

    // User Input
    XboxController m_xbox;
};