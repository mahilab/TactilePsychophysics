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
    
    // Method of Constant Stimuli Functions

    Enumerator runMCSExperiment();

    void writeMCSOutputVariables(Csv& csv, Timestamp ts);

    void writeMCSOutputData(Csv& csv, PsychTest::QueryMCS trial);

    // Staircase Method Functions  

    Enumerator runSMExperiment();

    void writeSMOutputVariables(Csv& csv, Timestamp ts);

    void writeSMOutputData(Csv& csv, PsychTest::QuerySM trial);

    // Method of Adjustments Functions

    void responseWindowMA(PsychTest::WhichStim whichStim);

    Enumerator runMAExperiment();
    
    void writeMAOutputVariables(Csv& csv, Timestamp ts);

    void writeMAOutputData(Csv& csv, PsychTest::QueryMA trial);
    
    // Hardware Specific Functions

    void connectToIO();

    void importUserHardwareParams();
    
    void stopExp();

    void calibrate();

    Enumerator bringToStartPosition();

    Enumerator bringToContact();

    void plotDebugExpInfo();

    void lockExtraDofs();

    void setControlDof();

    void setStimulus(double N);

    void userLimitsExceeded();

    void collectSensorData(PsychTest::WhichStim whichStim);

    void avgSensorData();

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

    // Create buffers for calculating the forces and positions during each cue
    mahi::util::RingBuffer<double> m_stim1_normF{50};
    mahi::util::RingBuffer<double> m_stim1_shearF{50};
    mahi::util::RingBuffer<double> m_stim1_normP{50};
    mahi::util::RingBuffer<double> m_stim1_shearP{50};

    mahi::util::RingBuffer<double> m_stim2_normF{50};
    mahi::util::RingBuffer<double> m_stim2_shearF{50};
    mahi::util::RingBuffer<double> m_stim2_normP{50};
    mahi::util::RingBuffer<double> m_stim2_shearP{50};
    
    double m_stim1_avgNormF;
    double m_stim1_avgShearF;
    double m_stim1_avgNormP;
    double m_stim1_avgShearP;

    double m_stim2_avgNormF;
    double m_stim2_avgShearF;
    double m_stim2_avgNormP;
    double m_stim2_avgShearP;

    // Plotting variables
    ScrollingBuffer lockForce, lockPosition, testForce, testPosition, ref, comp, curr;
    float t = 0;
    float m_history = 30.0f;

    // CM
    CMHub m_hub;
    std::shared_ptr<CM> m_cm_test;
    std::shared_ptr<CM> m_cm_lock;
    CM::Params m_paramsTest;
    CM::Params m_paramsLock;

    // User Input
    XboxController m_xbox;
};