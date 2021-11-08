#include <Mahi/Util/Math/Functions.hpp>
#include "PsychophysicalTesting.hpp"
#include <filesystem>
#include <fstream>

// Written by Janelle Clark with Nathan Dunkelberger, based off code by Evan Pezent

using namespace mahi::daq;
using namespace mahi::util;
namespace fs = std::filesystem;

PsychTest::PsychTest(int subnum, Params config, PsychTest::WhichExp whichExp, WhichDof whichdof, ControlType controller) :
    m_jnd_stim_levels(m_params.n_mcs_comparisons, 0)
{
    m_subject = subnum;
    m_whichExp = whichExp;
    m_whichDof = whichdof;
    m_controller = controller;
    if (m_whichExp == PsychTest::MCS){
        m_stim_trials_mcs.reserve(300); // arbitrary, more than we'll need
    } else if (m_whichExp == PsychTest::SM) {
        m_stim_trials_sm.reserve(300); // arbitrary, more than we'll need
    }else if (m_whichExp == PsychTest::MA) {
        m_stim_trials_ma.reserve(300); // arbitrary, more than we'll need
    }
    std::string user_calibration_file = "C:/Git/TactilePsychophysics/calibs/User/subject_" + std::to_string(m_subject) + ".json";
    m_up.importParams(user_calibration_file);
    m_userparams = m_up.getParams();    
    setParams(config);
    LOG(Info) << "Opened PsychTest for subject " << m_subject << ". Set to default settings";
}

PsychTest::~PsychTest(){};

//=============================================================================
// PUBLIC (THREAD SAFE)
//=============================================================================

void PsychTest::setParams(Params config) {
    m_params = config;

    if (m_whichExp == PsychTest::MCS){
        n_jnd_trials_window = m_params.n_mcs_comparisons * m_params.n_mcs_reps * 2; // 180 trials per window
        n_jnd_trials_total  = n_jnd_trials_window * m_params.n_mcs_windows; // 900 trials total
        m_jnd_ref_index     = int(floor(double(m_params.n_mcs_comparisons)/2)); // this is the index since indexing starts at 0
    } else if (m_whichExp == PsychTest::SM) {
        if (m_controller == PsychTest::Position){
            m_jnd_stimulus_interval = m_params.sm_pos_inc; // Question - or defined by subject range??????????????
        } else if (m_controller == PsychTest::Force){
            m_jnd_stimulus_interval = m_params.sm_force_inc; // Question - or defined by subject range??????????????
        }
    }

    
    if (m_whichDof == Shear){
        if (m_controller == Position){
            m_userStimulusContact  = m_userparams.positionCont_t; 
            m_userStimulusMin = m_userparams.positionMin_t;
            m_userStimulusMax = m_maxRangePercent*m_userparams.positionMax_t;
        }else if (m_controller == Force){
            m_userStimulusContact  = m_userparams.forceCont_t; 
            m_userStimulusMin = m_userparams.forceMin_t;
            m_userStimulusMax = m_maxRangePercent*m_userparams.forceMax_t;
        }
    }else if (m_whichDof == Normal){
        if (m_controller == Position){
            m_userStimulusContact  = m_userparams.positionCont_n; 
            m_userStimulusMin = m_userparams.positionMin_n;
            m_userStimulusMax = m_maxRangePercent*m_userparams.positionMax_n;
        }else if (m_controller == Force){
            m_userStimulusContact  = m_userparams.forceCont_n; 
            m_userStimulusMin = m_userparams.forceMin_n;
            m_userStimulusMax = m_maxRangePercent*m_userparams.forceMax_n;
        }
    }
    
    if (m_whichExp == PsychTest::MCS){
        m_jnd_stimulus_reference = 0.75*(m_userStimulusMax - m_userStimulusMin) + m_userStimulusMin;
        m_jnd_stimulus_interval  = 0.02*(m_userStimulusMax - m_userStimulusMin); /// ????????????? base this value on lit/pilots. Based on default param file values if not found yet
        buildComparisonVector();
    } else if (m_whichExp == PsychTest::SM){
        m_jnd_stimulus_reference = 0;
    }
    
}

bool PsychTest::exportParams(const std::string& filepath) {
    fs::path path(filepath);
    PsychTest::Params params = getParams();
    Timestamp ts;
    json j;
    j["n_mcs_comparisons"]      = params.n_mcs_comparisons;
    j["n_mcs_reps"]             = params.n_mcs_reps;
    j["n_mcs_windows"]          = params.n_mcs_windows;
    j["n_sm_staircases"]        = params.n_sm_staircases;
    j["n_sm_reversals"]        = params.n_sm_reversals;
    j["sm_pos_inc"]             = params.sm_pos_inc;
    j["sm_force_inc"]           = params.sm_force_inc;
    j["n_ma_trials"]            = params.n_ma_trials;
    j["stimulus_time"]          = params.stimulus_time;
    j["ramp_time"]              = params.ramp_time;
    j["travel_time"]            = params.travel_time;

    std::ofstream file(path);
    if (file.is_open()) {
        file << std::setw(10) << j;
        LOG(Info) << "Exported PsychTest Subject " << m_subject << " parameters to " << path.generic_string();
        file.close();
        return true;
    }
    LOG(Error) << "Failed to export PsychTest Subject " << m_subject << " parameters because because " << path << " could not be created or opened.";
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

            params.n_mcs_comparisons    = j["n_mcs_comparisons"].get<int>();
            params.n_mcs_reps           = j["n_mcs_reps"].get<int>();
            params.n_mcs_windows        = j["n_mcs_windows"].get<int>(); 
            params.n_sm_staircases      = j["n_sm_staircases"].get<int>();
            params.n_sm_reversals      = j["n_sm_reversals"].get<int>();
            params.sm_pos_inc           = j["sm_pos_inc"].get<double>();
            params.sm_force_inc         = j["sm_force_inc"].get<double>();
            params.n_ma_trials          = j["n_ma_trials"].get<int>();
            params.stimulus_time        = j["stimulus_time"].get<double>();
            params.ramp_time            = j["ramp_time"].get<double>();
            params.travel_time          = j["travel_time"].get<double>();
                
            setParams(params);
            LOG(Info) << "Imported PsychTest Subject " << m_subject << " parameters from " << path.generic_string();
            if (m_subject != m_userparams.subnum)
                LOG(Error) << "Subject number at construction, " << m_subject << "is not the same as in the imported parameters.";

        }
        catch(...) {
            LOG(Error) << "Failed to import PsychTest Subject " << m_subject << " parameters.";
        }

        return true;
    }
    LOG(Error) << "Failed to import PsychTest Subject " << m_subject << " parameters because " << path << " does not exist.";
    return false;
}

PsychTest::Params PsychTest::getParams() const {
    return m_params;
}

void PsychTest::setNormalPositionRange(double min, double max) {
    m_userparams.positionMin_n = min;
    m_userparams.positionMax_n = max;
    LOG(Info) << "Set PsychTest Subject " << m_subject << " normal position range to [ " << min << " , " << max << " mm].";
}

void PsychTest::setShearPositionRange(double min, double max) {
    m_userparams.positionMin_t = min;
    m_userparams.positionMax_t = max;
    LOG(Info) << "Set PsychTest Subject" << m_subject << " shear position range to [ " << min << " , " << max << " mm].";
}

void PsychTest::setNormalForceRange(double min, double max) {
    m_userparams.forceMin_n = min;
    m_userparams.forceMax_n = max;
    LOG(Info) << "Set PsychTest Subject" << m_subject << " normal force range to [ " << min << " , " << max << " N].";
}

void PsychTest::setShearForceRange(double min, double max) {
    m_userparams.forceMin_t = min;
    m_userparams.forceMax_t = max;
    LOG(Info) << "Set PsychTest Subject" << m_subject << " shear force range to [ " << min << " , " << max << " N].";
}

//////////// Method of Constant Stimuli (MCS) Functions ///////////////

void PsychTest::setResponseMCS(int answer, int greater){
    m_q_mcs.answer = answer;
    m_q_mcs.greater = greater;
}

void PsychTest::printQueriesMCS(QueryMCS q) {
    std::cout << "time " << q.time << std::endl;
    std::cout << "testmode " << q.testmode << std::endl; 
    std::cout << "whichExp " << q.whichExp << std::endl;
    std::cout << "whichDof " << q.whichDof << std::endl;
    std::cout << "controller " << q.controller << std::endl;
    std::cout << "trialnum " << q.trialnum << std::endl;
    std::cout << "generated_num " << q.generated_num << std::endl;
    std::cout << "window " << q.window << std::endl; 
    std::cout << "level " << q.level << std::endl;
    std::cout << "stimulus1 " << q.stimulus1 << std::endl;
    std::cout << "stimulus2 " << q.stimulus2 << std::endl;
    std::cout << "standard " << q.standard << std::endl; 
    std::cout << "comparison " << q.comparison << std::endl;
    std::cout << "correct " << q.correct << std::endl;
    std::cout << "answer " << q.answer << std::endl;
    std::cout << "greater " << q.greater << std::endl; 
}

void PsychTest::buildComparisonVector(){

    double min_comparison = m_jnd_stimulus_reference - double(m_jnd_ref_index) * m_jnd_stimulus_interval;
    
    for (int i = 0; i < m_params.n_mcs_comparisons; ++i) { // this writes a vector of length "m_params.n_mcs_comparisons" centered at "m_jnd_stimulus_reference"
        m_jnd_stim_levels[i] = min_comparison + double(i) * m_jnd_stimulus_interval;
    std::cout << "m_jnd_stim_levels[i] " << m_jnd_stim_levels[i] << " i " << i << " ref " << m_jnd_stimulus_reference << std::endl;
    }
    std::cout << "min " << m_userStimulusMin << " max " << m_userStimulusMax << " interval " << m_jnd_stimulus_interval << std::endl;
}

void PsychTest::buildStimTrials() {
    std::cout << "Build Stim Trials" << std::endl;
        std::random_device rd;
        std::mt19937 g(rd());
        g.seed(m_subject);
        int i = 0;
        for (int w = 0; w < m_params.n_mcs_windows; ++w) {
            std::vector<QueryMCS> window_trials;
            window_trials.reserve(n_jnd_trials_window);
            for (int r = 0; r < m_params.n_mcs_reps; ++r) {
                for (int f = 0; f < m_params.n_mcs_comparisons; ++f) {
                    QueryMCS trial;
                    //  standard first
                    trial.testmode   = m_testmode;
                    trial.whichExp   = m_whichExp;
                    trial.whichDof   = m_whichDof; 
                    trial.controller = m_controller;
                    trial.generated_num     = i++;
                    trial.window     = w;
                    trial.level      = f - floor(m_params.n_mcs_comparisons/2); 
                    trial.stimulus1  = m_jnd_stimulus_reference;
                    trial.stimulus2  = m_jnd_stim_levels[f];
                    trial.standard   = 1;
                    trial.comparison = 2;
                    trial.correct    = trial.stimulus1 == trial.stimulus2 ? 0 : trial.stimulus1 > trial.stimulus2 ? 1 : 2;
                    trial.answer     = -1;
                    trial.greater    = -1;
                    window_trials.push_back(trial);
                    // standard second
                    trial.testmode   = m_testmode;
                    trial.whichExp   = m_whichExp;
                    trial.whichDof   = m_whichDof; 
                    trial.controller = m_controller;
                    trial.generated_num     = i++;
                    trial.window     = w;
                    trial.level      = f - floor(m_params.n_mcs_comparisons/2); 
                    trial.stimulus1  = m_jnd_stim_levels[f];
                    trial.stimulus2  = m_jnd_stimulus_reference;
                    trial.standard   = 2;
                    trial.comparison = 1;
                    trial.correct    = trial.stimulus1 == trial.stimulus2 ? 0 : trial.stimulus1 > trial.stimulus2 ? 1 : 2;
                    trial.answer     = -1;
                    trial.greater    = -1;
                    window_trials.push_back(trial);
                }
            }
            // shuffle window trials
            std::shuffle(window_trials.begin(), window_trials.end(), g);
            // append
            m_stim_trials_mcs.push_back(window_trials);
        }
        // assign trial numbers
        i = 0;
        for (int w = 0; w < m_params.n_mcs_windows; ++w) {
            for (auto& trial : m_stim_trials_mcs[w])
                trial.trialnum = i++;
        }
    }

//////////// Staircase Method (SM) Functions ///////////////

void PsychTest::setResponseSM(int answer, int greater){

    // Start filling in and save query for finished trial
    m_q_sm.answer = answer;
    m_q_sm.greater = greater;
    
    double lastComp = (m_q_sm.comparison == 1) ? m_q_sm.stimulus1 : m_q_sm.stimulus2;
    // Determine if the next trial has a larger or smaller comparison (if right, toward the std, if wrong, away from it)
    if (m_q_sm.answer == m_q_sm.correct){ // answered correctly
        m_jnd_stimulus_comparison = (lastComp > m_jnd_stimulus_reference) ? (lastComp - m_jnd_stimulus_interval) : (lastComp + m_jnd_stimulus_interval);
    }else{ // answered incorrectly
        m_jnd_stimulus_comparison = (lastComp > m_jnd_stimulus_reference) ? (lastComp + m_jnd_stimulus_interval) : (lastComp - m_jnd_stimulus_interval);
    }
    // If it's an absolute threshold experiment, don't allow the comparison to be negative
    if (m_jnd_stimulus_reference == 0 && m_jnd_stimulus_comparison < 0){
        m_jnd_stimulus_comparison = abs(m_jnd_stimulus_comparison);
        // If you get stuck here with infinite comparisons of this value, the increment should be reduced
    }

    // If it's force control and normal testing, don't let force go below contact force
    if ((m_whichDof == Normal) && (m_controller == Force) && (m_jnd_stimulus_comparison < m_userStimulusContact)){
        m_jnd_stimulus_comparison = m_userStimulusContact;
        // If this is above the threshold, then adjust the contact force value
        // If you get stuck here with infinite comparisons of this value, the increment should be reduced
    }

    // clamp at stimulus max and stimulus min (comfort threshold and absolute threshold (or 0 if determining it))
    if((m_jnd_stimulus_comparison < m_userStimulusMin) || (m_jnd_stimulus_comparison > m_userStimulusMax)){
        if(m_controller==PsychTest::Position)
            LOG(Warning) << "Comparison value " << m_jnd_stimulus_comparison <<" mm clamped by stimulus position thresholds " << m_userStimulusMin << " mm and " << m_userStimulusMax  << "mm";
        else if (m_controller==PsychTest::Force)
            LOG(Warning) << "Comparison value " << m_jnd_stimulus_comparison <<" N clamped by stimulus force thresholds " << m_userStimulusMin << " N and " << m_userStimulusMax  << "N";
    }
    m_jnd_stimulus_comparison = clamp(m_jnd_stimulus_comparison, m_userStimulusMin, m_userStimulusMax);
    
    // Determine the direction change since last trial
    if (m_jnd_stimulus_comparison > lastComp)
        m_direction = Up;
    else if (m_jnd_stimulus_comparison < lastComp)
        m_direction = Down;
    else
        m_direction = None;

    // Determine if a reversal has taken place
    // Question ----- Actually, reversal was the last query? overwrite isReversal there?????????????????????????
    m_isReversal = m_direction != m_lastSlope && m_direction !=None && m_lastSlope !=None;
        
    if(m_direction !=None){
        m_lastSlope = m_direction;
    }
    
    m_num_reversal += (m_isReversal ? 1 : 0); // if a reveral has taken place increment the number of reversals

    // Finish filling in and save query for finished trial
    m_q_sm.num_reversal  = m_num_reversal;
    m_q_sm.isReversal = m_isReversal;
    m_stim_trials_sm.push_back(m_q_sm);

    std::cout << "____________________________" << std::endl;
    std::cout << "Last Query" << std::endl;
    printQueriesSM(m_q_sm); // ?????????????????????????????????????????
    std::cout << " " << std::endl;

    m_q_sm.testmode = m_testmode;
}

void PsychTest::printQueriesSM(QuerySM q) {
    std::cout << "time " << q.time << std::endl;
    std::cout << "testmode " << q.testmode << std::endl; 
    std::cout << "whichExp " << expchoice[q.whichExp] << std::endl;
    std::cout << "whichDof " << dofChoice[q.whichDof] << std::endl;
    std::cout << "controller " << controlChoice[q.controller] << std::endl;
    std::cout << "trialnum " << q.trialnum << std::endl;
    std::cout << "num_staircase " << q.num_staircase << std::endl;
    std::cout << "direction " << currdirection[q.direction] << std::endl; 
    std::cout << "lastSlope " << currdirection[q.lastSlope] << std::endl; 
    std::cout << "num_reversal " << q.num_reversal << std::endl;
    std::cout << "isReversal " << q.isReversal << std::endl;
    std::cout << "stimulus1 " << q.stimulus1 << std::endl;
    std::cout << "stimulus2 " << q.stimulus2 << std::endl;
    std::cout << "standard " << q.standard << std::endl; 
    std::cout << "comparison " << q.comparison << std::endl;
    std::cout << "correct " << q.correct << std::endl;
    std::cout << "answer " << q.answer << std::endl;
    std::cout << "greater " << q.greater << std::endl; 
}

void PsychTest::setNextTrialSM(){
    // randomly determine order of comparison and reference and starting comparison
    int n_randBool = 5; // final vector will be twice this length, if not 5, also update the ==9 term at the end of the loop
    static std::random_device rd;
    static std::mt19937 g(rd());
    static std::vector<bool>   m_stdFirst;

    if (isNewStair){
        isNewStair = 0;
        // order of reference and comparison values
        for (size_t i = 0; i < n_randBool; i++)
        {
            m_stdFirst.push_back(true);
            m_stdFirst.push_back(false);
        }
        std::shuffle(m_stdFirst.begin(),m_stdFirst.end(),g);

        // intitial starting value
        double maxStartVal = 0.15*m_userStimulusMax; // start between 10 and 15% of their max value
        double minStartVal = 0.1*m_userStimulusMax; // start between 10 and 15% of their max value
        int n_startComps = floor((maxStartVal-minStartVal)/m_jnd_stimulus_interval); // divide the range into chunks by the step-size from the params
        std::cout << "n_startComps: " << n_startComps << std::endl; // ????????????????????????????????????
        double maxStartComp = n_startComps*m_jnd_stimulus_interval + minStartVal; // max start value as a multiple of the interval
        std::vector<double> startComps(n_startComps);
        startComps[0] = minStartVal; 
        for (int i = 1; i < n_startComps; i++){
            startComps[i] = startComps[i-1] + m_jnd_stimulus_interval;
        }
        shuffle(startComps.begin(),startComps.end(),g);

        m_jnd_stimulus_comparison = startComps[1]; //randomly decide starting point by first index in the shuffle
        m_num_reversal = 0;
        m_lastSlope = None;

    }else if(m_q_sm.answer == -1){
            LOG(Error) << "Response for last trial is not recorded, unable to continue.";
    } // is it a new stair?

    // index for trial order boolean (standard or comparison first)
    auto num_0_9 = m_trialnum%(n_randBool*2);
    
    // Update m_q_sm query to puch to vector for the next trial
    m_trialnum++;

    int time = 0;
    m_q_sm.testmode      = m_testmode;
    m_q_sm.whichExp      = m_whichExp;
    m_q_sm.whichDof      = m_whichDof;
    m_q_sm.controller    = m_controller;
    m_q_sm.trialnum      = m_trialnum;
    m_q_sm.num_staircase = m_num_staircase;
    m_q_sm.direction     = m_direction;
    m_q_sm.lastSlope     = m_lastSlope;
    m_q_sm.num_reversal  = m_num_reversal;
    m_q_sm.isReversal    = -1;
    m_q_sm.stimulus1     = m_stdFirst[num_0_9] ? m_jnd_stimulus_reference : m_jnd_stimulus_comparison;
    m_q_sm.stimulus2     = m_stdFirst[num_0_9] ? m_jnd_stimulus_comparison : m_jnd_stimulus_reference;
    m_q_sm.standard      = m_stdFirst[num_0_9] ? 1 : 2;
    m_q_sm.comparison    = m_stdFirst[num_0_9] ? 2 : 1;
    m_q_sm.correct       = m_q_sm.stimulus1 == m_q_sm.stimulus2 ? 0 : m_q_sm.stimulus1 > m_q_sm.stimulus2 ? 1 : 2;
    m_q_sm.answer        = -1;
    m_q_sm.greater       = -1;
    
    // reshuffle trial order vector every 10 trials
    if (num_0_9 == 9){
        std::shuffle(m_stdFirst.begin(),m_stdFirst.end(),g);
    }
}

void PsychTest::setNewStaircase(){
    // reinitialize variables
    m_trialnum = 0;
    m_direction = None;
    m_lastSlope = None;
    m_num_reversal = 0;
    m_isReversal = 0;
    m_jnd_stimulus_comparison = 0;

    //increment num of staircases
    m_num_staircase++;
    isNewStair = 1;
}

//////////// Method of Adjustments (MA) Functions ///////////////

void PsychTest::setResponseMA(int adjust, double submittedVal){
    m_adjust = adjust;
    m_change = submittedVal - m_jnd_stimulus_comparison;
    m_finalVal = submittedVal;

    // Determine the direction change since last trial
    if (submittedVal > m_jnd_stimulus_comparison)
        m_direction = Up;
    else if (submittedVal < m_jnd_stimulus_comparison)
        m_direction = Down;
    else
        m_direction = None;
    
    m_q_ma.adjust = m_adjust;
    m_q_ma.change = m_change;
    m_q_ma.direction = m_direction;
    m_q_ma.finalVal = m_finalVal;
    m_stim_trials_ma.push_back(m_q_ma);
}

void PsychTest::printQueriesMA(QueryMA q) {
    std::cout << "time " << q.time << std::endl;
    std::cout << "testmode " << q.testmode << std::endl; 
    std::cout << "whichExp " << expchoice[q.whichExp] << std::endl;
    std::cout << "whichDof " << dofChoice[q.whichDof] << std::endl;
    std::cout << "controller " << controlChoice[q.controller] << std::endl;
    std::cout << "trialnum " << q.trialnum << std::endl;
    std::cout << "stimulus1 " << q.stimulus1 << std::endl;
    std::cout << "stimulus2 " << q.stimulus2 << std::endl;
    std::cout << "standard " << q.standard << std::endl; 
    std::cout << "comparison " << q.comparison << std::endl;
    std::cout << "adjust " << q.adjust << std::endl;
    std::cout << "direction " << currdirection[q.direction] << std::endl;
    std::cout << "change " << q.change << std::endl;
    std::cout << "finalVal " << q.finalVal << std::endl; 
}

void PsychTest::setNextTrialMA(){
    static std::random_device rd;
    static std::mt19937 g(rd());
    static std::vector<double> startComps;
    static int j(0);
    
    // determine initial starting point
    double maxStartVal;
    if (m_trialnum != 0){
        if(m_q_ma.adjust == -1)
            LOG(Error) << "Response for last trial is not recorded, unable to continue.";  
    }else{
        if (m_controller == PsychTest::Position){
            m_jnd_stimulus_interval = m_params.sm_pos_inc; // Question - or defined by subject range??????????????
            maxStartVal = 7; // start at less than a point torable by everyone ??????????????????? determine through pilots
        } else if (m_controller == PsychTest::Force){
            m_jnd_stimulus_interval = m_params.sm_force_inc; // Question - or defined by subject range??????????????
            maxStartVal = 7; // start at less than a point torable by everyone ??????????????????? determine through pilots
        }

        // intitial starting value options
        int n_startComps = floor(maxStartVal/m_jnd_stimulus_interval); // divide the range into chunks by the step-size from the params
        double maxStartComp = n_startComps*m_jnd_stimulus_interval; // max start value as a multiple of the interval
        startComps.resize(n_startComps);
        startComps[0] = m_jnd_stimulus_interval; 
        for (int i = 1; i < n_startComps; i++){
            startComps[i] = startComps[i-1] + m_jnd_stimulus_interval;
        }

        shuffle(startComps.begin(),startComps.end(),g);
    }

    // Randomly choose start value from the vector created above
    m_jnd_stimulus_comparison = startComps[j]; 

    // Update m_q_sm query to puch to vector for the next trial 
    m_trialnum++;
    m_q_ma.time          = 0;
    m_q_ma.testmode      = m_testmode;
    m_q_ma.whichExp      = m_whichExp;
    m_q_ma.whichDof      = m_whichDof;
    m_q_ma.controller    = m_controller;
    m_q_ma.trialnum      = m_trialnum;
    m_q_ma.stimulus1     = m_jnd_stimulus_reference;
    m_q_ma.stimulus2     = m_jnd_stimulus_comparison;
    m_q_ma.standard      = 1;
    m_q_ma.comparison    = 2;
    m_q_ma.adjust        = -1;
    m_q_ma.direction     = None;
    m_q_ma.change        = 0;
    m_q_ma.finalVal      = -1;

    // reshuffle trial order vector every 10 trials
    if (j == startComps.size()){
        std::shuffle(startComps.begin(),startComps.end(),g);
    }
    j++;
}
