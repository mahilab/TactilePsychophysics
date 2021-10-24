#include "PsychGui.hpp"

using namespace mahi::gui;
using namespace mahi::util;
using namespace mahi::robo;
using namespace xbox;

PsychGui::PsychGui(int subject, PsychTest::WhichExp whichExp, PsychTest::WhichDof whichDOF, PsychTest::ControlType controller) : 
    Application(600,600,"Capstan Module Psychophysical Test (Subject " + std::to_string(subject) + ")" ,false), 
    m_pt(subject, PsychTest::Params(), whichExp, whichDOF, controller) // Hardware specific
    {          
        connectToIO();
        importUserHardwareParams();
        set_frame_limit(90_Hz);
    }

    PsychGui::~PsychGui() {
        stopExp();
    }

    void PsychGui::update() {

        ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{600,1000}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::BeginDisabled(m_pt.m_testmode != PsychTest::Idle);

        // Flags to walk through menus
        static bool flag_confirm_exp_settings = 0;
        static bool flag_start_calibration = 0;
        static bool flag_bring_to_start = 0;
        static bool flag_study_started = 0;

        // Confirm experiment settings and subject number
        if (!flag_confirm_exp_settings){
            ImGui::BulletText(("You've intiated a " + m_pt.method[m_pt.m_whichExp] + " Experiment with the following settings:").c_str());
            ImGui::BulletText("For Subject %i",  m_pt.m_subject);
            ImGui::BulletText("Testing the %s Direction", m_pt.dofChoice[m_pt.m_whichDof]); // Hardware specific
            ImGui::BulletText("Using %s Control", m_pt.controlChoice[m_pt.m_controller]);   // Hardware specific
            ImGui::Separator();
            ImGui::SetNextItemWidth(100);

            if(m_pt.m_whichExp == PsychTest::MA){
                if (ImGui::Button("Confirm Settings, Start JND",ImVec2(-1,0))) {
                    m_flag_is_JND = 1;
                    flag_confirm_exp_settings = 1;
                }
                ImGui::SetNextItemWidth(100);
                if (ImGui::Button("Confirm Settings, Start Abs. Threshold",ImVec2(-1,0))) {
                    flag_confirm_exp_settings = 1;
                }
            }else{
                if (ImGui::Button("Confirm Settings",ImVec2(-1,0))) {
                    flag_confirm_exp_settings = 1;
                }
            }
            
            ImGui::SetNextItemWidth(100);
            if (ImGui::Button("Cancel",ImVec2(-1,0))) {
                stopExp(); 
            }
        }

        // after settings are confirmed, calibrate device
        if (flag_confirm_exp_settings && !flag_start_calibration){
            ImGui::Text("Move the end-effector all the way toward the shoulder and");
            ImGui::Text(" about an inch above the skin for calibration.");
            if (ImGui::Button("Calibrate",ImVec2(-1,0))){
                calibrate(); 
                flag_start_calibration = 1;
            }
        }
        
        // start experiment or cancel for some reason
        if (flag_start_calibration && !flag_study_started){
            if (ImGui::Button("Start Study",ImVec2(-1,0))){
                // Set controllers for testing DOF
                start_coroutine(lockExtraDofs());
                start_coroutine(setControlDof());

                if (m_pt.m_whichExp == PsychTest::MCS){
                    m_pt.buildStimTrials();
                    start_coroutine(runMCSExperiment());
                }else if (m_pt.m_whichExp == PsychTest::SM){
                    m_pt.setNextTrialSM();
                    start_coroutine(runSMExperiment());
                }else if (m_pt.m_whichExp == PsychTest::MA){
                    m_pt.setNextTrialMA();
                    start_coroutine(runMAExperiment());
                }
                
                flag_study_started = 1;
            }
            
            ImGui::Checkbox("Debug Mode", &m_debug);

            ImGui::SetNextItemWidth(100);
            if (ImGui::Button("Cancel",ImVec2(-1,0))) {
                stopExp();
            }
        }
        ImGui::EndDisabled();

        // idle mode
        if (m_pt.m_testmode == PsychTest::Idle) {
            //When not coducting trials, go to neutral contact position
            if(flag_start_calibration){
                start_coroutine(bringToStartPosition());
            }
        }
        ImGui::End();     
    }

    int PsychGui::responseWindow(PsychTest::WhichStim whichStim) {
        float windowHeight = m_debug ? 1500.0 : -1;
        float windowWidth = m_debug ? 600.0 : -1;
        float buttonHeight = m_debug ? 50.0 : -1;

        ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{windowWidth,windowHeight}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::BeginDisabled(true,1.0f);
        int ans = -1; // default value, means nothing

        if (whichStim == PsychTest::Choose)
            ImGui::PushStyleColor(ImGuiCol_Button, Blues::DeepSkyBlue);
        ImGui::Button("Which Stimulus Was Larger?", ImVec2(-1,0));
        if (whichStim == PsychTest::Choose)
            ImGui::PopStyleColor();

        if(whichStim != PsychTest::Choose) {
            
            if (whichStim == PsychTest::First)
                ImGui::PushStyleColor(ImGuiCol_Button, Blues::DeepSkyBlue);
            ImGui::Button("First", ImVec2(290,buttonHeight));
            if (whichStim == PsychTest::First)
                ImGui::PopStyleColor();

            ImGui::SameLine();

            if (whichStim == PsychTest::Second)
                ImGui::PushStyleColor(ImGuiCol_Button, Blues::DeepSkyBlue);
            ImGui::Button("Second", ImVec2(290,buttonHeight));   
            if (whichStim == PsychTest::Second)
                ImGui::PopStyleColor();
                
            ImGui::EndDisabled();

        }else if (whichStim == PsychTest::Choose){

            ImGui::EndDisabled();

            if (whichStim == PsychTest::Choose){
                if (ImGui::Button("First", ImVec2(290,buttonHeight)) || m_xbox.is_button_pressed(XboxController::A))
                    ans = 1; // means the first stimulus is larger
                ImGui::SameLine();
                if (ImGui::Button("Second", ImVec2(290,buttonHeight)) || m_xbox.is_button_pressed(XboxController::B))
                    ans = 2; // means the second stimulus is larger
            }
        }

        //plot force and position information
        if (m_debug){
            plotDebugExpInfo();
        }

        ImGui::End();
        return ans;
    }


    void PsychGui::rampStimulus(double start, double end, double ramptime, double elapsed){
        double stimVal = Tween::Linear(start, end, (float)(elapsed / ramptime));
        PsychGui::setStimulus(stimVal);
    }

    void PsychGui::rampLock(double start, double end, double ramptime, double elapsed){
        double stimVal = Tween::Linear(start, end, (float)(elapsed / ramptime));
        PsychGui::setLock(stimVal);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    //           Method of Constant Stimuli Specific Functions
    //////////////////////////////////////////////////////////////////////////////////////

    Enumerator PsychGui::runMCSExperiment() {
        m_pt.m_testmode = PsychTest::Run;
        Timestamp ts;
        
        std::string filename;
        filename = "C:/Git/TactilePsychophysics/data/_jnd_subject_" + std::to_string(m_pt.m_subject) + "_" + m_pt.dofChoice[m_pt.m_whichDof] + "_" + m_pt.controlChoice[m_pt.m_controller] + "_" + m_pt.expchoice[m_pt.m_whichExp] + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
        Csv csv(filename);
        
        //Create data file
        writeMCSOutputVariables(csv,ts);

        // Run trials
        for (int w = 0; w < m_psychparams.n_mcs_windows; ++w) {
            set_window_title("CM " + m_pt.method[m_pt.m_whichExp] + " (Subject " + std::to_string(m_pt.m_subject) + ") (Test " + m_pt.dofChoice[m_pt.m_whichDof] + " dof, " + m_pt.controlChoice[m_pt.m_controller] + " Control) (S" + std::to_string(w+1) + ")"); // Hardware specific

            // beginning delay
            double elapsed = 0;
            while (elapsed < 1) {
                responseWindow(PsychTest::NA);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
            for (auto& trial : m_pt.m_stim_trials_mcs[w]) {
                ////////////////////////////////////// run trial
                m_pt.m_q_mcs = trial;
                m_pt.m_jnd_stimulus_comparison = m_pt.m_q_mcs.comparison == 1 ? m_pt.m_q_mcs.stimulus1 : m_pt.m_q_mcs.stimulus2;
                // render first stimulus - ramp up
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_mcs.stimulus1, m_psychparams.stimulus_time, elapsed);
                    responseWindow(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 

                // render first stimulus - hold stim and record position and force info
                setStimulus(m_pt.m_q_mcs.stimulus1);
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    responseWindow(PsychTest::First);
                    // collect values while the stimulus is held
                    collectSensorData(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }

                // render first stimulus - ramp down
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    rampStimulus(m_pt.m_q_mcs.stimulus1, m_pt.m_userStimulusContact, m_psychparams.stimulus_time, elapsed);
                    responseWindow(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                setStimulus(m_pt.m_userStimulusContact);
                // first delay
                elapsed = 0;
                while (elapsed < 0.5) {
                    responseWindow(PsychTest::NA);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // render second stimulus - ramp up
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_mcs.stimulus2, m_psychparams.stimulus_time, elapsed);
                    responseWindow(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                // render second stimulus - hold stim and record position and force info
                setStimulus(m_pt.m_q_mcs.stimulus2);
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    responseWindow(PsychTest::Second);
                    //collect values while the stimulus is held
                    collectSensorData(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // Average of the force and position values
                avgSensorData();

                // render second stimulus - ramp down
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    rampStimulus(m_pt.m_q_mcs.stimulus2, m_pt.m_userStimulusContact, m_psychparams.stimulus_time, elapsed);
                    responseWindow(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // End at contact point
                setStimulus(m_pt.m_userStimulusContact);

                // collect response
                while (true) {
                    int answer = responseWindow(PsychTest::Choose);
                    if (answer != -1) {
                        int greater = m_pt.m_q_mcs.comparison == answer ? 1 : 0;
                        m_pt.setResponseMCS(answer,greater);

                        // Write Output Data
                        writeMCSOutputData(csv, m_pt.m_q_mcs);

                        break;
                    }
                    co_yield nullptr;
                }

                // second delay
                elapsed = 0;
                while (elapsed < 1) {
                    responseWindow(PsychTest::NA);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
            }      

            if (w < (m_psychparams.n_mcs_windows - 1)) {
                double remaining = 10;
                while (remaining > 0) {
                    ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
                    ImGui::Text("Break (%.3f)", remaining);
                    ImGui::End();
                    remaining -= delta_time().as_seconds();
                    co_yield nullptr;
                } 
            }
        }
        m_pt.m_testmode = PsychTest::Idle;
        set_window_title("CM " + m_pt.method[m_pt.m_whichExp] + " (Subject " + std::to_string(m_pt.m_subject) + ") (Test " + m_pt.dofChoice[m_pt.m_whichDof] + " dof, " + m_pt.controlChoice[m_pt.m_controller] + " Control)"); // Hardware specific
    }
    
    void PsychGui::writeMCSOutputVariables(Csv& csv, Timestamp ts){
        csv.write_row("Mode","Dof","ControlType","Trial","GenNum","Window","Level","Stimulus1","Stimulus2","Standard","Comparison","Correct","Answer","Greater","NormF1", "ShearF1", "NormP1", "ShearP1","NormF2", "ShearF2", "NormP2", "ShearP2" );
    }
    
    void PsychGui::writeMCSOutputData(Csv& csv, PsychTest::QueryMCS trial){
        csv.write_row(trial.testmode, trial.whichDof, trial.controller, trial.trialnum, trial.generated_num, trial.window, trial.level, trial.stimulus1, trial.stimulus2, trial.standard, trial.comparison, trial.correct, trial.answer, trial.greater, m_stim1_avgNormF, m_stim1_avgShearF, m_stim1_avgNormP, m_stim1_avgShearP, m_stim2_avgNormF, m_stim2_avgShearF, m_stim2_avgNormP, m_stim2_avgShearP);
    }
 
    //////////////////////////////////////////////////////////////////////////////////////
    //           Staircase Method Specific Functions
    //////////////////////////////////////////////////////////////////////////////////////          

    Enumerator PsychGui::runSMExperiment() {
        Timestamp ts;

        while (m_pt.m_q_sm.num_staircase < m_psychparams.n_sm_staircases){ 
            m_pt.m_testmode = PsychTest::Run;
        
            std::string filename;
            filename = "C:/Git/TactilePsychophysics/data/_jnd_subject_" + std::to_string(m_pt.m_subject) + "_" + m_pt.dofChoice[m_pt.m_whichDof] + "_" + m_pt.controlChoice[m_pt.m_controller] + "_" + m_pt.expchoice[m_pt.m_whichExp] + "_num" + std::to_string(m_pt.m_q_sm.num_staircase) + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
            Csv csv(filename);
            
            //Create data file
            writeSMOutputVariables(csv,ts);

            set_window_title("CM " + m_pt.method[m_pt.m_whichExp] + " Num " + std::to_string(m_pt.m_q_sm.num_staircase) + " (Subject " + std::to_string(m_pt.m_subject) + ") (Test " + m_pt.dofChoice[m_pt.m_whichDof] + " dof, " + m_pt.controlChoice[m_pt.m_controller] + " Control)"); // Hardware specific
            
            // beginning delay
            double elapsed = 0;
            while (elapsed < 1) {
                responseWindow(PsychTest::NA);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }

            // run trials
            while (m_pt.m_q_sm.num_reversal < m_psychparams.n_sm_crossovers){

                // render first stimulus - ramp up
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_sm.stimulus1, m_psychparams.stimulus_time, elapsed);
                    responseWindow(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                // render first stimulus - hold stim and record position and force info
                setStimulus(m_pt.m_q_sm.stimulus1);
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    responseWindow(PsychTest::First);
                    // collect values while the stimulus is held
                    collectSensorData(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }

                // render first stimulus - ramp down
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    rampStimulus(m_pt.m_q_sm.stimulus1, m_pt.m_userStimulusContact, m_psychparams.stimulus_time, elapsed);
                    responseWindow(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                setStimulus(m_pt.m_userStimulusContact);
                // first delay
                elapsed = 0;
                while (elapsed < 0.5) {
                    responseWindow(PsychTest::NA);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // render second stimulus - ramp up
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_sm.stimulus2, m_psychparams.stimulus_time, elapsed);
                    responseWindow(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                // render second stimulus - hold stim and record position and force info
                setStimulus(m_pt.m_q_sm.stimulus2);
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    responseWindow(PsychTest::Second);
                    //collect values while the stimulus is held
                    collectSensorData(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // Average of the force and position values
                avgSensorData();

                // render second stimulus - ramp down
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    rampStimulus(m_pt.m_q_sm.stimulus2, m_pt.m_userStimulusContact, m_psychparams.stimulus_time, elapsed);
                    responseWindow(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // End at contact point
                setStimulus(m_pt.m_userStimulusContact);

                // collect response
                while (true) {
                    int answer = responseWindow(PsychTest::Choose);
                    if (answer != -1) {
                        int greater = m_pt.m_q_sm.comparison == answer ? 1 : 0;
                        m_pt.setResponseSM(answer,greater);

                        // Write Output Data
                        writeSMOutputData(csv, m_pt.m_q_sm);

                        break;
                    }
                    co_yield nullptr;
                }

                m_pt.setNextTrialSM(); 

                // second delay
                elapsed = 0;
                while (elapsed < 1) {
                    responseWindow(PsychTest::NA);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                
            } // while - under the number of reversals      

            // 10s break between staircases
            double remaining = 10;
            while (remaining > 0) {
                ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
                ImGui::Text("Staircase Complete");
                ImGui::Text("Break (%.3f)", remaining);
                ImGui::End();
                remaining -= delta_time().as_seconds();
                co_yield nullptr;
            } 
            m_pt.m_testmode = PsychTest::Idle;
            set_window_title("CM " + m_pt.method[m_pt.m_whichExp] + " Num " + std::to_string(m_pt.m_q_sm.num_staircase) + " (Subject " + std::to_string(m_pt.m_subject) + ") (Test " + m_pt.dofChoice[m_pt.m_whichDof] + " dof, " + m_pt.controlChoice[m_pt.m_controller] + " Control)"); // Hardware specific
            m_pt.setNewStaircase(); // staircase num is incremented inside here while preparing settings for next trial
            m_pt.setNextTrialSM();
       } // while under the number of staircases

        double remaining = 10;
        while (remaining > 0) {
            ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            ImGui::Text("Experiment Complete, Notify the Experimentor to Remove You from the Device");
            ImGui::End();
            remaining -= delta_time().as_seconds();
            co_yield nullptr;
        } 
    } // run experiment
    
    void PsychGui::writeSMOutputVariables(Csv& csv, Timestamp ts){
        csv.write_row("StairNum","Mode","Dof","ControlType","Trial","Dir","lastSlope","CrossNum","Stimulus1","Stimulus2","Standard","Comparison","Correct","Answer","Greater","NormF1", "ShearF1", "NormP1", "ShearP1","NormF2", "ShearF2", "NormP2", "ShearP2" );
    }

    void PsychGui::writeSMOutputData(Csv& csv, PsychTest::QuerySM trial){
        csv.write_row(trial.num_staircase, trial.testmode, trial.whichDof, trial.controller, trial.trialnum, trial.direction, trial.lastSlope, trial.num_reversal, trial.stimulus1, trial.stimulus2, trial.standard, trial.comparison, trial.correct, trial.answer, trial.greater, m_stim1_avgNormF, m_stim1_avgShearF, m_stim1_avgNormP, m_stim1_avgShearP, m_stim2_avgNormF, m_stim2_avgShearF, m_stim2_avgNormP, m_stim2_avgShearP);
    } 

    //////////////////////////////////////////////////////////////////////////////////////
    //           Method of Adjustments Functions
    //////////////////////////////////////////////////////////////////////////////////////
     
     void PsychGui::responseWindowMA(PsychTest::WhichStim whichStim) {
        float windowHeight = m_debug ? 1500.0 : -1;
        float windowWidth = m_debug ? 600.0 : -1;
        float buttonHeight = m_debug ? 50.0 : -1;

        ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{windowWidth,windowHeight}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::BeginDisabled(true,1.0f);
        static bool flag_chooseAdjust = 0;

        if (whichStim == PsychTest::Choose)
            ImGui::PushStyleColor(ImGuiCol_Button, Blues::DeepSkyBlue);
        if(!flag_chooseAdjust){
            if(m_flag_is_JND)
                ImGui::Button("Is the second stimulus the same as the first?", ImVec2(-1,0));
            else
                ImGui::Button("Is the current value at your maximum threshold?", ImVec2(-1,0));
        }else
            ImGui::Button("Adjust the value to your maximum threshold", ImVec2(-1,0));

        if (whichStim == PsychTest::Choose)
            ImGui::PopStyleColor();

        if(!m_flag_presentStims) {            
            if (whichStim == PsychTest::First)
                ImGui::PushStyleColor(ImGuiCol_Button, Blues::DeepSkyBlue);
            ImGui::Button("First", ImVec2(290,buttonHeight));
            if (whichStim == PsychTest::First)
                ImGui::PopStyleColor();

            ImGui::SameLine();

            if (whichStim == PsychTest::Second)
                ImGui::PushStyleColor(ImGuiCol_Button, Blues::DeepSkyBlue);
            ImGui::Button("Second", ImVec2(290,buttonHeight));   
            if (whichStim == PsychTest::Second){
                ImGui::PopStyleColor(); 
            } 
            ImGui::EndDisabled();
        }

        if (whichStim == PsychTest::Choose && !flag_chooseAdjust){ 

            ImGui::EndDisabled();

            if (ImGui::Button("Yes, Confirm Value", ImVec2(290,buttonHeight)) || m_xbox.is_button_pressed(XboxController::A)){
                m_adjust = 0; // the stimuli feel the same, finish trial.
                m_flag_reachedMAValue = 1;
            }
            ImGui::SameLine();
            if (ImGui::Button("No, Adjust Value", ImVec2(290,buttonHeight)) || m_xbox.is_button_pressed(XboxController::B)){
                // the stimuli don't feel the same, need to adjust
                flag_chooseAdjust = 1;
            }
        }

        if (flag_chooseAdjust && !m_flag_reachedMAValue){

            ImGui::EndDisabled();

            m_whatChange = 0;
            if (ImGui::Button("Increase Value", ImVec2(290,buttonHeight)) || m_xbox.is_button_pressed(XboxController::Up)){
                m_whatChange = 1;
            }
            ImGui::SameLine();
            if (ImGui::Button("Decrease Value", ImVec2(290,buttonHeight)) || m_xbox.is_button_pressed(XboxController::Down)){
                m_whatChange = -1;
            }
            ImGui::Separator();

            if (ImGui::Button("Submit Value", ImVec2(290,buttonHeight)) || m_xbox.is_button_pressed(XboxController::A) || m_xbox.is_button_pressed(XboxController::B)){
                m_adjust = 1; // yes, and done
                m_flag_reachedMAValue = 1;
                flag_chooseAdjust = 0;
            }                  
        }

        //plot force and position information
        if (m_debug){
            plotDebugExpInfo();
        }

        ImGui::End();
    }
    
    Enumerator PsychGui::runMAExperiment(){
        Timestamp ts;
 
        m_pt.m_testmode = PsychTest::Run;
    
        std::string filename;
        filename = "C:/Git/TactilePsychophysics/data/_jnd_subject_" + std::to_string(m_pt.m_subject) + "_" + m_pt.dofChoice[m_pt.m_whichDof] + "_" + m_pt.controlChoice[m_pt.m_controller] + "_" + m_pt.expchoice[m_pt.m_whichExp] + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
        Csv csv(filename);
        
        //Create data file
        writeMAOutputVariables(csv,ts);

        set_window_title("CM " + m_pt.method[m_pt.m_whichExp] + " (Subject " + std::to_string(m_pt.m_subject) + ") (Test " + m_pt.dofChoice[m_pt.m_whichDof] + " dof, " + m_pt.controlChoice[m_pt.m_controller] + " Control)"); // Hardware specific
        

        // run trials
        while (m_pt.m_q_ma.trialnum < m_psychparams.n_ma_trials){

            m_jnd_current_stimulus = m_pt.m_q_ma.stimulus2;
            double elapsed = 0;
            if (m_flag_is_JND){
                // beginning delay
                
                while (elapsed < 1) {
                    responseWindowMA(PsychTest::NA);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }

                // render first stimulus - ramp up
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_ma.stimulus1, m_psychparams.stimulus_time, elapsed);
                    responseWindowMA(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                // render first stimulus - hold stim and record position and force info
                setStimulus(m_pt.m_q_ma.stimulus1);
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    responseWindowMA(PsychTest::First);
                    // collect values while the stimulus is held
                    collectSensorData(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }

                // render first stimulus - ramp down
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    rampStimulus(m_pt.m_q_ma.stimulus1, m_pt.m_userStimulusContact, m_psychparams.stimulus_time, elapsed);
                    responseWindowMA(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                setStimulus(m_pt.m_userStimulusContact);
                // first delay
                elapsed = 0;
                while (elapsed < 0.5) {
                    responseWindowMA(PsychTest::NA);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // render second stimulus - ramp up
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_ma.stimulus2, m_psychparams.stimulus_time, elapsed);
                    responseWindowMA(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                // render second stimulus - hold stim and record position and force info
                setStimulus(m_pt.m_q_ma.stimulus2);
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    responseWindowMA(PsychTest::Second);
                    //collect values while the stimulus is held
                    collectSensorData(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // Average of the force and position values
                avgSensorData();

                // render second stimulus - ramp down
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    rampStimulus(m_pt.m_q_ma.stimulus2, m_pt.m_userStimulusContact, m_psychparams.stimulus_time, elapsed);
                    responseWindowMA(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // End at contact point
                setStimulus(m_pt.m_userStimulusContact); 

            } // end two stimuli in JND case

            m_flag_presentStims = 1;

            // second delay
            elapsed = 0;
            while (elapsed < 1) {
                responseWindowMA(PsychTest::NA);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }

            // render comparison stimulus for adjustment - ramp up
            elapsed = 0;
            while (elapsed < m_psychparams.stimulus_time) {
                rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_ma.stimulus2, m_psychparams.stimulus_time, elapsed);
                responseWindowMA(PsychTest::Choose);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            } 

            // collect response and adjust value
            while (true) {
                responseWindowMA(PsychTest::Choose);

                double stimChange = m_whatChange*m_pt.m_jnd_stimulus_interval;
                setStimulus(m_jnd_current_stimulus += stimChange);

                if (m_adjust != -1) {
                    m_pt.setResponseMA(m_adjust,m_jnd_current_stimulus);
                    // Write Output Data
                    writeMAOutputData(csv, m_pt.m_q_ma);

                    break;
                }
                co_yield nullptr;
            }

            // return to contact position for next trial - ramp down
            elapsed = 0;
            while (elapsed < m_psychparams.stimulus_time) {
                rampStimulus(m_jnd_current_stimulus, m_pt.m_userStimulusContact, m_psychparams.stimulus_time, elapsed);
                responseWindowMA(PsychTest::NA);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            } 
            // render second stimulus - hold stim and record position and force info
            setStimulus(m_pt.m_userStimulusContact);

            // third delay
            elapsed = 0;
            while (elapsed < 1) {
                responseWindowMA(PsychTest::NA);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }

            // Prepare for next trial
            m_pt.setNextTrialMA(); 
            m_flag_presentStims = 0;
            m_flag_reachedMAValue = 0;
            m_adjust = -1;

            
        } // while - under the number of trials      

        // End of Experiment
        double remaining = 10;
        while (remaining > 0) {
            ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            ImGui::Text("Experiment Complete, Notify the Experimentor to Remove You from the Device");
            ImGui::End();
            remaining -= delta_time().as_seconds();
            co_yield nullptr;
        } 
        m_pt.m_testmode = PsychTest::Idle;
        set_window_title("CM " + m_pt.method[m_pt.m_whichExp] + " (Subject " + std::to_string(m_pt.m_subject) + ") (Test " + m_pt.dofChoice[m_pt.m_whichDof] + " dof, " + m_pt.controlChoice[m_pt.m_controller] + " Control)"); // Hardware specific
    }
    
    void PsychGui::writeMAOutputVariables(Csv& csv, Timestamp ts){
        csv.write_row("Mode","Dof","ControlType","Trial","Stimulus1","Stimulus2","Standard","Comparison","Adjust","Dir","Change","FinalVal","NormF1", "ShearF1", "NormP1", "ShearP1","NormF2", "ShearF2", "NormP2", "ShearP2" );
    }

    void PsychGui::writeMAOutputData(Csv& csv, PsychTest::QueryMA trial){
        csv.write_row(trial.testmode, trial.whichDof, trial.controller, trial.trialnum, trial.stimulus1, trial.stimulus2, trial.standard, trial.comparison, trial.adjust, trial.direction, trial.change, trial.finalVal, m_stim1_avgNormF, m_stim1_avgShearF, m_stim1_avgNormP, m_stim1_avgShearP, m_stim2_avgNormF, m_stim2_avgShearF, m_stim2_avgNormP, m_stim2_avgShearP);
    } 

    //////////////////////////////////////////////////////////////////////////////////////
    //           Hardware Specific Functions
    //////////////////////////////////////////////////////////////////////////////////////
     
     void PsychGui::connectToIO(){
        m_hub.createDevice(1, 2, 2, 1, 1, Axis::AxisX, "FT06833.cal", {0,1,2,3,4,5},1); 
        m_hub.createDevice(2, 0, 0, 0, 0, Axis::AxisZ, "FT06833.cal", {0,1,2,3,4,5},1);

        if (m_pt.m_whichDof == PsychTest::Shear) { // test shear direction
            m_cm_test = m_hub.getDevice(1);            
            m_cm_lock = m_hub.getDevice(2);
        }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal direction
            m_cm_test = m_hub.getDevice(2);
            m_cm_lock = m_hub.getDevice(1);
        }

        if (!m_xbox.is_connected())
            LOG(Error) << "No Xbox controller detected!";
     }

     void PsychGui::importUserHardwareParams(){
        std::string user_calibration_file = "C:/Git/TactilePsychophysics/calibs/Psych/JanellePhD.json";
        m_pt.importParams(user_calibration_file);
        m_psychparams = m_pt.getParams();

        std::string cm_norm_cal_file = "C:/Git/TactilePsychophysics/calibs/CM/dof_normal.json";
        std::string cm_tan_cal_file = "C:/Git/TactilePsychophysics/calibs/CM/dof_tangential.json";
        if (m_pt.m_whichDof == PsychTest::Shear) {
            m_cm_test->importParams(cm_tan_cal_file);
            m_cm_lock->importParams(cm_norm_cal_file);
        }else if (m_pt.m_whichDof == PsychTest::Normal){
            m_cm_test->importParams(cm_norm_cal_file);
            m_cm_lock->importParams(cm_tan_cal_file);
        }

        m_paramsTest = m_cm_test->getParams(); 
        m_paramsLock = m_cm_lock->getParams(); 

        m_hub.start();

        if (m_pt.m_whichDof == PsychTest::Shear) { // test shear direction
            m_cm_lock->setPositionRange(m_pt.m_userparams.positionMin_n, m_pt.m_userparams.positionMax_n); // [mm] subject-specific
            m_cm_lock->setForceRange(m_pt.m_userparams.forceMin_n, m_pt.m_userparams.forceMax_n); // [N] subject-specific

            m_cm_test->setPositionRange(m_pt.m_userparams.positionMin_t, m_pt.m_userparams.positionMax_t); // [mm] subject-specific
            m_cm_test->setForceRange(m_pt.m_userparams.forceMin_t, m_pt.m_userparams.forceMax_t); // [N] subject-specific

        }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal direction
            m_cm_lock->setPositionRange(m_pt.m_userparams.positionMin_t, m_pt.m_userparams.positionMax_t); // [mm] subject-specific
            m_cm_lock->setForceRange(m_pt.m_userparams.forceMin_t, m_pt.m_userparams.forceMax_t); // [N] subject-specific
            
            m_cm_test->setPositionRange(m_pt.m_userparams.positionMin_n, m_pt.m_userparams.positionMax_n); // [mm] subject-specific
            m_cm_test->setForceRange(m_pt.m_userparams.forceMin_n, m_pt.m_userparams.forceMax_n); // [N] subject-specific
        }
    }

    void PsychGui::stopExp(){
        m_cm_test->disable();
        m_cm_lock->disable();
        m_hub.stop();
    }

    void PsychGui::calibrate(){
        m_cm_test->zeroPosition();
        m_cm_lock->zeroPosition();

        m_cm_test->zeroForce();
        m_cm_lock->zeroForce();

        start_coroutine(bringToContact());

        start_coroutine(bringToStartPosition());
        m_cm_test->enable();
        m_cm_lock->enable();
    }

    Enumerator PsychGui::bringToStartPosition(){ // above arm, Idle/pre-experiment position
        // disable while switching controllers
        m_cm_test->disable();
        m_cm_lock->disable();
       
        m_cm_lock->setControlMode(CM::Position);
        m_cm_test->setControlMode(CM::Position);
        m_cm_lock->setControlValue(m_cm_lock->scaleRefToCtrlValue(m_cm_lock->getSpoolPosition())); // start from where you are
        m_cm_test->setControlValue(m_cm_test->scaleRefToCtrlValue(m_cm_test->getSpoolPosition())); // start from where you are
        
        // re-enable controllers
        m_cm_test->enable();
        m_cm_lock->enable();

        // move lock dof directly, will lift up if normal, will stay put if tangential
        m_cm_lock->setControlValue(0.0); 
        m_cm_lock->limits_exceeded();

        // move out of stimulus to starting position above the arm
        double elapsed = 0;
        while (elapsed < m_psychparams.stimulus_time) {
            rampStimulus(m_cm_test->getSpoolPosition(), 0.0, m_psychparams.stimulus_time, elapsed);
            elapsed += delta_time().as_seconds();
            co_yield nullptr;
        }
    }


    Enumerator PsychGui::bringToContact(){
        // disable while switching controllers
        m_cm_test->disable();
        m_cm_lock->disable();
       
        m_cm_lock->setControlMode(CM::Position);
        m_cm_test->setControlMode(CM::Position);
        m_cm_lock->setControlValue(m_cm_lock->scaleRefToCtrlValue(m_cm_lock->getSpoolPosition())); // start from where you are
        m_cm_test->setControlValue(m_cm_test->scaleRefToCtrlValue(m_cm_test->getSpoolPosition())); // start from where you are
        
        // re-enable controllers
        m_cm_test->enable();
        m_cm_lock->enable();
        m_cm_test->limits_exceeded();
        m_cm_lock->limits_exceeded();

        if (m_pt.m_whichDof == PsychTest::Shear){
            // keep the shear direction locked in the zero position
            m_cm_test->setControlValue(0);
            m_cm_test->limits_exceeded();

            // move ee to contact point
            double elapsed = 0;
            while (m_cm_lock->getForce() > m_pt.m_userparams.forceCont_n) {
                rampLock(m_cm_lock->getSpoolPosition(), 20.0, m_psychparams.stimulus_time, elapsed); /// !!!!! 20.0 final position is arbitrary, and farther than we'd ever want to go, stopping at desired force
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
        }else {            
            // move ee to contact point
            double elapsed = 0;
            while (m_cm_test->getForce() > m_pt.m_userparams.forceCont_n) {
                rampStimulus(m_cm_test->getSpoolPosition(), 20.0, m_psychparams.stimulus_time, elapsed); /// !!!!! 20.0 final position is arbitrary, and farther than we'd ever want to go, stopping at desired force
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
            
            // keep the shear direction locked in the zero position
            m_cm_lock->setControlValue(0);
            m_cm_lock->limits_exceeded();
        }

        // Record the positions at the contact force
        m_pt.m_userparams.positionCont_n = (m_pt.m_whichDof == PsychTest::Normal) ? m_cm_test->getSpoolPosition() : m_cm_lock->getSpoolPosition();
    }

    Enumerator PsychGui::lockExtraDofs(){ // starting from centered an inch above arm
        m_pt.m_userShearTestNormPos = 0.75*(m_pt.m_userparams.positionMax_n - m_pt.m_userparams.positionMin_n) + m_pt.m_userparams.positionMin_n;

        // disable while switching controller
        m_cm_lock->disable();
        m_cm_lock->setControlMode(CM::Position);
        m_cm_lock->setControlValue(m_cm_lock->scaleRefToCtrlValue(m_cm_lock->getSpoolPosition())); // start from where you are
        m_cm_lock->enable();
        m_cm_lock->limits_exceeded();

        if (m_pt.m_whichDof == PsychTest::Shear){
            // move normal dof to testing location
            double elapsed = 0;
            while (m_psychparams.stimulus_time > elapsed) {
                rampLock(m_cm_lock->getSpoolPosition(), m_pt.m_userShearTestNormPos, m_psychparams.stimulus_time, elapsed); 
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
            setLock(m_pt.m_userShearTestNormPos);
        }else {            
            // move shear dof to testing location
            double elapsed = 0;
            while (m_psychparams.stimulus_time > elapsed) {
                rampStimulus(m_cm_lock->getSpoolPosition(), 0, m_psychparams.stimulus_time, elapsed);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
        }
    }

    Enumerator PsychGui::setControlDof(){
        // Set controller for testing DOF
        m_cm_test->disable();
        if (m_pt.m_whichDof == PsychTest::Shear){ // already at 0, directly commanded
            if (m_pt.m_controller == PsychTest::Position){
                m_cm_test->setControlMode(CM::Position);
                m_pt.m_userStimulusMin = m_pt.m_userparams.positionMin_t;
                m_pt.m_userStimulusMax = m_pt.m_userparams.positionMax_t;
                m_pt.m_userStimulusContact = m_pt.m_userparams.positionCont_t;
            }else if (m_pt.m_controller == PsychTest::Force){
                m_cm_test->setControlMode(CM::Force);
                m_pt.m_userStimulusMin = m_pt.m_userparams.forceMin_t;
                m_pt.m_userStimulusMax = m_pt.m_userparams.forceMax_t;
                m_pt.m_userStimulusContact = m_pt.m_userparams.forceCont_t;
            }
            setStimulus(0);
            m_cm_test->enable();
            m_cm_test->limits_exceeded();
        }else if (m_pt.m_whichDof == PsychTest::Normal){ // currently above the skin by an inch, need to move down
            m_cm_test->setControlMode(CM::Position);
            setStimulus(m_cm_test->getSpoolPosition());
            m_cm_test->enable();
            m_cm_test->limits_exceeded();

            // move normal dof to testing location
            double elapsed = 0;
            while (m_psychparams.stimulus_time > elapsed) {
                rampStimulus(m_cm_test->getSpoolPosition(), m_pt.m_userparams.positionCont_n, m_psychparams.stimulus_time, elapsed); 
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }

            if (m_pt.m_controller == PsychTest::Position){
                setStimulus(m_pt.m_userparams.positionCont_n);
                m_cm_test->limits_exceeded();
                userLimitsExceeded();

                m_pt.m_userStimulusMin = m_pt.m_userparams.positionMin_n;
                m_pt.m_userStimulusMax = m_pt.m_userparams.positionMax_n;
                m_pt.m_userStimulusContact = m_pt.m_userparams.positionCont_n;
            }else if (m_pt.m_controller == PsychTest::Force){
                m_cm_test->disable();
                m_cm_test->setControlMode(CM::Force);
                setStimulus(0);
                m_cm_test->enable();
                m_cm_test->limits_exceeded();
                userLimitsExceeded();

                m_pt.m_userStimulusMin = m_pt.m_userparams.forceMin_n;
                m_pt.m_userStimulusMax = m_pt.m_userparams.forceMax_n;
                m_pt.m_userStimulusContact = m_pt.m_userparams.forceCont_n;
                
            }

        }
    }

    void PsychGui::setStimulus(double N) {
        m_cm_test->setControlValue(m_cm_test->scaleRefToCtrlValue(N));
        m_cm_test->limits_exceeded();
        userLimitsExceeded();
    }

    void PsychGui::setLock(double N) {
        m_cm_lock->setControlValue(m_cm_lock->scaleRefToCtrlValue(N));
        m_cm_lock->limits_exceeded();
        userLimitsExceeded();
    }

    void PsychGui::userLimitsExceeded(){
        if (m_pt.m_whichDof == PsychTest::Shear){ // test shear
            if(m_cm_test->getForce() > m_pt.m_userparams.forceMax_t){
                LOG(Warning) << "Exceeded User Shear Force Limit, " << m_pt.m_userparams.forceMax_t << " N with a value of " << m_cm_test->getForce() << " N.";
                stopExp();
            } else if(m_cm_lock->getForce() > m_pt.m_userparams.forceMax_n){
                LOG(Warning) << "Exceeded User Normal Force Limit, " << m_pt.m_userparams.forceMax_n << " N with a value of " << m_cm_lock->getForce() << " N.";
                stopExp();
            }
            
            if(m_cm_test->getSpoolPosition() > m_pt.m_userparams.positionMax_t){
                LOG(Warning) << "Exceeded User Shear Position Limit, " << m_pt.m_userparams.positionMax_t << " mm with a value of " << m_cm_test->getSpoolPosition() << " mm.";
                stopExp();
            }else if(m_cm_lock->getSpoolPosition() > m_pt.m_userparams.positionMax_n){
                LOG(Warning) << "Exceeded User Normal Position Limit, " << m_pt.m_userparams.positionMax_n << " mm with a value of " << m_cm_lock->getSpoolPosition() << " mm.";
                stopExp();
            }
    
        }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal
            if(m_cm_test->getForce() > m_pt.m_userparams.forceMax_n){
                LOG(Warning) << "Exceeded User Normal Force Limit, " << m_pt.m_userparams.forceMax_n << " N with a value of " << m_cm_test->getForce() << " N.";
                stopExp();
            } else if(m_cm_lock->getForce() > m_pt.m_userparams.forceMax_t){
                LOG(Warning) << "Exceeded User Shear Force Limit, " << m_pt.m_userparams.forceMax_t << " N with a value of " << m_cm_lock->getForce() << " N.";
                stopExp();
            }
            
            if(m_cm_test->getSpoolPosition() > m_pt.m_userparams.positionMax_n){
                LOG(Warning) << "Exceeded User Normal Position Limit, " << m_pt.m_userparams.positionMax_n << " mm with a value of " << m_cm_test->getSpoolPosition() << " mm.";
                stopExp();
            } else if(m_cm_lock->getSpoolPosition() > m_pt.m_userparams.positionMax_t){
                LOG(Warning) << "Exceeded User Shear Position Limit, " << m_pt.m_userparams.positionMax_t << " mm with a value of " << m_cm_lock->getSpoolPosition() << " mm.";
                stopExp();
            }
    
        }

    }

    void PsychGui::collectSensorData(PsychTest::WhichStim whichStim){

        if(whichStim == PsychTest::First){
            if (m_pt.m_whichDof == PsychTest::Shear){ // test shear
                m_stim1_normF.push_back(m_cm_lock->getForce());
                m_stim1_shearF.push_back(m_cm_test->getForce());
                m_stim1_normP.push_back(m_cm_lock->getSpoolPosition());
                m_stim1_shearP.push_back(m_cm_test->getSpoolPosition());
            }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal
                m_stim1_normF.push_back(m_cm_test->getForce());
                m_stim1_shearF.push_back(m_cm_lock->getForce());
                m_stim1_normP.push_back(m_cm_test->getSpoolPosition());
                m_stim1_shearP.push_back(m_cm_lock->getSpoolPosition());
            }
        }else if(whichStim == PsychTest::Second){
            if (m_pt.m_whichDof == PsychTest::Shear){ // test shear
                m_stim2_normF.push_back(m_cm_lock->getForce());
                m_stim2_shearF.push_back(m_cm_test->getForce());
                m_stim2_normP.push_back(m_cm_lock->getSpoolPosition());
                m_stim2_shearP.push_back(m_cm_test->getSpoolPosition());
            }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal
                m_stim2_normF.push_back(m_cm_test->getForce());
                m_stim2_shearF.push_back(m_cm_lock->getForce());
                m_stim2_normP.push_back(m_cm_test->getSpoolPosition());
                m_stim2_shearP.push_back(m_cm_lock->getSpoolPosition());
            }
        }
    }

    void PsychGui::avgSensorData(){
        m_stim1_avgNormF = mean(m_stim1_normF.get_vector());
        m_stim1_avgShearF = mean(m_stim1_shearF.get_vector());
        m_stim1_avgNormP = mean(m_stim1_normP.get_vector());
        m_stim1_avgShearP = mean(m_stim1_shearP.get_vector());

        m_stim2_avgNormF = mean(m_stim2_normF.get_vector());
        m_stim2_avgShearF = mean(m_stim2_shearF.get_vector());
        m_stim2_avgNormP = mean(m_stim2_normP.get_vector());
        m_stim2_avgShearP = mean(m_stim2_shearP.get_vector());
    }

        void PsychGui::plotDebugExpInfo(){
        
        static bool paused = false;
        if(ImGui::Button(paused ? "unpause" : "pause")) paused = !paused;
        ImGui::Separator();
        
        t += ImGui::GetIO().DeltaTime;
        ImGui::SliderFloat("History",&m_history,1,30,"%.1f s");

        if(!paused){
            ref.AddPoint(t, m_pt.m_jnd_stimulus_reference* 1.0f);
            comp.AddPoint(t, m_pt.m_jnd_stimulus_comparison* 1.0f);
            curr.AddPoint(t, m_jnd_current_stimulus* 1.0f);
        }

        ImPlot::SetNextPlotLimitsX(t - m_history, t, !paused ? ImGuiCond_Always : ImGuiCond_Once);
        if (ImPlot::BeginPlot("##StimValues", NULL, NULL, ImVec2(-1,200), 0, 0, 0)) {
            ImPlot::PlotLine("Ref Value", &ref.Data[0].x, &ref.Data[0].y, ref.Data.size(), ref.Offset, 2*sizeof(float));
            ImPlot::PlotLine("Comparison", &comp.Data[0].x, &comp.Data[0].y, comp.Data.size(), comp.Offset, 2*sizeof(float));
            ImPlot::PlotLine("Current Value", &curr.Data[0].x, &curr.Data[0].y, curr.Data.size(), curr.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }

        ImGui::Separator();

        ImGui::PushItemWidth(100);
        ImGui::LabelText("Reference Value", "%f", m_pt.m_jnd_stimulus_reference);
        ImGui::LabelText("Comparison Value", "%f", m_pt.m_jnd_stimulus_comparison);
        if(m_pt.m_whichExp == PsychTest::MCS){
            ImGui::LabelText("Reference Place", "%i", m_pt.m_q_mcs.standard);
            ImGui::LabelText("Comparison Place", "%i", m_pt.m_q_mcs.comparison);
            ImGui::LabelText("Trial Number", "%i", m_pt.m_q_mcs.trialnum);
        }else if(m_pt.m_whichExp == PsychTest::SM){
            ImGui::LabelText("Reference Place", "%i", m_pt.m_q_sm.standard);
            ImGui::LabelText("Comparison Place", "%i", m_pt.m_q_sm.comparison);
            ImGui::LabelText("Last Slope", "%s", m_pt.currdirection[m_pt.m_q_sm.lastSlope]);
            ImGui::LabelText("Number of Reversals", "%i", m_pt.m_q_sm.num_reversal);
            ImGui::LabelText("Trial Number", "%i", m_pt.m_q_sm.trialnum);
        }else if (m_pt.m_whichExp == PsychTest::MA){
            ImGui::LabelText("Current Value", "%f", m_jnd_current_stimulus);
            ImGui::LabelText("Trial Number", "%i", m_pt.m_q_ma.trialnum);
        }
        
        ImGui::PopItemWidth();

        ImGui::Separator();

        if(!paused){
        lockForce.AddPoint(t, m_cm_lock->getForce()* 1.0f);
        lockPosition.AddPoint(t, m_cm_lock->getSpoolPosition()* 1.0f);
        testForce.AddPoint(t, m_cm_test->getForce()* 1.0f);
        testPosition.AddPoint(t, m_cm_test->getSpoolPosition()* 1.0f);
        }

        ImPlot::SetNextPlotLimitsX(t - m_history, t,  !paused ? ImGuiCond_Always : ImGuiCond_Once);
        if (ImPlot::BeginPlot("##Forces", NULL, NULL, ImVec2(-1,200), 0, 0, 0)) {
            ImPlot::PlotLine("Locked CM - Force", &lockForce.Data[0].x, &lockForce.Data[0].y, lockForce.Data.size(), lockForce.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Test CM - Force", &testForce.Data[0].x, &testForce.Data[0].y, testForce.Data.size(), testForce.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }

        ImPlot::SetNextPlotLimitsX(t - m_history, t,  !paused ? ImGuiCond_Always : ImGuiCond_Once);
        if (ImPlot::BeginPlot("##Torques", NULL, NULL, ImVec2(-1,200), 0, 0, 0)) {
            ImPlot::PlotLine("Locked CM - Position", &lockPosition.Data[0].x, &lockPosition.Data[0].y, lockPosition.Data.size(), lockPosition.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Test CM - Position", &testPosition.Data[0].x, &testPosition.Data[0].y, testPosition.Data.size(), testPosition.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }
    }

