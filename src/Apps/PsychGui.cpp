#include "PsychGui.hpp"

using namespace mahi::gui;
using namespace mahi::util;
using namespace mahi::robo;
using namespace xbox;

PsychGui::PsychGui(int subject, PsychTest::WhichExp whichExp, PsychTest::WhichDof whichDOF, PsychTest::ControlType controller) : 
    Application(600,600,"Capstan Module Psychophysical Test (Subject " + std::to_string(subject) + ")" ,false), 
    m_pt(subject, PsychTest::Params(), whichExp, whichDOF, controller), // Hardware specific
    ts(),
    filename_timeseries("C:/Git/TactilePsychophysics/data/" + m_pt.expchoice[m_pt.m_whichExp] + "/_subject_" + std::to_string(m_pt.m_subject) + "_timeseries_" + m_pt.dofChoice[m_pt.m_whichDof] + "_" + m_pt.controlChoice[m_pt.m_controller] + "_" + m_pt.expchoice[m_pt.m_whichExp] + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv"),
    csv_timeseries(filename_timeseries),
    filename("C:/Git/TactilePsychophysics/data/" + m_pt.expchoice[m_pt.m_whichExp] + "/_subject_" + std::to_string(m_pt.m_subject) + "_trialdata_" + m_pt.dofChoice[m_pt.m_whichDof] + "_" + m_pt.controlChoice[m_pt.m_controller] + "_" + m_pt.expchoice[m_pt.m_whichExp] + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv"),
    csv(filename)
    {          
        connectToIO();
        importUserHardwareParams();
        set_frame_limit(90_Hz);

        if(whichExp == PsychTest::MCS){
            writeMCSOutputTimeVariables(csv_timeseries);
            writeMCSOutputStimVariables(csv);
        } else if (whichExp == PsychTest::SM){
            writeSMOutputTimeVariables(csv_timeseries);
            writeSMOutputStimVariables(csv);
        } else if (whichExp == PsychTest::MA){
            writeMAOutputTimeVariables(csv_timeseries);
            writeMAOutputStimVariables(csv);
        }

    }

    PsychGui::~PsychGui() {
        stopExp();
    }

    void PsychGui::update() {

        ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{600,1000}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::BeginDisabled(m_pt.m_testmode == PsychTest::Run);

        // Flags to walk through menus
        static bool flag_confirm_exp_settings = 0;
        static bool flag_start_calibration = 0;
        static bool flag_bring_to_start = 0;
        static bool flag_study_started = 0;
        static bool flag_bring_contact = 0;
        static bool flag_bring_start = 0;
        static bool flag_lock_extra_dofs = 0;
        static bool flag_set_control_dofs = 0;

        // Confirm experiment settings and subject number
        if (!flag_confirm_exp_settings){
            ImGui::BulletText(("You've intiated a " + m_pt.method[m_pt.m_whichExp] + " Experiment with the following settings:").c_str());
            ImGui::BulletText("For Subject %i",  m_pt.m_subject);
            ImGui::BulletText("Testing the %s Direction", m_pt.dofChoice[m_pt.m_whichDof]); // Hardware specific
            ImGui::BulletText("Using %s Control", m_pt.controlChoice[m_pt.m_controller]);   // Hardware specific
            ImGui::Separator();
            ImGui::SetNextItemWidth(100);
            m_pt.m_testmode = PsychTest::SetUp;

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

        // after device calibrated, bring to contact
        if (flag_start_calibration && !flag_bring_contact){
            if (ImGui::Button("Bring to Contact",ImVec2(-1,0))){
                start_coroutine(findContact());
                flag_bring_contact = 1;
            }
        }

        // after bring to contact, back to start position
        if (flag_bring_contact && !flag_bring_start){
            if (ImGui::Button("Bring to Start Position",ImVec2(-1,0))){
                start_coroutine(bringToStartPosition()); 
                flag_bring_start = 1;
            }
        }

        // when at start position, lock extra dofs
        if (flag_bring_start && !flag_lock_extra_dofs){
            if (ImGui::Button("Lock Extra Dofs",ImVec2(-1,0))){
                start_coroutine(lockExtraDofs());
                flag_lock_extra_dofs = 1;
            }
        }

        // after locking extra dofs, set the control dofs
        if (flag_lock_extra_dofs && !flag_set_control_dofs){
            if (ImGui::Button("Set Control Dofs",ImVec2(-1,0))){
                start_coroutine(setControlDof());
                flag_set_control_dofs = 1;
            }
        }
        
        // start experiment or cancel for some reason
        if (flag_set_control_dofs && !flag_study_started){
            if (ImGui::Button("Start Study",ImVec2(-1,0))){
                // Set controllers for testing DOF
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
            if(!m_flag_first_to_start && flag_start_calibration){
                start_coroutine(bringToStartPosition());
                m_flag_first_to_start = 1;
            }
        }

        //plot force and position information
        if (m_debug)
            plotDebugExpInfo();


        if(m_pt.m_whichExp == PsychTest::MCS){
            writeMCSOutputTimeData(csv_timeseries);
        } else if (m_pt.m_whichExp == PsychTest::SM){
            writeSMOutputTimeData(csv_timeseries);
        } else if (m_pt.m_whichExp == PsychTest::MA){
            writeMAOutputTimeData(csv_timeseries);
        }

        ImGui::End();     
    }

    int PsychGui::responseWindow(PsychTest::WhichStim whichStim) {
        float windowHeight = m_debug ? 1500.0 : -1;
        float windowWidth = m_debug ? 600.0 : -1;
        float buttonHeight = m_debug ? 50.0 : -1;

        m_whichStim = whichStim;

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

        ImGui::End();
        return ans;
    }


    void PsychGui::rampStimulus(double start, double end, double ramptime, double elapsed){
        double stimVal = Tween::Linear(start, end, (float)(elapsed / ramptime));
        PsychGui::setTest(stimVal);
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
        std::cout << "set to run mode" << std::endl;
        m_flag_first_to_start = 0;


        // Run trials
        for (int w = 0; w < m_psychparams.n_mcs_windows; ++w) {
            set_window_title("CM " + m_pt.method[m_pt.m_whichExp] + " (Subject " + std::to_string(m_pt.m_subject) + ") (Test " + m_pt.dofChoice[m_pt.m_whichDof] + " dof, " + m_pt.controlChoice[m_pt.m_controller] + " Control) (S" + std::to_string(w+1) + ")"); // Hardware specific

            // beginning delay
            std::cout << "1 second delay" << std::endl;
            double elapsed = 0;
            while (elapsed < 1) {
                responseWindow(PsychTest::NA);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
            int i = 1;
            for (auto& trial : m_pt.m_stim_trials_mcs[w]) {
                std::cout << "start trial " << i << std::endl;
                i++;
                ////////////////////////////////////// run trial
                m_pt.m_q_mcs = trial;
                m_pt.m_jnd_stimulus_comparison = m_pt.m_q_mcs.comparison == 1 ? m_pt.m_q_mcs.stimulus1 : m_pt.m_q_mcs.stimulus2;
                // render first stimulus - ramp up
                std::cout << "ramp up to first stim" << std::endl;
                elapsed = 0;
                while (elapsed < m_psychparams.ramp_time) {
                    rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_mcs.stimulus1, m_psychparams.ramp_time, elapsed);
                    responseWindow(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    if(int(elapsed*10) == 0){
                        double pos = m_cm_test->getSpoolPosition();
                        double force = m_cm_test->getForce(1);
                        std::cout << "      elapsed " << elapsed << " pos " << pos << " force " << force << std::endl;
                    }
                    co_yield nullptr;
                } 

                // render first stimulus - hold stim and record position and force info
                std::cout << "hold stim" << std::endl;
                setTest(m_pt.m_q_mcs.stimulus1);
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    responseWindow(PsychTest::First);
                    // collect values while the stimulus is held
                    collectSensorData(PsychTest::First, m_pt.m_q_mcs.standard);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }

                // render first stimulus - ramp down
                std::cout << "ramp down first stim to contact" << std::endl;
                elapsed = 0;
                while (elapsed < m_psychparams.ramp_time) {
                    rampStimulus(m_pt.m_q_mcs.stimulus1, m_pt.m_userStimulusContact, m_psychparams.ramp_time, elapsed);
                    responseWindow(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                setTest(m_pt.m_userStimulusContact);
                // first delay
                std::cout << "0.5 sec delay" << std::endl;
                elapsed = 0;
                while (elapsed < 0.5) {
                    responseWindow(PsychTest::NA);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // render second stimulus - ramp up
                std::cout << "ramp up to second stim" << std::endl;
                elapsed = 0;
                while (elapsed < m_psychparams.ramp_time) {
                    rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_mcs.stimulus2, m_psychparams.ramp_time, elapsed);
                    responseWindow(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                // render second stimulus - hold stim and record position and force info
                std::cout << "hold second stim" << std::endl;
                setTest(m_pt.m_q_mcs.stimulus2);
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    responseWindow(PsychTest::Second);
                    //collect values while the stimulus is held
                    collectSensorData(PsychTest::Second, m_pt.m_q_mcs.standard);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }

                // render second stimulus - ramp down
                std::cout << "ramp down second stim to contact" << std::endl;
                elapsed = 0;
                while (elapsed < m_psychparams.ramp_time) {
                    rampStimulus(m_pt.m_q_mcs.stimulus2, m_pt.m_userStimulusContact, m_psychparams.ramp_time, elapsed);
                    responseWindow(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // End at contact point
                setTest(m_pt.m_userStimulusContact);

                // collect response
                std::cout << "choose response" << std::endl;
                while (true) {
                    int answer = responseWindow(PsychTest::Choose);
                    if (answer != -1) {
                        int greater = m_pt.m_q_mcs.comparison == answer ? 1 : 0;
                        m_pt.setResponseMCS(answer,greater);

                        // Write Output Data
                        writeMCSOutputStimData(csv, m_pt.m_q_mcs);

                        break;
                    }
                    co_yield nullptr;
                }

                // second delay
                std::cout << "delay 1 second" << std::endl;
                elapsed = 0;
                while (elapsed < 1) {
                    responseWindow(PsychTest::NA);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
            }  // for trial within each window    

            std::cout << "delay 10 seconds between windows" << std::endl;
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
        } // for window
        std::cout << "set idle" << std::endl;
        m_pt.m_testmode = PsychTest::Idle;
        set_window_title("CM " + m_pt.method[m_pt.m_whichExp] + " (Subject " + std::to_string(m_pt.m_subject) + ") (Test " + m_pt.dofChoice[m_pt.m_whichDof] + " dof, " + m_pt.controlChoice[m_pt.m_controller] + " Control)"); // Hardware specific
    }
    
    void PsychGui::writeMCSOutputStimVariables(Csv& csv){
        csv.write_row("Time","Mode","Dof","ControlType","Trial","GenNum","Window","Level","Stimulus1","Stimulus2","Standard","Comparison","Correct","Answer","Greater","NormFRef", "ShearFRef", "NormPRef", "ShearPRef","NormFComp", "ShearFComp", "NormPComp", "ShearPComp" );
    }
    
    void PsychGui::writeMCSOutputStimData(Csv& csv, PsychTest::QueryMCS trial){
        avgSensorData();
        csv.write_row(time().as_seconds(), trial.testmode, trial.whichDof, trial.controller, trial.trialnum, trial.generated_num, trial.window, trial.level, trial.stimulus1, trial.stimulus2, trial.standard, trial.comparison, trial.correct, trial.answer, trial.greater, m_ref_avgNormF, m_ref_avgShearF, m_ref_avgNormP, m_ref_avgShearP, m_comp_avgNormF, m_comp_avgShearF, m_comp_avgNormP, m_comp_avgShearP);
    }

    void PsychGui::writeMCSOutputTimeVariables(Csv& csv){
        csv.write_row("Time","Mode","Dof","ControlType","Trial","whichStim","NormF", "ShearF", "NormP", "ShearP", "eeRadius", "CombinedE","ContactRadius","PlanarA","SphericalA","MeanStress","MeanStrain","ElasticStrainEnergy_N","YoungModulusNS","PoissonNS","CouplingBetaNS","ShearModulusNS","ComplianceN_NS","ComplianceT_NS");
    }

    void PsychGui::writeMCSOutputTimeData(Csv& csv){
        getFNUpdate();
        m_q_hz_ns = m_hz.makeQuery_TanNoSlip(m_R, m_NormF, m_ShearF, m_NormP, m_ShearP);
        csv.write_row(time().as_seconds(), m_pt.m_testmode, m_pt.m_whichDof, m_pt.m_controller, m_pt.m_q_mcs.trialnum, m_whichStim, m_NormF, m_ShearF, m_NormP, m_ShearP, m_q_hz_ns.R, m_q_hz_ns.combinedE, m_q_hz_ns.a, m_q_hz_ns.planarA, m_q_hz_ns.sphericalA, m_q_hz_ns.meanStress, m_q_hz_ns.meanStrain, m_q_hz_ns.Wn, m_q_hz_ns.E, m_q_hz_ns.v, m_q_hz_ns.couplingP, m_q_hz_ns.G, m_q_hz_ns.complianceN, m_q_hz_ns.complianceT);
    }
 
    //////////////////////////////////////////////////////////////////////////////////////
    //           Staircase Method Specific Functions
    //////////////////////////////////////////////////////////////////////////////////////          

    Enumerator PsychGui::runSMExperiment() {

            set_window_title("CM " + m_pt.method[m_pt.m_whichExp] + " Num " + std::to_string(m_pt.m_q_sm.num_staircase) + " (Subject " + std::to_string(m_pt.m_subject) + ") (Test " + m_pt.dofChoice[m_pt.m_whichDof] + " dof, " + m_pt.controlChoice[m_pt.m_controller] + " Control)"); // Hardware specific
            
            // beginning delay
            double elapsed = 0;
            while (elapsed < 1) {
                responseWindow(PsychTest::NA);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }

            while(m_pt.m_q_sm.num_staircase < m_psychparams.n_sm_staircases){
                m_pt.m_testmode = PsychTest::Run;
                m_flag_first_to_start = 0;

                std::cout << "trial num: " << m_pt.m_q_sm.trialnum << ", stair num: " << m_pt.m_q_sm.num_staircase << std::endl;
                if ( m_pt.m_q_sm.trialnum == 1 && m_pt.m_q_sm.num_staircase > 0){ // bring to contact if new stair, but not the first staircase (is from setcontroldofs)
                    std::cout << "Set control dof" << std::endl;
                    // Set controller/position for testing and lock DOFs
                    if (m_pt.m_whichDof == PsychTest::Shear){ // already at 0, directly commanded
                        // move normal dof to testing location
                        std::cout << "     move normal direction to 85% of the range for testing  (t)" << std::endl;
                        double elapsed = 0;
                        double travelT = 4*m_psychparams.travel_time;
                        while ( travelT > elapsed) {
                            rampLock(m_cm_lock->getSpoolPosition(), m_pt.m_userShearTestNormPos, travelT, elapsed); 
                            elapsed += delta_time().as_seconds();
                            co_yield nullptr;
                        }
                        setLock(m_pt.m_userShearTestNormPos);
                        
                        if (m_pt.m_controller == PsychTest::Force){
                            std::cout << "     set to Force control" << std::endl;
                            setForceControl(1);
                        }
                        std::cout << "     set to zero" << std::endl;
                        setTest(0);
                    }else if (m_pt.m_whichDof == PsychTest::Normal){ // currently above the skin by an inch, need to move down
                        // move normal dof to testing location
                        std::cout << "     move norm to contact point" << std::endl;
                        double elapsed = 0;
                        while (m_psychparams.travel_time > elapsed) {
                            rampStimulus(m_cm_test->getSpoolPosition(), m_pt.m_userparams.positionCont_n, m_psychparams.travel_time, elapsed); 
                            elapsed += delta_time().as_seconds();
                            co_yield nullptr;
                        }

                        std::cout << "set controller to m_pt.m_userparams.positionCont_n" << m_pt.m_userparams.positionCont_n << std::endl;
                        std::cout << "m_cm_test->getSpoolPosition()" << m_cm_test->getSpoolPosition() << std::endl;

                        if (m_pt.m_controller == PsychTest::Force){
                            std::cout << "     set to force control at contact point" << std::endl;
                            setForceControl(1);
                            setTest(m_pt.m_userparams.forceCont_n);
                        }

                    }
                }

                // run trials
                while (m_pt.m_q_sm.num_reversal < m_psychparams.n_sm_reversals){
                    // render first stimulus - ramp up
                    elapsed = 0;
                    while (elapsed < m_psychparams.ramp_time) {
                        rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_sm.stimulus1, m_psychparams.ramp_time, elapsed);
                        responseWindow(PsychTest::First);
                        elapsed += delta_time().as_seconds();
                        co_yield nullptr;
                    } 
                    
                    // render first stimulus - hold stim and record position and force info
                    setTest(m_pt.m_q_sm.stimulus1);
                    elapsed = 0;
                    while (elapsed < m_psychparams.stimulus_time) {
                        responseWindow(PsychTest::First);
                        // collect values while the stimulus is held
                        collectSensorData(PsychTest::First, m_pt.m_q_sm.standard);
                        elapsed += delta_time().as_seconds();
                        co_yield nullptr;
                    }

                    // render first stimulus - ramp down
                    elapsed = 0;
                    while (elapsed < m_psychparams.ramp_time) {
                        rampStimulus(m_pt.m_q_sm.stimulus1, m_pt.m_userStimulusContact, m_psychparams.ramp_time, elapsed);
                        responseWindow(PsychTest::First);
                        elapsed += delta_time().as_seconds();
                        co_yield nullptr;
                    } 
                    setTest(m_pt.m_userStimulusContact);
                    // first delay
                    elapsed = 0;
                    while (elapsed < 0.5) {
                        responseWindow(PsychTest::NA);
                        elapsed += delta_time().as_seconds();
                        co_yield nullptr;
                    }
                    // render second stimulus - ramp up
                    elapsed = 0;
                    while (elapsed < m_psychparams.ramp_time) {
                        rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_sm.stimulus2, m_psychparams.ramp_time, elapsed);
                        responseWindow(PsychTest::Second);
                        elapsed += delta_time().as_seconds();
                        co_yield nullptr;
                    } 
                    // render second stimulus - hold stim and record position and force info
                    setTest(m_pt.m_q_sm.stimulus2);
                    elapsed = 0;
                    while (elapsed < m_psychparams.stimulus_time) {
                        responseWindow(PsychTest::Second);
                        //collect values while the stimulus is held
                        collectSensorData(PsychTest::Second, m_pt.m_q_sm.standard);
                        elapsed += delta_time().as_seconds();
                        co_yield nullptr;
                    }

                    // render second stimulus - ramp down
                    elapsed = 0;
                    while (elapsed < m_psychparams.ramp_time) {
                        rampStimulus(m_pt.m_q_sm.stimulus2, m_pt.m_userStimulusContact, m_psychparams.ramp_time, elapsed);
                        responseWindow(PsychTest::Second);
                        elapsed += delta_time().as_seconds();
                        co_yield nullptr;
                    }
                    // End at contact point
                    setTest(m_pt.m_userStimulusContact);

                    // collect response
                    while (true) {
                        int answer = responseWindow(PsychTest::Choose);
                        if (answer != -1) {
                            int greater = m_pt.m_q_sm.comparison == answer ? 1 : 0;
                            m_pt.setResponseSM(answer,greater);

                            // Write Output Data
                            writeSMOutputStimData(csv, m_pt.m_q_sm);

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
                m_pt.m_testmode = PsychTest::Idle;
                double remaining = 10;
                while (remaining > 0) {
                    ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
                    ImGui::Text("Staircase Complete");
                    ImGui::Text("Break (%.3f)", remaining);
                    ImGui::End();
                    remaining -= delta_time().as_seconds();
                    co_yield nullptr;
                } 
                set_window_title("CM " + m_pt.method[m_pt.m_whichExp] + " Num " + std::to_string(m_pt.m_q_sm.num_staircase) + " (Subject " + std::to_string(m_pt.m_subject) + ") (Test " + m_pt.dofChoice[m_pt.m_whichDof] + " dof, " + m_pt.controlChoice[m_pt.m_controller] + " Control)"); // Hardware specific
                m_pt.setNewStaircase(); // staircase num is incremented inside here while preparing settings for next trial
                m_pt.setNextTrialSM();
        } // while - number of staircases

        double remaining = 10;
        while (remaining > 0) {
            ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            ImGui::Text("Experiment Complete, Notify the Experimentor to Remove You from the Device");
            ImGui::End();
            remaining -= delta_time().as_seconds();
            co_yield nullptr;
        } 
        
    } // run experiment
    
    void PsychGui::writeSMOutputStimVariables(Csv& csv){
        csv.write_row("Time", "Mode","Dof","ControlType","StairNum","Trial","Dir","lastSlope","ReversalNum","isReversal","Stimulus1","Stimulus2","Standard","Comparison","Correct","Answer","Greater","NormFRef", "ShearFRef", "NormPRef", "ShearPRef","NormFComp", "ShearFComp", "NormPComp", "ShearPComp" );
    }

    void PsychGui::writeSMOutputStimData(Csv& csv, PsychTest::QuerySM trial){
        avgSensorData();
        csv.write_row(time().as_seconds(), trial.testmode, trial.whichDof, trial.controller, trial.num_staircase, trial.trialnum, trial.direction, trial.lastSlope, trial.num_reversal, trial.isReversal, trial.stimulus1, trial.stimulus2, trial.standard, trial.comparison, trial.correct, trial.answer, trial.greater, m_ref_avgNormF, m_ref_avgShearF, m_ref_avgNormP, m_ref_avgShearP, m_comp_avgNormF, m_comp_avgShearF, m_comp_avgNormP, m_comp_avgShearP);
    } 

    void PsychGui::writeSMOutputTimeVariables(Csv& csv){
        csv.write_row("Time", "Mode","Dof","ControlType","StairNum","Trial","whichStim","NormF", "ShearF", "NormP", "ShearP", "eeRadius", "CombinedE","ContactRadius","PlanarA","SphericalA","MeanStress","MeanStrain","ElasticStrainEnergy_N","YoungModulusNS","PoissonNS","CouplingBetaNS","ShearModulusNS","ComplianceN_NS","ComplianceT_NS");
    }

    void PsychGui::writeSMOutputTimeData(Csv& csv){
        getFNUpdate();
        m_q_hz_ns = m_hz.makeQuery_TanNoSlip(m_R, m_NormF, m_ShearF, m_NormP, m_ShearP);
        csv.write_row(time().as_seconds(), m_pt.m_testmode, m_pt.m_whichDof, m_pt.m_controller, m_pt.m_q_sm.num_staircase, m_pt.m_q_sm.trialnum, m_whichStim, m_NormF, m_ShearF, m_NormP, m_ShearP, m_q_hz_ns.R, m_q_hz_ns.combinedE, m_q_hz_ns.a, m_q_hz_ns.planarA, m_q_hz_ns.sphericalA, m_q_hz_ns.meanStress, m_q_hz_ns.meanStrain, m_q_hz_ns.Wn, m_q_hz_ns.E, m_q_hz_ns.v, m_q_hz_ns.couplingP, m_q_hz_ns.G, m_q_hz_ns.complianceN, m_q_hz_ns.complianceT);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    //           Method of Adjustments Functions
    //////////////////////////////////////////////////////////////////////////////////////
     
     void PsychGui::responseWindowMA(PsychTest::WhichStim whichStim) {
        float windowHeight = m_debug ? 1500.0 : -1;
        float windowWidth = m_debug ? 600.0 : -1;
        float buttonHeight = m_debug ? 50.0 : 500.0;

        m_whichStim = whichStim;

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

            if (ImGui::Button("Submit Value", ImVec2(290,buttonHeight)) || m_xbox.is_button_pressed(XboxController::A)){
                m_adjust = 1; // yes, and done
                m_flag_reachedMAValue = 1;
                flag_chooseAdjust = 0;
            }                  
        }

        ImGui::End();
    }
    
    Enumerator PsychGui::runMAExperiment(){

 
        m_pt.m_testmode = PsychTest::Run;
        m_flag_first_to_start = 0;

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
                while (elapsed < m_psychparams.ramp_time) {
                    rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_ma.stimulus1, m_psychparams.ramp_time, elapsed);
                    responseWindowMA(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                // render first stimulus - hold stim and record position and force info
                setTest(m_pt.m_q_ma.stimulus1);
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    responseWindowMA(PsychTest::First);
                    // collect values while the stimulus is held
                    collectSensorData(PsychTest::First, m_pt.m_q_ma.standard);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }

                // render first stimulus - ramp down
                elapsed = 0;
                while (elapsed < m_psychparams.ramp_time) {
                    rampStimulus(m_pt.m_q_ma.stimulus1, m_pt.m_userStimulusContact, m_psychparams.ramp_time, elapsed);
                    responseWindowMA(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                setTest(m_pt.m_userStimulusContact);
                // first delay
                elapsed = 0;
                while (elapsed < 0.5) {
                    responseWindowMA(PsychTest::NA);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // render second stimulus - ramp up
                elapsed = 0;
                while (elapsed < m_psychparams.ramp_time) {
                    rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_ma.stimulus2, m_psychparams.ramp_time, elapsed);
                    responseWindowMA(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                // render second stimulus - hold stim and record position and force info
                setTest(m_pt.m_q_ma.stimulus2);
                elapsed = 0;
                while (elapsed < m_psychparams.stimulus_time) {
                    responseWindowMA(PsychTest::Second);
                    //collect values while the stimulus is held
                    collectSensorData(PsychTest::Second, m_pt.m_q_ma.standard);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }

                // render second stimulus - ramp down
                elapsed = 0;
                while (elapsed < m_psychparams.ramp_time) {
                    rampStimulus(m_pt.m_q_ma.stimulus2, m_pt.m_userStimulusContact, m_psychparams.ramp_time, elapsed);
                    responseWindowMA(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // End at contact point
                setTest(m_pt.m_userStimulusContact); 

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
            while (elapsed < m_psychparams.ramp_time) {
                rampStimulus(m_pt.m_userStimulusContact, m_pt.m_q_ma.stimulus2, m_psychparams.ramp_time, elapsed);
                responseWindowMA(PsychTest::Choose);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            } 

            // collect response and adjust value
            while (true) {
                responseWindowMA(PsychTest::Choose);
                collectSensorData(PsychTest::Adjust, m_pt.m_q_ma.standard);

                double stimChange = m_whatChange*m_pt.m_jnd_stimulus_interval;
                double stimSet = m_jnd_current_stimulus += stimChange;
                if((stimSet<m_pt.m_userStimulusMin)||(stimSet>m_pt.m_userStimulusMax)){
                    LOG(Warning) << "stimulus value " << stimSet <<" clamped by thresholds " << m_pt.m_userStimulusMin << " and " << m_pt.m_userStimulusMax;
                }
                double stimSet_clamp = clamp(stimSet, m_pt.m_userStimulusMin, m_pt.m_userStimulusMax);
                setTest(stimSet_clamp);

                if (m_adjust != -1) {
                    m_pt.setResponseMA(m_adjust,m_jnd_current_stimulus);
                    // Write Output Data
                    writeMAOutputStimData(csv, m_pt.m_q_ma);

                    break;
                }
                co_yield nullptr;
            }

            // return to contact position for next trial - ramp down
            elapsed = 0;
            while (elapsed < m_psychparams.ramp_time) {
                rampStimulus(m_jnd_current_stimulus, m_pt.m_userStimulusContact, m_psychparams.ramp_time, elapsed);
                responseWindowMA(PsychTest::NA);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            } 
            setTest(m_pt.m_userStimulusContact);

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
    
    void PsychGui::writeMAOutputStimVariables(Csv& csv){
        csv.write_row("Time", "Mode","Dof","ControlType","Trial","Stimulus1","Stimulus2","Standard","Comparison","Adjust","Dir","Change","FinalVal","NormFRef", "ShearFRef", "NormPRef", "ShearPRef","NormFComp", "ShearFComp", "NormPComp", "ShearPComp","NormFAdjust", "ShearFAdjust", "NormPAdjust", "ShearPAdjust"  );
    }

    void PsychGui::writeMAOutputStimData(Csv& csv, PsychTest::QueryMA trial){
        avgSensorData();
        csv.write_row(time().as_seconds(), trial.testmode, trial.whichDof, trial.controller, trial.trialnum, trial.stimulus1, trial.stimulus2, trial.standard, trial.comparison, trial.adjust, trial.direction, trial.change, trial.finalVal, m_ref_avgNormF, m_ref_avgShearF, m_ref_avgNormP, m_ref_avgShearP, m_comp_avgNormF, m_comp_avgShearF, m_comp_avgNormP, m_comp_avgShearP, m_adjust_avgNormF, m_adjust_avgShearF, m_adjust_avgNormP, m_adjust_avgShearP);
    } 

    void PsychGui::writeMAOutputTimeVariables(Csv& csv){
        csv.write_row("Time", "Mode","Dof","ControlType","Trial","whichStim","NormF", "ShearF", "NormP", "ShearP", "eeRadius", "CombinedE","ContactRadius","PlanarA","SphericalA","MeanStress","MeanStrain","ElasticStrainEnergy_N","YoungModulusNS","PoissonNS","CouplingBetaNS","ShearModulusNS","ComplianceN_NS","ComplianceT_NS");
    }

    void PsychGui::writeMAOutputTimeData(Csv& csv){
        getFNUpdate();
        m_q_hz_ns = m_hz.makeQuery_TanNoSlip(m_R, m_NormF, m_ShearF, m_NormP, m_ShearP);
        csv.write_row(time().as_seconds(), m_pt.m_testmode, m_pt.m_whichDof, m_pt.m_controller, m_pt.m_q_ma.trialnum, m_whichStim, m_NormF, m_ShearF, m_NormP, m_ShearP, m_q_hz_ns.R, m_q_hz_ns.combinedE, m_q_hz_ns.a, m_q_hz_ns.planarA, m_q_hz_ns.sphericalA, m_q_hz_ns.meanStress, m_q_hz_ns.meanStrain, m_q_hz_ns.Wn, m_q_hz_ns.E, m_q_hz_ns.v, m_q_hz_ns.couplingP, m_q_hz_ns.G, m_q_hz_ns.complianceN, m_q_hz_ns.complianceT);
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
            m_cm_lock->setPositionRange(m_paramsLock.positionMin, m_pt.m_maxRangePercent*m_pt.m_userparams.positionMax_n); // [mm] subject-specific
            m_cm_lock->setForceRange(m_paramsLock.forceMin, m_pt.m_maxRangePercent*m_pt.m_userparams.forceMax_n); // [N] subject-specific

            m_cm_test->setPositionRange(m_paramsTest.positionMin, m_pt.m_maxRangePercent*m_pt.m_userparams.positionMax_t); // [mm] subject-specific
            m_cm_test->setForceRange(m_paramsTest.forceMin, m_pt.m_maxRangePercent*m_pt.m_userparams.forceMax_t); // [N] subject-specific

        }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal direction
            m_cm_lock->setPositionRange(m_paramsLock.positionMin, m_pt.m_maxRangePercent*m_pt.m_userparams.positionMax_t); // [mm] subject-specific
            m_cm_lock->setForceRange(m_paramsLock.forceMin, m_pt.m_maxRangePercent*m_pt.m_userparams.forceMax_t); // [N] subject-specific
            
            m_cm_test->setPositionRange(m_paramsTest.positionMin, m_pt.m_maxRangePercent*m_pt.m_userparams.positionMax_n); // [mm] subject-specific
            m_cm_test->setForceRange(m_paramsTest.forceMin, m_pt.m_maxRangePercent*m_pt.m_userparams.forceMax_n); // [N] subject-specific
        }
    }

    void PsychGui::stopExp(){
        m_cm_test->disable();
        m_cm_lock->disable();
        m_hub.stop();
    }

    void PsychGui::calibrate(){
        std::cout << "Calibrate" << std::endl;
        m_cm_test->zeroPosition();
        m_cm_lock->zeroPosition();

        m_cm_test->zeroForce();
        m_cm_lock->zeroForce();
        std::cout << "     READY" << std::endl;
    }

    Enumerator PsychGui::findContact(){ // Put both in position control for now
        std::cout << "Bring to contact" << std::endl;
        // disable while switching controllers
        setPositionControl(0);
        setPositionControl(1);
        std::cout << "     set to position control in current location" << std::endl;

        if (m_pt.m_whichDof == PsychTest::Shear){
            // keep the shear direction locked in the zero position
            std::cout << "     keep shear direction locked (t)" << std::endl;
            setTest(0);

            // move ee to contact point
            std::cout << "     ramp normal position until contact force is reached (t)" << std::endl;
            double step = 0.10;
            double setPoint = m_cm_lock->getSpoolPosition();
            int num_above_force = 0;
            while (num_above_force < 20) {
                if (m_cm_lock->getForce(1) > m_pt.m_userparams.forceCont_n) num_above_force++;
                else num_above_force = 0;
                setLock(setPoint);
                setPoint += step;
                co_yield nullptr;
            }

            std::cout << "     force control to contact force" << std::endl;
            setForceControl(0);
            setLock(m_pt.m_userparams.forceCont_n);
        }else {   
            // keep the shear direction locked in the zero position
            std::cout << "     keep shear direction locked (n)" << std::endl;
            setLock(0);

            // move ee to contact point
            std::cout << "     ramp normal force until contact force is reached (n)" << std::endl;
            double step = 0.10;
            double setPoint = m_cm_test->getSpoolPosition();
            int num_above_force = 0;
            while (num_above_force < 20) {
                if (m_cm_test->getForce(1) > m_pt.m_userparams.forceCont_n) num_above_force++;
                else num_above_force = 0;
                setTest(setPoint);
                setPoint += step;
                co_yield nullptr;
            }

            std::cout << "     force control to contact force" << std::endl;
            setForceControl(1);
            setTest( m_pt.m_userparams.forceCont_n);
        }

        // Allow time to settle at contact force
        std::cout << "     allow time to settle at desired force" << std::endl;
        double elapsed = 0;
        while (elapsed < 1) {
            elapsed += delta_time().as_seconds();
            co_yield nullptr;
        }

        // Rezero at contact force so contact point is zero. Set back to position control
        m_cm_test->zeroPosition();
        setPositionControl(1);
        m_cm_lock->zeroPosition();
        setPositionControl(0);
        std::cout << "     zero and set back to position control" << std::endl;
        if( m_pt.m_controller == PsychTest::Position){std::cout << "     READY" << std::endl;}
           
    }

    Enumerator PsychGui::bringToStartPosition(){ // above arm, Idle/pre-experiment position, assume already in position control
        // disable while switching controllers
        std::cout << "Bring to start" << std::endl;

        if (m_pt.m_whichDof == PsychTest::Shear){
            // move shear to center
            double elapsed = 0;
            while (elapsed < m_psychparams.travel_time) {
                std::cout << "m_cm_test->getSpoolPosition()" << m_cm_test->getSpoolPosition() << " m_pt.m_userparams.positionStart_t " << m_pt.m_userparams.positionStart_t << std::endl;
                rampStimulus(m_cm_test->getSpoolPosition(), m_pt.m_userparams.positionStart_t, m_psychparams.travel_time, elapsed);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
            std::cout << "     set shear (test) to zero position" << std::endl;
            //setTest(m_pt.m_userparams.positionStart_t);

            // move out of stimulus to starting position above the arm
            elapsed = 0;
            while (elapsed < m_psychparams.travel_time) {
                std::cout << "m_cm_lock->getSpoolPosition()" << m_cm_lock->getSpoolPosition() << " m_pt.m_userparams.positionStart_n " << m_pt.m_userparams.positionStart_n << std::endl;
                rampLock(m_cm_lock->getSpoolPosition(), m_pt.m_userparams.positionStart_n, m_psychparams.travel_time, elapsed);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
            std::cout << "     ramp normal (lock) to start position" << std::endl;
            setLock(m_pt.m_userparams.positionStart_n);
        
        }else if (m_pt.m_whichDof == PsychTest::Normal){
            // move shear to center
            double elapsed = 0;
            while (elapsed < m_psychparams.travel_time) {
                rampStimulus(m_cm_lock->getSpoolPosition(), m_pt.m_userparams.positionStart_t, m_psychparams.travel_time, elapsed);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
            std::cout << "     set shear (lock) to zero position" << std::endl;
            setLock(m_pt.m_userparams.positionStart_t);

            // move out of stimulus to starting position above the arm
            elapsed = 0;
            while (elapsed < m_psychparams.travel_time) {
                rampStimulus(m_cm_test->getSpoolPosition(), m_pt.m_userparams.positionStart_n, m_psychparams.travel_time, elapsed);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
            std::cout << "     ramp normal (test) to start position" << std::endl;
            setTest(m_pt.m_userparams.positionStart_n);
        }
        std::cout << "     READY" << std::endl;
    }

    Enumerator PsychGui::lockExtraDofs(){ // starting from centered an inch above arm, assume already in position control
        std::cout << "Lock extra dofs" << std::endl;
        m_pt.m_userShearTestNormPos = 0.8*(m_pt.m_userparams.positionMax_n - m_pt.m_userparams.positionMin_n) + m_pt.m_userparams.positionMin_n;

        std::cout << "     set lock in position control in the current location" << std::endl;
        if (m_pt.m_whichDof == PsychTest::Shear){
            // move normal dof to testing location
            std::cout << "     move normal direction to 80% of the range for testing  (t)" << std::endl;
            double elapsed = 0;
            double travelT = 4*m_psychparams.travel_time;
            while ( travelT > elapsed) {
                rampLock(m_cm_lock->getSpoolPosition(), m_pt.m_userShearTestNormPos, travelT, elapsed); 
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
            setLock(m_pt.m_userShearTestNormPos);
        }else if (m_pt.m_whichDof == PsychTest::Normal) {            
            // move shear dof to testing location
            std::cout << "     move the shear direction to zero for testing  (n)" << std::endl;
            double elapsed = 0;
            while (m_psychparams.travel_time > elapsed) {
                rampLock(m_cm_lock->getSpoolPosition(), 0, m_psychparams.travel_time, elapsed);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
        }
        std::cout << "     READY" << std::endl;
    }

    Enumerator PsychGui::setControlDof(){ // assume already in position control
        std::cout << "Set control dof" << std::endl;
        // Set controller for testing DOF
        if (m_pt.m_whichDof == PsychTest::Shear){ // already at 0, directly commanded
            if (m_pt.m_controller == PsychTest::Force){
                std::cout << "     set to Force control" << std::endl;
                setForceControl(1);
            }
            std::cout << "     set to zero" << std::endl;
            setTest(0);
        }else if (m_pt.m_whichDof == PsychTest::Normal){ // currently above the skin by an inch, need to move down
            // move normal dof to testing location
            std::cout << "     move norm to contact point" << std::endl;
            double elapsed = 0;
            while (m_psychparams.travel_time > elapsed) {
                rampStimulus(m_cm_test->getSpoolPosition(), m_pt.m_userparams.positionCont_n, m_psychparams.travel_time, elapsed); 
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }

            if (m_pt.m_controller == PsychTest::Force){
                std::cout << "     set to force control at contact point" << std::endl;
                setForceControl(1);
                setTest(m_pt.m_userparams.forceCont_n);
            }
        }
        std::cout << "     READY" << std::endl;
    }

    void PsychGui::setPositionControl(bool isTest){
        if(isTest){
            double currentPos = m_cm_test->getSpoolPosition();
            std::cout << "currentPos " << currentPos << std::endl;
            m_cm_test->disable();
            m_cm_test->setControlMode(CM::Position);
            setTest(currentPos);
            m_cm_test->enable();
            m_cm_test->limits_exceeded();
            userLimitsExceeded();

            if (m_pt.m_whichDof == PsychTest::Shear){ // test shear
                m_pt.m_userStimulusMin = m_pt.m_userparams.positionMin_t;
                m_pt.m_userStimulusMax = m_pt.m_maxRangePercent*m_pt.m_userparams.positionMax_t;
                m_pt.m_userStimulusContact = m_pt.m_userparams.positionCont_t;
            }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal
                m_pt.m_userStimulusMin = m_pt.m_userparams.positionMin_n;
                m_pt.m_userStimulusMax = m_pt.m_maxRangePercent*m_pt.m_userparams.positionMax_n;
                m_pt.m_userStimulusContact = m_pt.m_userparams.positionCont_n;
            }
        }else{
            double currentPos = m_cm_lock->getSpoolPosition();
            m_cm_lock->disable();
            m_cm_lock->setControlMode(CM::Position);
            setLock(m_cm_lock->getSpoolPosition());
            m_cm_lock->enable();
            m_cm_lock->limits_exceeded();
            userLimitsExceeded();
        }
    }

    void PsychGui::setForceControl(bool isTest){
        if(isTest){
            double currentF = m_cm_test->getForce(1);
            m_cm_test->disable();
            m_cm_test->setControlMode(CM::Force);
            setTest(currentF);
            m_cm_test->enable();
            m_cm_test->limits_exceeded();
            userLimitsExceeded();

            if (m_pt.m_whichDof == PsychTest::Shear){ // test shear
                m_pt.m_userStimulusMin = m_pt.m_userparams.forceMin_t;
                m_pt.m_userStimulusMax = m_pt.m_maxRangePercent*m_pt.m_userparams.forceMax_t;
                m_pt.m_userStimulusContact = m_pt.m_userparams.forceCont_t;
            }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal
                m_pt.m_userStimulusMin = m_pt.m_userparams.forceMin_n;
                m_pt.m_userStimulusMax = m_pt.m_maxRangePercent*m_pt.m_userparams.forceMax_n;
                m_pt.m_userStimulusContact = m_pt.m_userparams.forceCont_n;
            }
        }else{
            double currentF = m_cm_lock->getForce(1);
            m_cm_lock->disable();
            m_cm_lock->setControlMode(CM::Force);
            setLock(currentF);
            m_cm_lock->enable();
            m_cm_lock->limits_exceeded();
            userLimitsExceeded();
        }
    }

    void PsychGui::setTest(double N) {
        m_cm_test->setControlValue(m_cm_test->scaleRefToCtrlValue(N));
        m_cm_test->limits_exceeded();
        userLimitsExceeded();
    }

    void PsychGui::setLock(double N) {
        m_cm_lock->setControlValue(m_cm_lock->scaleRefToCtrlValue(N));
        m_cm_lock->limits_exceeded();
        userLimitsExceeded();
    }

    void  PsychGui::getFNUpdate(){
        if (m_pt.m_whichDof == PsychTest::Shear){ // test shear
            m_ShearF = mean(m_cm_test->FBuff.get_vector());
            m_NormF = mean(m_cm_lock->FBuff.get_vector());
            m_ShearP = m_cm_test->getSpoolPosition();
            m_NormP = m_cm_lock->getSpoolPosition();  
        }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal
            m_NormF = mean(m_cm_test->FBuff.get_vector());
            m_ShearF = mean(m_cm_lock->FBuff.get_vector());
            m_NormP = m_cm_test->getSpoolPosition();
            m_ShearP = m_cm_lock->getSpoolPosition();
        }
    }

    void PsychGui::userLimitsExceeded(){
        if (m_pt.m_whichDof == PsychTest::Shear){ // test shear
            if(m_pt.m_controller == PsychTest::Force){
                if(m_cm_test->getForce(1) > m_pt.m_userparams.forceMax_t){
                    LOG(Warning) << "Exceeded User Shear Force Limit, " << m_pt.m_userparams.forceMax_t << " N with a value of " << m_cm_test->getForce(1) << " N.";
                    stopExp();
                } else if(m_cm_lock->getForce(1) > m_pt.m_userparams.forceMax_n){
                    LOG(Warning) << "Exceeded User Normal Force Limit, " << m_pt.m_userparams.forceMax_n << " N with a value of " << m_cm_lock->getForce(1) << " N.";
                    stopExp();
                }
            }

            if(m_pt.m_controller == PsychTest::Position){
                if(m_cm_test->getSpoolPosition() > m_pt.m_userparams.positionMax_t){
                    LOG(Warning) << "Exceeded User Shear Position Limit, " << m_pt.m_userparams.positionMax_t << " mm with a value of " << m_cm_test->getSpoolPosition() << " mm.";
                    stopExp();
                }else if(m_cm_lock->getSpoolPosition() > m_pt.m_userparams.positionMax_n){
                    LOG(Warning) << "Exceeded User Normal Position Limit, " << m_pt.m_userparams.positionMax_n << " mm with a value of " << m_cm_lock->getSpoolPosition() << " mm.";
                    stopExp();
                }
            }
    
        }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal

            if(m_pt.m_controller == PsychTest::Force){
                if(m_cm_test->getForce(1) > m_pt.m_userparams.forceMax_n){
                    LOG(Warning) << "Exceeded User Normal Force Limit, " << m_pt.m_userparams.forceMax_n << " N with a value of " << m_cm_test->getForce(1) << " N.";
                    stopExp();
                } else if(m_cm_lock->getForce(1) > m_pt.m_userparams.forceMax_t){
                    LOG(Warning) << "Exceeded User Shear Force Limit, " << m_pt.m_userparams.forceMax_t << " N with a value of " << m_cm_lock->getForce(1) << " N.";
                    stopExp();
                }
            }

            if(m_pt.m_controller == PsychTest::Position){
                if(m_cm_test->getSpoolPosition() > m_pt.m_userparams.positionMax_n){
                    LOG(Warning) << "Exceeded User Normal Position Limit, " << m_pt.m_userparams.positionMax_n << " mm with a value of " << m_cm_test->getSpoolPosition() << " mm.";
                    stopExp();
                } else if(m_cm_lock->getSpoolPosition() > m_pt.m_userparams.positionMax_t){
                    LOG(Warning) << "Exceeded User Shear Position Limit, " << m_pt.m_userparams.positionMax_t << " mm with a value of " << m_cm_lock->getSpoolPosition() << " mm.";
                    stopExp();
                }
            }
    
        }

    }

    void PsychGui::collectSensorData(PsychTest::WhichStim whichStim, int refOrder){

        if(((whichStim == PsychTest::First) && (refOrder == 1)) || ((whichStim == PsychTest::Second) && (refOrder == 2))){
            if (m_pt.m_whichDof == PsychTest::Shear){ // test shear
                m_ref_normF.push_back(m_cm_lock->getForce(1));
                m_ref_shearF.push_back(m_cm_test->getForce(1));
                m_ref_normP.push_back(m_cm_lock->getSpoolPosition());
                m_ref_shearP.push_back(m_cm_test->getSpoolPosition());
            }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal
                m_ref_normF.push_back(m_cm_test->getForce(1));
                m_ref_shearF.push_back(m_cm_lock->getForce(1));
                m_ref_normP.push_back(m_cm_test->getSpoolPosition());
                m_ref_shearP.push_back(m_cm_lock->getSpoolPosition());
            }
        }else if(((whichStim == PsychTest::Second) && (refOrder == 1)) || ((whichStim == PsychTest::First) && (refOrder == 2))){
            if (m_pt.m_whichDof == PsychTest::Shear){ // test shear
                m_comp_normF.push_back(m_cm_lock->getForce(1));
                m_comp_shearF.push_back(m_cm_test->getForce(1));
                m_comp_normP.push_back(m_cm_lock->getSpoolPosition());
                m_comp_shearP.push_back(m_cm_test->getSpoolPosition());
            }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal
                m_comp_normF.push_back(m_cm_test->getForce(1));
                m_comp_shearF.push_back(m_cm_lock->getForce(1));
                m_comp_normP.push_back(m_cm_test->getSpoolPosition());
                m_comp_shearP.push_back(m_cm_lock->getSpoolPosition());
            }
        }else if(whichStim == PsychTest::Adjust){
            if (m_pt.m_whichDof == PsychTest::Shear){ // test shear
                m_adjust_normF.push_back(m_cm_lock->getForce(1));
                m_adjust_shearF.push_back(m_cm_test->getForce(1));
                m_adjust_normP.push_back(m_cm_lock->getSpoolPosition());
                m_adjust_shearP.push_back(m_cm_test->getSpoolPosition());
            }else if (m_pt.m_whichDof == PsychTest::Normal){ // test normal
                m_adjust_normF.push_back(m_cm_test->getForce(1));
                m_adjust_shearF.push_back(m_cm_lock->getForce(1));
                m_adjust_normP.push_back(m_cm_test->getSpoolPosition());
                m_adjust_shearP.push_back(m_cm_lock->getSpoolPosition());
            }
        }
    }

    void PsychGui::avgSensorData(){
        m_ref_avgNormF = mean(m_ref_normF.get_vector());
        m_ref_avgShearF = mean(m_ref_shearF.get_vector());
        m_ref_avgNormP = mean(m_ref_normP.get_vector());
        m_ref_avgShearP = mean(m_ref_shearP.get_vector());

        m_comp_avgNormF = mean(m_comp_normF.get_vector());
        m_comp_avgShearF = mean(m_comp_shearF.get_vector());
        m_comp_avgNormP = mean(m_comp_normP.get_vector());
        m_comp_avgShearP = mean(m_comp_shearP.get_vector());

        m_adjust_avgNormF = mean(m_adjust_normF.get_vector());
        m_adjust_avgShearF = mean(m_adjust_shearF.get_vector());
        m_adjust_avgNormP = mean(m_adjust_normP.get_vector());
        m_adjust_avgShearP = mean(m_adjust_shearP.get_vector());
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

        if(m_pt.m_whichDof == PsychTest::Shear){
            ImGui::LabelText("Normal Encoder", "%i", m_cm_lock->getEncoderCounts());
            ImGui::LabelText("Shear Encoder", "%i", m_cm_test->getEncoderCounts());
        }else{
            ImGui::LabelText("Normal Encoder", "%i", m_cm_test->getEncoderCounts());
            ImGui::LabelText("Shear Encoder", "%i", m_cm_lock->getEncoderCounts());
        }

        // ImGui::Separator();

        // if(!paused){
        // torCmd.AddPoint(t, m_cm_test->m_torque* 1.0f);
        // }

        // ImPlot::SetNextPlotLimitsX(t - m_history, t,  !paused ? ImGuiCond_Always : ImGuiCond_Once);
        // if (ImPlot::BeginPlot("##torques", NULL, NULL, ImVec2(-1,200), 0, 0, 0)) {
        //     ImPlot::PlotLine("commanded torque", &torCmd.Data[0].x, &torCmd.Data[0].y, torCmd.Data.size(), torCmd.Offset, 2 * sizeof(float));
        //     ImPlot::EndPlot();
        // }
            
        
        ImGui::PopItemWidth();

        ImGui::Separator();

        if(!paused){
        lockForce.AddPoint(t, m_cm_lock->getForce(1)* 1.0f);
        lockPosition.AddPoint(t, m_cm_lock->getSpoolPosition()* 1.0f);
        testForce.AddPoint(t, m_cm_test->getForce(1)* 1.0f);
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

