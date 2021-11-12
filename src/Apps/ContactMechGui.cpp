#include "ContactMechGui.hpp"

// Written by Janelle Clark with Nathan Dunkelberger

using namespace mahi::gui;
using namespace mahi::util;
using namespace mahi::robo;

ContactMechGui::ContactMechGui(int subject, WhichExp whichExp, WhichDof whichDOF) : 
    Application(600,600,"Contact Mechanics Test (Subject " + std::to_string(subject) + ")" ,false),
    ts(),
    filename_timeseries("C:/Git/TactilePsychophysics/data/" + expchoice[whichExp] + "/_subject_" + std::to_string(subject) + "_timeseries_" + dofChoice[whichDOF] + "_dof_" + expchoice[whichExp] + "_exp_" + std::to_string(whichExp) + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv"),
    csv_timeseries(filename_timeseries),
    filename("C:/Git/TactilePsychophysics/data/" + expchoice[whichExp] + "/_subject_" + std::to_string(subject) + "_poiData_" + dofChoice[whichDOF] + "_dof_" + expchoice[whichExp] + "_exp_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv"),
    csv(filename),
    m_ftc()
    {       
        std::cout << "dofChoice[whichDOF]" << dofChoice[whichDOF] << std::endl;   
        m_subject = subject;
        m_whichExp = whichExp;
        m_whichDof = whichDOF;
        
        connectToIO();
        importUserHardwareParams();
        set_frame_limit(90_Hz);
        m_ftc = FTC(&m_hub.daq.AI[0], &m_hub.daq.AI[1], &m_hub.daq.AI[2], &m_hub.daq.AI[3], &m_hub.daq.AI[4], &m_hub.daq.AI[5], "FT06833.cal");
    
        
        //write output variables, time series and just poi
        std::cout << "write output variables" << std::endl;
        writeOutputVariables(csv_timeseries);
        writeOutputVariables(csv);

    }

    ContactMechGui::~ContactMechGui() {
        stopExp();
    }

    void ContactMechGui::update() {


        ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{600,1000}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::BeginDisabled(m_testmode == ContactMechGui::Run);

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
            ImGui::BulletText(("You've intiated a " + method[m_whichExp] + " Experiment with the following settings:").c_str());
            ImGui::BulletText("For Subject %i",  m_subject);
            ImGui::BulletText("Testing the %s Direction", dofChoice[m_whichDof]); // Hardware specific
            ImGui::Separator();
            ImGui::SetNextItemWidth(100);
            m_testmode = ContactMechGui::SetUp;

            if (ImGui::Button("Confirm Settings",ImVec2(-1,0))) {
                flag_confirm_exp_settings = 1;
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

        // after bring to contact, back to start position
        if (flag_bring_start && !flag_lock_extra_dofs){
            if (ImGui::Button("Lock Extra Dofs",ImVec2(-1,0))){
                start_coroutine(lockExtraDofs());
                flag_lock_extra_dofs = 1;
            }
        }
        
        // after bring to contact, back to start position
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
                lockExtraDofs();

                if (m_whichExp != ContactMechGui::Cycle){
                    start_coroutine(runICSRExperiment());
                    std::cout << "start multiple coroutines?" << std::endl;
                }else{
                    start_coroutine(runCycleExperiment());
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

        if(m_debug)
            plotWindow();

        // idle mode
        if (m_testmode == ContactMechGui::Idle) {
            ImGui::Text("Idle Mode, cycle %f out of 5", m_cyclenum);
            //When not coducting trials, go to neutral contact position
            if(!m_flag_first_to_start && flag_start_calibration){
                std::cout << "m_flag_first_to_start" << m_flag_first_to_start << std::endl;
                start_coroutine(bringToStartPosition());
                m_flag_first_to_start = 1;
            }
        }

        // collect time series info
        writeOutputData(csv_timeseries);
        ImGui::End();     
    }

    void ContactMechGui::moveConstVel(double elapsed, TestLockDof isTest, WhichSpeed whichSpeed, double start, double stop){ // start and stop can be force or position
        bool isIncreasing = start < stop;
        double sign = isIncreasing ? 1 : -1;

        double vel = (whichSpeed == Experiment) ? m_params.stimulus_velocity : m_params.travel_velocity;

        if (isTest == Test){
            m_targetPosTest += sign*vel*elapsed;
            ContactMechGui::setTest(m_targetPosTest);
            //std::cout << "m_targetPosTest " << m_targetPosTest << "current position" << getTestPos() << std::endl;
        }else if (isTest == Lock) {
            m_targetPosLock += sign*vel*elapsed;
            ContactMechGui::setLock(m_targetPosLock);
        }
    }

    void ContactMechGui::updateQuery(){
        getFNUpdate();
        m_q.time = time().as_milliseconds();
        m_q.testmode = m_testmode;
        m_q.whichExp = m_whichExp;
        m_q.whichDof = m_whichDof;
        m_q.controller = m_controller;
        m_q.poi = m_poi;
        m_q.cyclenum = m_cyclenum;
        m_q.Fn = m_Fn;
        m_q.Ft = m_Ft;
        m_q.deltaN = m_deltaN;
        m_q.deltaT = m_deltaT;
    }

    void ContactMechGui::writeOutputVariables(Csv& csv){
        csv.write_row("Time", "Mode","Dof","ControlType", "PointOfInterest","Cycle","Fn","Ft","deltaN","deltaS", "eeRadius", "CombinedE","ContactRadius","PlanarA","SphericalA","MeanStress","MeanStrain","ElasticStrainEnergy_N","YoungModulusNS","PoissonNS","CouplingBetaNS","ShearModulusNS","ComplianceN_NS","ComplianceT_NS","CentroidX","CentroidY","CentroidZ");
    }
    
    void ContactMechGui::writeOutputData(Csv& csv){
        updateQuery();
        m_q_hz_ns = m_hz.makeQuery_TanNoSlip(m_R, m_q.Fn, m_q.Ft, m_q.deltaN, m_q.deltaT);
        m_ftc_centroid = m_ftc.getCentroid();
        csv.write_row(time().as_seconds(), m_q.testmode, m_q.whichDof, m_q.controller, m_q.poi, m_q.cyclenum, m_q.Fn, m_q.Ft, m_q.deltaN, m_q.deltaT, m_q_hz_ns.R, m_q_hz_ns.combinedE, m_q_hz_ns.a, m_q_hz_ns.planarA, m_q_hz_ns.sphericalA, m_q_hz_ns.meanStress, m_q_hz_ns.meanStrain, m_q_hz_ns.Wn, m_q_hz_ns.E, m_q_hz_ns.v, m_q_hz_ns.couplingP, m_q_hz_ns.G, m_q_hz_ns.complianceN, m_q_hz_ns.complianceT, m_ftc_centroid[0], m_ftc_centroid[1], m_ftc_centroid[2]);
    }


    //////////////////////////////////////////////////////////////////////////////////////
    //           Indentation, Creep, and Stress Relaxation Experiment-Specific Functions
    //////////////////////////////////////////////////////////////////////////////////////

    Enumerator ContactMechGui::runICSRExperiment() {
        m_testmode = ContactMechGui::Run;
        std::cout << "set to run mode" << std::endl;
        m_flag_first_to_start = 0;
        
        
        // Run trials
        set_window_title("CM " + method[m_whichExp] + " (Subject " + std::to_string(m_subject) + ") (Test " + dofChoice[m_whichDof] + " dof)"); // Hardware specific

        // beginning delay
        std::cout << "1 second delay" << std::endl;
        double elapsed = 0;
        while (elapsed < 1) {
            elapsed += delta_time().as_seconds();
            co_yield nullptr;
        }

        // n trials depends on the experiment
        std::cout << "Set exp trials, force, and hold time" << std::endl;
        int n_trials;
        double finalForce;
        double holdTime;
        if (m_whichExp == Ind){
            n_trials =  m_params.n_ind_trials;
            finalForce = (m_whichDof == Normal) ? m_params.ind_final_force_n : m_params.ind_final_force_t;
            holdTime = 0.1;
        }else if (m_whichExp == Creep){
            n_trials =  m_params.n_creep_trials;
            finalForce = (m_whichDof == Normal) ? m_params.creep_final_force_n : m_params.creep_final_force_t;
            holdTime = m_params.creep_hold;
        }else if (m_whichExp == Relax){
            n_trials =  m_params.n_relax_trials;
            finalForce = (m_whichDof == Normal) ? m_params.relax_final_force_n : m_params.relax_final_force_t;
            holdTime = m_params.relax_hold;
        }

        if(m_whichDof == Shear){
            finalForce = 1.5;
        }

        m_cyclenum = 1;
        while (m_cyclenum < n_trials) {
            std::cout << "start trial " << m_cyclenum << std::endl;
            ////////////////////////////////////// run trial
            // go to intitial cycle stimulus, at contact
            if(m_whichDof == Shear){
                // move normal dof to testing location
                std::cout << "     move normal direction to 85% of the range for testing  (t)" << std::endl;
                m_targetPosLock = m_cm_lock->getSpoolPosition();
                while (m_cm_lock->getSpoolPosition() < m_userShearTestNormPos) {
                    moveConstVel(delta_time().as_seconds(), Lock, Calibration, m_cm_lock->getSpoolPosition(), m_userShearTestNormPos);
                    co_yield nullptr;
                }
                setLock(m_userShearTestNormPos);
            }

            std::cout << "      from start to min, stop at " <<  m_params.initial_force << " N" << std::endl;
            m_targetPosTest = getTestPos();
            while (getTestForce() < m_params.initial_force) { // normal force less than 1G
                std::cout << "      intitial: move down to " << m_params.initial_force << " N, currently at " << getTestForce() << " N" << std::endl;
                moveConstVel(delta_time().as_seconds(), Test, Experiment, getTestForce(), m_params.initial_force); 
                co_yield nullptr;
            } 
            m_poi = Min;
            writeOutputData(csv);
            m_poi = Other;
            std::cout << "      at " << currpoi[m_poi] << std::endl;

            // increase stimulus to peak
            m_targetPosTest = getTestPos();
            std::cout << "      from min to final, stop at " <<  finalForce << " N, currently " << getTestForce() << " N" << std::endl;
            while (getTestForce() < finalForce) { // normal force less than 1G
                std::cout << "      max: move down to " << finalForce << " N, currently at " << getTestForce() << " N" << std::endl;
                moveConstVel(delta_time().as_seconds(), Test, Experiment, getTestForce(), finalForce);
                co_yield nullptr;
            }
            std::cout << "      finished moving " << m_cyclenum << std::endl;
            m_poi = (m_whichExp == Ind) ? Peak : HoldInitial;
            writeOutputData(csv);
            m_poi = Other;
            std::cout << "      at " << currpoi[m_poi] << std::endl;

            // Switch controllers to Force Control for Creep Experiment
            if(m_whichExp == Creep)
                setForceControl(Test);
            
            // hold peak for some amount
            elapsed = 0;
            while (elapsed < holdTime) {
                std::cout << "      hold: for " << holdTime << " s, currently at " << elapsed << " s" << std::endl;
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            } 
            if(m_whichExp != Ind){
                m_poi = HoldFinal;
                writeOutputData(csv);
                m_poi = Other;
            }

            // Switch controllers back to Position Control for Creep Experiment
            if(m_whichExp == Creep)
                setPositionControl(Test);

            std::cout << "      from final to min, stop at " <<  m_params.initial_force << " N" << std::endl;
            // decrease to intial cycle stimulus, at contact
            m_targetPosTest = getTestPos();
            while (getTestForce() > m_params.initial_force) { // normal force less than 1G
                std::cout << "      min: move up to " << m_params.initial_force << " N, currently at" << getTestForce() << " N" << std::endl;
                moveConstVel(delta_time().as_seconds(), Test, Experiment, getTestForce(), m_params.initial_force);
                co_yield nullptr;
            }
            m_poi = Min;
            writeOutputData(csv);
            m_poi = Other;

            // move back to start position
            if(m_whichDof == Normal){
                std::cout << "      from min to start, stop at " <<  m_params.start_height << " mm" << std::endl;
                // decrease to starting point, at contact
                m_targetPosTest = getTestPos();
                while (m_targetPosTest > m_params.start_height) {
                    std::cout << "      start: move up to " << m_params.start_height << " mm, currently at" << getTestPos() << " mm" << std::endl;
                    moveConstVel(delta_time().as_seconds(), Test, Experiment, getTestPos(), m_params.start_height);
                    co_yield nullptr;
                }
            }else if(m_whichDof == Shear){
                std::cout << "      from min to start, stop at 0.0 mm" << std::endl;
                // decrease to starting point, at contact
                m_targetPosTest = getTestPos();
                while (m_targetPosTest > 0.0) {
                    std::cout << "      start: over to " << 0.0 << " mm, currently at " << getTestPos() << " mm" << std::endl;
                    moveConstVel(delta_time().as_seconds(), Test, Experiment, getTestPos(), 0.0);
                    co_yield nullptr;
                }

                std::cout << "      lift tactor to start position, stop at " <<  m_params.start_height << " mm" << std::endl;
                // decrease to starting point, at contact
                m_targetPosLock = getLockPos();
                while (m_targetPosLock > m_params.start_height) {
                    std::cout << "      start: move up to " << m_params.start_height << " mm, currently at" << getLockPos() << " mm" << std::endl;
                    moveConstVel(delta_time().as_seconds(), Lock, Experiment, getLockPos(), m_params.start_height);
                    co_yield nullptr;
                }
            }
            m_poi = Start;
            writeOutputData(csv);
            m_poi = Other;

            // thirty second delay unless last trial
            if ((m_cyclenum + 1) < m_params.n_ind_trials){
                double remaining = 30;
                while (remaining > 0) {
                    ImGui::Text("Break (%.3f)", remaining);
                    ImGui::End();
                    remaining -= delta_time().as_seconds();
                    co_yield nullptr;
                }
            }
            m_cyclenum++;
        }  // for trials   

        double remaining = 30;
        while (remaining > 0) {
            ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            ImGui::Text("End of Experiment. Notify experimentor to remove you from the device");
            ImGui::End();
            remaining -= delta_time().as_seconds();
            co_yield nullptr;
        } 
        
        m_testmode = ContactMechGui::Idle;
        set_window_title("CM " + method[m_whichExp] + " (Subject " + std::to_string(m_subject) + ") (Test " + dofChoice[m_whichDof] + " dof)"); // Hardware specific
    }
    
    //////////////////////////////////////////////////////////////////////////////////////
    //           Cycle Between User Bounds Specific Functions
    //////////////////////////////////////////////////////////////////////////////////////
    Enumerator ContactMechGui::runCycleExperiment(){
        m_testmode = ContactMechGui::Run;
        
        m_flag_first_to_start = 0;

        // Run trials
        set_window_title("CM " + method[m_whichExp] + " (Subject " + std::to_string(m_subject) + ") (Test " + dofChoice[m_whichDof] + " dof)"); // Hardware specific

        // beginning delay
        double elapsed = 0;
        while (elapsed < 1) {
            elapsed += delta_time().as_seconds();
            co_yield nullptr;
        }

        m_cyclenum = 1;
        while (m_cyclenum < m_params.n_cycle_cycles) {
            std::cout << "start trial " << m_cyclenum << std::endl;
            std::cout << " from start to min, stop at " <<  m_userStimulusPosMin << " mm" << std::endl;
            ////////////////////////////////////// run trial
            if(m_cyclenum == 1){
                // go to intiial cycle stimulus, at contact
                m_targetPosTest = m_cm_test->getSpoolPosition();
                while(m_targetPosTest < m_userStimulusPosMin) { // user absolute threshold
                    moveConstVel(delta_time().as_seconds(), Test, Experiment, m_cm_test->getSpoolPosition(), m_userStimulusPosMin);
                    co_yield nullptr;
                } 
                m_poi = Min;
                writeOutputData(csv);
                m_poi = Other;
            }

            std::cout << " from min to final, stop at " <<  m_userStimulusPosMax << " mm" << std::endl;

            // increase stimulus to peak
            m_targetPosTest = m_cm_test->getSpoolPosition();
            while(m_targetPosTest < m_userStimulusPosMax) { // user maximum comfort threshold
                moveConstVel(delta_time().as_seconds(), Test, Experiment, m_cm_test->getSpoolPosition(), m_userStimulusPosMax);
                co_yield nullptr;
            }
            m_poi = Peak;
            writeOutputData(csv);
            m_poi = Other;
            std::cout << " at peak " << std::endl;
            
            // hold peak for some amount
            elapsed = 0;
            while (elapsed < 0.1) {
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }

            std::cout << " from final to min, stop at " <<  m_userStimulusPosMin << " mm" << std::endl;
            // decrease to intial cycle stimulus, at contact
            m_targetPosTest = m_cm_test->getSpoolPosition();
            while(m_targetPosTest > m_userStimulusPosMin) {
                moveConstVel(delta_time().as_seconds(), Test, Experiment, m_cm_test->getSpoolPosition(), m_userStimulusPosMin);
                co_yield nullptr;
            }
            m_poi = Min;
            writeOutputData(csv);
            m_poi = Other;
            m_cyclenum++;
        }  // for trials   

        double remaining = 10;
        while (remaining > 0) {
            ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            ImGui::Text("Break (%.3f)", remaining);
            ImGui::End();
            remaining -= delta_time().as_seconds();
            co_yield nullptr;
        } 
        
        m_testmode = ContactMechGui::Idle; // will move back to start
        set_window_title("CM " + method[m_whichExp] + " (Subject " + std::to_string(m_subject) + ") (Test " + dofChoice[m_whichDof] + " dof)"); // Hardware specific
    }


    //////////////////////////////////////////////////////////////////////////////////////
    //           Hardware Specific Functions
    //////////////////////////////////////////////////////////////////////////////////////
     
     void ContactMechGui::connectToIO(){
        m_hub.createDevice(2, 0, 0, 0, 0, Axis::AxisZ, "FT06833.cal", {0,1,2,3,4,5},1);
        m_hub.createDevice(1, 2, 2, 1, 1, Axis::AxisX, "FT06833.cal", {0,1,2,3,4,5},1);
        
        if (m_whichDof == ContactMechGui::Shear) { // test shear direction
            m_cm_test = m_hub.getDevice(1);            
            m_cm_lock = m_hub.getDevice(2);
        }else if (m_whichDof == ContactMechGui::Normal){ // test normal direction
            m_cm_test = m_hub.getDevice(2);
            m_cm_lock = m_hub.getDevice(1);
        }

        m_cm_test->setForceFilterMode(CM::FilterMode::Lowpass);
        m_cm_lock->setForceFilterMode(CM::FilterMode::Lowpass);
     }

     void ContactMechGui::importUserHardwareParams(){
         std::cout << m_subject << std::endl;
        std::string user_calibration_file = "C:/Git/TactilePsychophysics/calibs/User/subject_" + std::to_string(m_subject) + ".json";
        m_up.importParams(user_calibration_file);
        m_userparams = m_up.getParams();

        std::string cm_norm_cal_file = "C:/Git/TactilePsychophysics/calibs/CM/dof_normal.json";
        std::string cm_tan_cal_file = "C:/Git/TactilePsychophysics/calibs/CM/dof_tangential.json";
        if (m_whichDof == ContactMechGui::Shear) {
            m_cm_test->importParams(cm_tan_cal_file);
            m_cm_lock->importParams(cm_norm_cal_file);
        }else if (m_whichDof == ContactMechGui::Normal){
            m_cm_test->importParams(cm_norm_cal_file);
            m_cm_lock->importParams(cm_tan_cal_file);
        }

        m_paramsCMTest = m_cm_test->getParams(); 
        m_paramsCMLock = m_cm_lock->getParams();

        m_hub.start();

        if (m_whichDof == ContactMechGui::Shear) { // test shear direction
            m_cm_lock->setPositionRange(m_paramsCMLock.positionMin, m_maxRangePercent*m_userparams.positionMax_n); // [mm] subject-specific
            m_cm_lock->setForceRange(m_paramsCMLock.forceMin, m_maxRangePercent*m_userparams.forceMax_n); // [N] subject-specific

            m_cm_test->setPositionRange(m_paramsCMTest.positionMin, m_maxRangePercent*m_userparams.positionMax_t); // [mm] subject-specific
            m_cm_test->setForceRange(m_paramsCMTest.forceMin, m_maxRangePercent*m_userparams.forceMax_t); // [N] subject-specific

        }else if (m_whichDof == ContactMechGui::Normal){ // test normal direction
            m_cm_lock->setPositionRange(m_paramsCMLock.positionMin, m_maxRangePercent*m_userparams.positionMax_t); // [mm] subject-specific
            m_cm_lock->setForceRange(m_paramsCMLock.forceMin, m_maxRangePercent*m_userparams.forceMax_t); // [N] subject-specific
            
            m_cm_test->setPositionRange(m_paramsCMTest.positionMin, m_maxRangePercent*m_userparams.positionMax_n); // [mm] subject-specific
            m_cm_test->setForceRange(m_paramsCMTest.forceMin, m_maxRangePercent*m_userparams.forceMax_n); // [N] subject-specific
        }

        if (m_whichDof == ContactMechGui::Shear){ // test shear
            m_userStimulusPosMin = m_userparams.positionMin_t;
            m_userStimulusPosMax = m_maxRangePercent*m_userparams.positionMax_t;
            m_userStimulusPosContact = m_userparams.positionCont_t;

            m_userStimulusForceMin = m_userparams.forceMin_t;
            m_userStimulusForceMax = m_maxRangePercent*m_userparams.forceMax_t;
            m_userStimulusForceContact = m_userparams.forceCont_t;
        }else if (m_whichDof == ContactMechGui::Normal){ // test normal
            m_userStimulusPosMin = m_userparams.positionMin_n;
            m_userStimulusPosMax = m_maxRangePercent*m_userparams.positionMax_n;
            m_userStimulusPosContact = m_userparams.positionCont_n;

            m_userStimulusForceMin = m_userparams.forceMin_n;
            m_userStimulusForceMax = m_maxRangePercent*m_userparams.forceMax_n;
            m_userStimulusForceContact = m_userparams.forceCont_n;
        }
        m_params.cycle_min_force = m_userparams.positionMin_n;
        m_params.cycle_peak_force = m_maxRangePercent*m_userparams.positionMax_n;

    }

    void ContactMechGui::stopExp(){
        m_cm_test->disable();
        m_cm_lock->disable();
        m_hub.stop();
    }

    void ContactMechGui::calibrate(){
        m_cm_test->zeroPosition();
        m_cm_lock->zeroPosition();

        m_cm_test->zeroForce();
        m_cm_lock->zeroForce();
    }

    Enumerator ContactMechGui::findContact(){
        std::cout << "Bring to contact" << std::endl;
        // disable while switching controllers
        setPositionControl(Test);
        setPositionControl(Lock);
        std::cout << "     set to position control in current location" << std::endl;

        if (m_whichDof == ContactMechGui::Shear){
            // keep the shear direction locked in the zero position
            std::cout << "     keep shear direction locked (t)" << std::endl;
            setTest(0);

            // move the normal direction to the contact position
            std::cout << "     ramp normal force until contact force is reached (t)" << std::endl;
            m_targetPosLock = m_cm_lock->getSpoolPosition();
            int num_above_force = 0;
            while (num_above_force < 20) {
                if (m_cm_lock->getForce() > m_userparams.forceCont_n) num_above_force++;
                else num_above_force = 0;
                moveConstVel(delta_time().as_seconds(), Lock, Calibration, 0, 1); // start and stop set to 0 and 1 so always increasing, keep it from dancing
                co_yield nullptr;
            }

            std::cout << "     force control to contact force" << std::endl;
            setForceControl(Lock);
            setLock( m_userparams.forceCont_n);
        }else {
            // keep the shear direction locked in the zero position
            std::cout << "     keep shear direction locked (n)" << std::endl;
            setLock(0);

            // move the normal direction to the contact position
            std::cout << "     ramp normal force until contact force is reached (n)" << std::endl;
            m_targetPosTest = m_cm_test->getSpoolPosition();
            int num_above_force = 0;
            while (num_above_force < 20) {
                if (m_cm_test->getForce() > m_userparams.forceCont_n) num_above_force++;
                else num_above_force = 0;
                moveConstVel(delta_time().as_seconds(), Test, Calibration, 0, 1); // start and stop set to 0 and 1 so always increasing, keep it from dancing
                co_yield nullptr;
            }
            std::cout << "     force control to contact force" << std::endl;
            setForceControl(Test);
            setTest( m_userparams.forceCont_n);
        }

        // Allow time to settle at contact force
        std::cout << "     allow time to settle at desired force" << std::endl;
        double elapsed = 0;
        while (elapsed < 1) {
            elapsed += delta_time().as_seconds();
            co_yield nullptr;
        }

        // Rezero at contact force so contact point is zero. Set back to position control
        std::cout << "     zero and set back to position control" << std::endl;
        m_cm_test->zeroPosition();
        setPositionControl(Test);
        m_cm_lock->zeroPosition();
        setPositionControl(Lock); 
        std::cout << "     READY" << std::endl;
    }

    Enumerator ContactMechGui::bringToStartPosition(){
        // disable while switching controllers
        std::cout << "Bring to start" << std::endl;
        if (m_whichDof == ContactMechGui::Shear){
            // move shear to center
            std::cout << "     set shear (test) to zero position" << std::endl;
            setTest(0);

            // move out of stimulus to starting position above the arm
            m_targetPosLock = m_cm_lock->getSpoolPosition();
            while (m_cm_lock->getSpoolPosition() > m_params.start_height) {
                moveConstVel(delta_time().as_seconds(), Lock, Calibration, m_cm_lock->getSpoolPosition(), m_params.start_height);
                co_yield nullptr;
            }
            std::cout << "     set normal (lock) to start position" << std::endl;
            setLock(m_params.start_height);
        } else if (m_whichDof == ContactMechGui::Normal){
            // move shear to center
            setLock(0);

            // move out of stimulus to starting position above the arm
            std::cout << "     set normal (test) to start position" << std::endl;
            std::cout << "start height: " << m_params.start_height << ", current height: " << m_cm_test->getSpoolPosition() << std::endl;
            m_targetPosTest = m_cm_test->getSpoolPosition();
            while (m_cm_test->getSpoolPosition() > m_params.start_height) {
                moveConstVel(delta_time().as_seconds(), Test, Calibration, m_cm_test->getSpoolPosition(), m_params.start_height);
                co_yield nullptr;
            }
            std::cout << "     set normal (test) to start position" << std::endl;
            setTest(m_params.start_height);
        }
        m_poi = Start;
        std::cout << "current poi: " << m_poi << std::endl;
        std::cout << "     READY" << std::endl;
    }


    Enumerator ContactMechGui::lockExtraDofs(){
        std::cout << "Lock extra dofs" << std::endl;
        m_userShearTestNormPos = 0.8*(m_maxRangePercent*m_userparams.positionMax_n - m_userparams.positionMin_n) + m_userparams.positionMin_n;

        std::cout << "     set lock in position control in the current location" << std::endl;
        if (m_whichDof == ContactMechGui::Shear){
            // move normal dof to testing location
            std::cout << "     move normal direction to 85% of the range for testing  (t)" << std::endl;
            m_targetPosLock = m_cm_lock->getSpoolPosition();
            while (m_cm_lock->getSpoolPosition() < m_userShearTestNormPos) {
                moveConstVel(delta_time().as_seconds(), Lock, Calibration, m_cm_lock->getSpoolPosition(), m_userShearTestNormPos);
                co_yield nullptr;
            }
            setLock(m_userShearTestNormPos);
        }else {            
            // move shear dof to testing location
            std::cout << "     move the shear direction to zero for testing  (n)" << std::endl;
            m_targetPosLock = m_cm_lock->getSpoolPosition();
            while (m_cm_lock->getSpoolPosition() < m_userparams.positionCont_t) {
                moveConstVel(delta_time().as_seconds(), Lock, Calibration, m_cm_lock->getSpoolPosition(), m_userparams.positionCont_t);
                co_yield nullptr;
            }
        }
        std::cout << "     READY" << std::endl;
    }

    Enumerator ContactMechGui::setControlDof(){ // assume already in position control
        std::cout << "Set control dof" << std::endl;
        // Set controller for testing DOF
        if (m_whichDof == ContactMechGui::Shear){ 
            // move shear dof to testing location
            std::cout << "     move the shear direction to zero for testing  (n)" << std::endl;
            m_targetPosTest = m_cm_test->getSpoolPosition();
            while (m_cm_test->getSpoolPosition() > m_userparams.positionCont_t) {
                moveConstVel(delta_time().as_seconds(), Test, Calibration, m_cm_test->getSpoolPosition(), m_userparams.positionCont_t);
                co_yield nullptr;
            }
            setTest(m_userparams.positionCont_t);
        }else if (m_whichDof == ContactMechGui::Normal){ // currently above the skin by an inch, need to move down
            // move normal dof to testing location
            std::cout << "     move norm to start height" << std::endl;
            m_targetPosTest = m_cm_test->getSpoolPosition();
            while (m_cm_test->getSpoolPosition() > m_params.start_height) {
                moveConstVel(delta_time().as_seconds(), Test, Calibration, m_cm_test->getSpoolPosition(), m_params.start_height);
                co_yield nullptr;
            }
            setTest(m_params.start_height);

            std::cout << "set controller m_params.start_height" << m_params.start_height << std::endl;
            std::cout << "m_cm_test->getSpoolPosition()" << m_cm_test->getSpoolPosition() << std::endl;
        }
        std::cout << "     READY" << std::endl;
    }

    void ContactMechGui::setPositionControl(TestLockDof isTest){
        if(isTest == Test){
            std::cout << "Set test dof to position control" << std::endl;
            double currentPos = m_cm_test->getSpoolPosition();
            m_cm_test->disable();
            m_cm_test->setControlMode(CM::Position);
            setTest(currentPos);
            m_cm_test->enable();
            m_cm_test->limits_exceeded();
            userLimitsExceeded();

        }else{
            std::cout << "Set lock dof to position control" << std::endl;
            double currentPos = m_cm_lock->getSpoolPosition();
            m_cm_lock->disable();
            m_cm_lock->setControlMode(CM::Position);
            setLock(m_cm_lock->getSpoolPosition());
            m_cm_lock->enable();
            m_cm_lock->limits_exceeded();
            userLimitsExceeded();
        }
    }

    void ContactMechGui::setForceControl(TestLockDof isTest){
        if(isTest==Test){
            std::cout << "Set test dof to force control" << std::endl;
            double currentF = m_cm_test->getForce(1);
            m_cm_test->disable();
            m_cm_test->setControlMode(CM::Force);
            setTest(currentF);
            m_cm_test->enable();
            m_cm_test->limits_exceeded();
            userLimitsExceeded();

        }else{
            std::cout << "Set lock dof to force control" << std::endl;
            double currentF = m_cm_lock->getForce(1);
            m_cm_lock->disable();
            m_cm_lock->setControlMode(CM::Force);
            setLock(currentF);
            m_cm_lock->enable();
            m_cm_lock->limits_exceeded();
            userLimitsExceeded();
        }
    }

    void ContactMechGui::setTest(double N) {
        double cv = m_cm_test->scaleRefToCtrlValue(N);
        if (abs(cv) > 1.0){ LOG(Warning) << "test control value clamped between 0 and 1 from " << cv;}
        m_cm_test->setControlValue(cv);
        m_cm_test->limits_exceeded();
        userLimitsExceeded();
    }

    void ContactMechGui::setLock(double N) {
        double cv = m_cm_lock->scaleRefToCtrlValue(N);
        if (abs(cv) > 1.0){ LOG(Warning) << "lock control value clamped between 0 and 1 from " << cv;}
        m_cm_lock->setControlValue(m_cm_lock->scaleRefToCtrlValue(N));
        m_cm_lock->limits_exceeded();
        userLimitsExceeded();
    }

    void  ContactMechGui::getFNUpdate(){
        if (m_whichDof == ContactMechGui::Shear){ // test shear
            m_Ft = mean(m_cm_test->FBuff.get_vector());
            m_Fn = mean(m_cm_lock->FBuff.get_vector());
            m_deltaT = getTestPos();
            m_deltaN = getLockPos();
    
        }else if (m_whichDof == ContactMechGui::Normal){ // test normal
            //std::cout << "      normal dof" << m_cm_test->FBuff.get_vector() <<  std::endl;
            m_Fn = mean(m_cm_test->FBuff.get_vector());
            //std::cout << "      tan dof" << m_cm_lock->FBuff.get_vector() << std::endl;
            m_Ft = mean(m_cm_lock->FBuff.get_vector());
            m_deltaN = getTestPos();
            m_deltaT = getLockPos();
        }

    }

    void  ContactMechGui::userLimitsExceeded(){
        getFNUpdate();

        if(m_controller == Force){
            if(m_Ft > m_userparams.forceMax_t){
                LOG(Warning) << "Exceeded User Shear Force Limit, " << m_userparams.forceMax_t << " N with a value of " << getTestForce() << " N.";
                stopExp();
            } else if(m_Fn > m_userparams.forceMax_n){
                LOG(Warning) << "Exceeded User Normal Force Limit, " << m_userparams.forceMax_n << " N with a value of " << getLockForce() << " N.";
                stopExp();
            }
        }

        if(m_controller == Position){
            if(m_deltaT > m_userparams.positionMax_t){
                LOG(Warning) << "Exceeded User Shear Position Limit, " << m_userparams.positionMax_t << " mm with a value of " << getTestPos() << " mm.";
                stopExp();
            }else if(m_deltaN > m_userparams.positionMax_n){
                LOG(Warning) << "Exceeded User Normal Position Limit, " << m_userparams.positionMax_n << " mm with a value of " << getLockPos() << " mm.";
                stopExp();
            }
        }

    }
    

    double ContactMechGui::getTestPos(){
        return m_cm_test->getSpoolPosition();
    };

    double ContactMechGui::getLockPos(){
        return m_cm_lock->getSpoolPosition();
    };

    double ContactMechGui::getTestForce(){
        return m_cm_test->getForce();
    };

    double ContactMechGui::getLockForce(){
        return m_cm_lock->getForce();
    };


    void ContactMechGui::plotWindow(){

        float windowHeight = m_debug ? 1500.0 : -1;
        float windowWidth = m_debug ? 600.0 : -1;
        float buttonHeight = m_debug ? 50.0 : -1;

        ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{windowWidth,windowHeight}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::EndDisabled();
        ImGui::LabelText("Point of Interest", "%s", currpoi[m_poi]);
        ImGui::LabelText("Test Force", "%f", m_cm_test->getForce());
        ImGui::LabelText("Test Position", "%f", m_cm_test->getSpoolPosition());
        ImGui::LabelText("Lock Force", "%f", m_cm_lock->getForce());
        ImGui::LabelText("Lock Position", "%f", m_cm_lock->getSpoolPosition());
        
        static bool paused = false;
        if(ImGui::Button(paused ? "unpause" : "pause")) paused = !paused;
        
        t += ImGui::GetIO().DeltaTime;
        ImGui::SliderFloat("History",&m_history,1,30,"%.1f s");

        ImGui::Separator();

        if(!paused){
        lockForce.AddPoint(t, m_cm_lock->getForce()* 1.0f);
        lockPosition.AddPoint(t, m_cm_lock->getSpoolPosition()* 1.0f);
        testForce.AddPoint(t, m_cm_test->getForce()* 1.0f);
        testPosition.AddPoint(t, m_cm_test->getSpoolPosition()* 1.0f);
        testCmd.AddPoint(t, m_targetPosTest* 1.0f);
        lockCmd.AddPoint(t, m_targetPosLock* 1.0f);
        }

        ImGui::Separator();

        ImGui::Text("Force Over Position");

        ImPlot::SetNextPlotLimitsX(-2, 2,  !paused ? ImGuiCond_Always : ImGuiCond_Once);
        if (ImPlot::BeginPlot("##ForcePos", NULL, NULL, ImVec2(-1,200), 0, 0, 0)) {
            ImPlot::PlotLine("Locked CM", &lockPosition.Data[0].y, &lockForce.Data[0].y, lockForce.Data.size(), lockForce.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Test CM", &testPosition.Data[0].y, &testForce.Data[0].y, testForce.Data.size(), testForce.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }

        ImGui::Separator();
        ImGui::Text("Commanded Position Plots");

        ImPlot::SetNextPlotLimitsX(t - m_history, t,  !paused ? ImGuiCond_Always : ImGuiCond_Once);
        if (ImPlot::BeginPlot("##CmdPos", NULL, NULL, ImVec2(-1,200), 0, 0, 0)) {
            ImPlot::PlotLine("Locked CM - Cmd", &testCmd.Data[0].x, &testCmd.Data[0].y, testCmd.Data.size(), testCmd.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Test CM - Cmd", &lockCmd.Data[0].x, &lockCmd.Data[0].y, lockCmd.Data.size(), lockCmd.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }

        ImGui::Separator();
        ImGui::Text("Time Plots");

        ImPlot::SetNextPlotLimitsX(t - m_history, t,  !paused ? ImGuiCond_Always : ImGuiCond_Once);
        if (ImPlot::BeginPlot("##Forces", NULL, NULL, ImVec2(-1,200), 0, 0, 0)) {
            ImPlot::PlotLine("Locked CM - Force", &lockForce.Data[0].x, &lockForce.Data[0].y, lockForce.Data.size(), lockForce.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Test CM - Force", &testForce.Data[0].x, &testForce.Data[0].y, testForce.Data.size(), testForce.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }

        ImPlot::SetNextPlotLimitsX(t - m_history, t,  !paused ? ImGuiCond_Always : ImGuiCond_Once);
        if (ImPlot::BeginPlot("##Position", NULL, NULL, ImVec2(-1,200), 0, 0, 0)) {
            ImPlot::PlotLine("Locked CM - Position", &lockPosition.Data[0].x, &lockPosition.Data[0].y, lockPosition.Data.size(), lockPosition.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Test CM - Position", &testPosition.Data[0].x, &testPosition.Data[0].y, testPosition.Data.size(), testPosition.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }

        ImGui::End();
    }

