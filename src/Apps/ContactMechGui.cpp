#include "ContactMechGui.hpp"

using namespace mahi::gui;
using namespace mahi::util;
using namespace mahi::robo;

ContactMechGui::ContactMechGui(int subject, WhichExp whichExp, WhichDof whichDOF) : 
    Application(600,600,"Contact Mechanics Test (Subject " + std::to_string(subject) + ")" ,false)
    {          
        m_subject = subject;
        m_whichExp = whichExp;
        m_whichDof = whichDOF;
        
        connectToIO();
        importUserHardwareParams();
        initializeHardware();
        set_frame_limit(90_Hz);
    }

    ContactMechGui::~ContactMechGui() {
        stopExp();
    }

    void ContactMechGui::update() {

        ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{600,1000}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::BeginDisabled(m_testmode != ContactMechGui::Idle);

        // Flags to walk through menus
        static bool flag_confirm_exp_settings = 0;
        static bool flag_start_calibration = 0;
        static bool flag_bring_to_start = 0;
        static bool flag_study_started = 0;

        // Confirm experiment settings and subject number
        if (!flag_confirm_exp_settings){
            ImGui::BulletText(("You've intiated a " + method[m_whichExp] + " Experiment with the following settings:").c_str());
            ImGui::BulletText("For Subject %i",  m_subject);
            ImGui::BulletText("Testing the %s Direction", dofChoice[m_whichDof]); // Hardware specific
            ImGui::Separator();
            ImGui::SetNextItemWidth(100);

            if (ImGui::Button("Confirm Settings",ImVec2(-1,0))) {
                flag_confirm_exp_settings = 1;
            }
            
            ImGui::SetNextItemWidth(100);
            if (ImGui::Button("Cancel",ImVec2(-1,0))) {
                stopExp(); 
            }
        }

        // after settings are confirmed, calibrate device (zero sensors)
        if (flag_confirm_exp_settings && !flag_start_calibration){
            if (ImGui::Button("Calibrate",ImVec2(-1,0))){
                calibrate(); 
                flag_start_calibration = 1;
            }
        }

        // after the device is calibrated, bring to start position
        if (flag_start_calibration && !flag_bring_to_start){
            if (ImGui::Button("Bring to Start Position",ImVec2(-1,0))){
                bringToStartPosition(); // Question - start from zero or current position and go to contact, Fn > ???, and then pull back ???? mm
                flag_bring_to_start = 1;
            }
        }
        
        // start experiment or cancel for some reason
        if (flag_bring_to_start && !flag_study_started){
            
            if (ImGui::Button("Start Study",ImVec2(-1,0))){
                if (m_whichExp == !ContactMechGui::Cycle){
                    start_coroutine(runICSRExperiment());
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

        if(m_debug) // Question ???????????
            plotWindow();

        // idle mode
        if (m_testmode == ContactMechGui::Idle) {
            //When not coducting trials, go to neutral contact position
            bringToStartPosition();
        }
        ImGui::End();     
    }

    void ContactMechGui::moveConstVel(double elapsed, bool isTest, bool isIncreasing){ // elapsed in s
        double sign = isIncreasing ? 1 : -1;

        if (isTest){
            m_targetPosTest += sign*m_params.stimulus_velocity * elapsed;
            ContactMechGui::setTest(m_targetPosTest);
        }else{
            m_targetPosLock += sign*m_params.stimulus_velocity * elapsed;
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
        csv.write_row("Mode","Dof","ControlType", "PointOfInterest","Cycle","Fn","Ft","deltaN","deltaN");
    }
    
    void ContactMechGui::writeOutputData(Csv& csv){
        updateQuery();
        csv.write_row(m_q.testmode, m_q.whichDof, m_q.controller, m_q.poi, m_q.cyclenum, m_q.Fn, m_q.Ft, m_q.deltaN, m_q.deltaT);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    //           Indentation, Creep, and Stress Relaxation Experiment-Specific Functions
    //////////////////////////////////////////////////////////////////////////////////////

    Enumerator ContactMechGui::runICSRExperiment() {
        m_testmode = ContactMechGui::Run;
        Timestamp ts;

        // Set controllers for testing DOF
        lockExtraDofs();
        
        //Create data file
        std::string filename;
        filename = "C:/Git/TactilePsychophysics/data/_subject_" + std::to_string(m_subject) + "_" + dofChoice[m_whichDof] + "_" + expchoice[m_whichExp] + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
        Csv csv(filename);
        writeOutputVariables(csv);

        // Run trials
        set_window_title("CM " + method[m_whichExp] + " (Subject " + std::to_string(m_subject) + ") (Test " + dofChoice[m_whichDof] + " dof)"); // Hardware specific

        // beginning delay
        double elapsed = 0;
        while (elapsed < 1) {
            elapsed += delta_time().as_seconds();
            co_yield nullptr;
        }

        // n trials depends on the experiment
        int n_trials;
        double finalForce;
        double holdTime;
        if (m_whichExp == Ind){
            n_trials =  m_params.n_ind_trials;
            finalForce = m_params.ind_final_force;
            holdTime = 0.1;
        }else if (m_whichExp == Creep){
            n_trials =  m_params.n_creep_trials;
            finalForce = m_params.creep_final_force;
            holdTime = m_params.creep_hold;
        }else if (m_whichExp == Relax){
            n_trials =  m_params.n_relax_trials;
            finalForce = m_params.relax_final_force;
            holdTime = m_params.relax_hold;
        }

        m_cyclenum = 1;
        while (m_cyclenum < n_trials) {
            std::cout << " from start to min, stop at " <<  m_params.initial_force << " N" << std::endl;
            ////////////////////////////////////// run trial
            // go to intiial cycle stimulus, at contact
            while (getTestForce() < m_params.initial_force) { // normal force less than 1G
                moveConstVel(delta_time().as_seconds(), 1, 1); 
                co_yield nullptr;
            } 
            m_poi = Min;
            writeOutputData(csv);

            std::cout << " from min to final, stop at " <<  finalForce << " N" << std::endl;

            // increase stimulus to peak
            while (getTestForce() < finalForce) { // normal force less than 1G
                moveConstVel(delta_time().as_seconds(), 1, 1); 
                co_yield nullptr;
            }
            m_poi = (m_whichExp == Ind) ? Peak : HoldInitial;
            writeOutputData(csv);
            std::cout << " at " << currpoi[m_poi] << std::endl;

            // Switch controllers to Force Control for Creep Experiment
            if(m_whichExp == Creep)
                switchControllers();
            
            // hold peak for some amount
            elapsed = 0;
            while (elapsed < holdTime) {
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            } 
            if(m_whichExp != Ind){
                m_poi = HoldFinal;
                writeOutputData(csv);
            }

            // Switch controllers back to Position Control for Creep Experiment
            if(m_whichExp == Creep)
                switchControllers();

            std::cout << " from final to min, stop at " <<  m_params.initial_force << " N" << std::endl;
            // decrease to intial cycle stimulus, at contact
            while (getTestForce() > m_params.initial_force) { // normal force less than 1G
                moveConstVel(delta_time().as_seconds(), 1, 0);
                co_yield nullptr;
            }
            m_poi = Min;

            std::cout << " from min to start, stop at " <<  m_params.start_height << " mm" << std::endl;
            // decrease to starting point, at contact
            while (getTestPos() > m_params.start_height) { // normal force less than 1G
                moveConstVel(delta_time().as_seconds(), 1, 0);
                co_yield nullptr;
            }
            m_poi = Start;

            // thirty second delay unless last trial
            if ((m_cyclenum + 1) > m_params.n_ind_trials){
                std::cout << "last trial" << std::endl;
                elapsed = 0;
                while (elapsed < 30) {
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
            }
        }  // for trials   

        double remaining = 10;
        while (remaining > 0) {
            ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            ImGui::Text("Break (%.3f)", remaining);
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
        Timestamp ts;

        // Set controllers for testing DOF
        lockExtraDofs();
        
        //Create data file
        std::string filename;
        filename = "C:/Git/TactilePsychophysics/data/_subject_" + std::to_string(m_subject) + "_" + dofChoice[m_whichDof] + "_" + expchoice[m_whichExp] + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
        Csv csv(filename);
        writeOutputVariables(csv);

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
            std::cout << " from start to min, stop at " <<  m_userStimulusMin << " mm" << std::endl;
            ////////////////////////////////////// run trial
            if(m_cyclenum == 1){
                // go to intiial cycle stimulus, at contact
                while (getTestPos() < m_userStimulusMin) { // user absolute threshold
                    moveConstVel(delta_time().as_seconds(), 1, 1); 
                    co_yield nullptr;
                } 
                m_poi = Min;
                writeOutputData(csv);
            }

            std::cout << " from min to final, stop at " <<  m_userStimulusMax << " mm" << std::endl;

            // increase stimulus to peak
            while (getTestPos() < m_userStimulusMax) { // user maximum comfort threshold
                moveConstVel(delta_time().as_seconds(), 1, 1); 
                co_yield nullptr;
            }
            m_poi = Peak;
            writeOutputData(csv);
            std::cout << " at peak " << std::endl;
            
            // hold peak for some amount
            elapsed = 0;
            while (elapsed < 0.1) {
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }

            std::cout << " from final to min, stop at " <<  m_userStimulusMin << " mm" << std::endl;
            // decrease to intial cycle stimulus, at contact
            while (getTestPos() > m_userStimulusMin) {
                moveConstVel(delta_time().as_seconds(), 1, 0);
                co_yield nullptr;
            }
            m_poi = Min;
            writeOutputData(csv);

            // thirty second delay unless last trial
            if ((m_cyclenum + 1) > m_params.n_ind_trials){
                std::cout << "last trial" << std::endl;
                elapsed = 0;
                while (elapsed < 30) {
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
            }
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
     }

     void ContactMechGui::importUserHardwareParams(){
         std::cout << m_subject << std::endl;
        std::string cm_calibration_file = "C:/Git/TactilePsychophysics/calibs/CM/subject_" + std::to_string(m_subject) + ".json";
        m_cm_test->importParams(cm_calibration_file);
        m_cm_lock->importParams(cm_calibration_file);
        m_CMparams = m_cm_test->getParams(); // don't need separate ones for m_cm_lock, same for each subject

        std::string user_calibration_file = "C:/Git/TactilePsychophysics/calibs/User/subject_" + std::to_string(m_subject) + ".json";
        m_up.importParams(user_calibration_file);
        m_userparams = m_up.getParams();

        m_hub.start();

        if (m_whichDof == ContactMechGui::Shear) { // test shear direction
            m_cm_lock->setPositionRange(m_userparams.positionMin_n, m_userparams.positionMax_n); // [mm] subject-specific
            m_cm_lock->setForceRange(m_userparams.forceMin_n, m_userparams.forceMax_n); // [N] subject-specific

            m_cm_test->setPositionRange(m_userparams.positionMin_t, m_userparams.positionMax_t); // [mm] subject-specific
            m_cm_test->setForceRange(m_userparams.forceMin_t, m_userparams.forceMax_t); // [N] subject-specific

        }else if (m_whichDof == ContactMechGui::Normal){ // test normal direction
            m_cm_lock->setPositionRange(m_userparams.positionMin_t, m_userparams.positionMax_t); // [mm] subject-specific
            m_cm_lock->setForceRange(m_userparams.forceMin_t, m_userparams.forceMax_t); // [N] subject-specific
            
            m_cm_test->setPositionRange(m_userparams.positionMin_n, m_userparams.positionMax_n); // [mm] subject-specific
            m_cm_test->setForceRange(m_userparams.forceMin_n, m_userparams.forceMax_n); // [N] subject-specific
        }
    }

    void ContactMechGui::initializeHardware(){
        m_controller = Position;
        m_cm_test->setControlMode(CM::Position);
        m_cm_test->setControlValue(0);
        //m_cm_test->enable();

        m_cm_lock->setControlMode(CM::Position);
        m_cm_lock->setControlValue(0);
        //m_cm_lock->enable();
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

    void ContactMechGui::bringToStartPosition(){
       
        m_cm_lock->setControlMode(CM::Position); // won't change throughout the trial (should be position control)
        m_cm_test->setControlMode(CM::Position); // start out at contact position

        if (m_whichDof == ContactMechGui::Shear){ // test shear, normal is locked
            m_cm_lock->setControlValue(m_cm_lock->scaleRefToCtrlValue(m_userparams.positionCont_n)); // ?????? value may need to change to calibration position, 0 for test normal, 0.75 for test shear?
            m_cm_test->setControlValue(m_cm_test->scaleRefToCtrlValue(m_userparams.positionCont_t));
        }
        else if (m_whichDof ==ContactMechGui::Normal){ // test normal, shear is locked
            m_cm_lock->setControlValue(m_cm_lock->scaleRefToCtrlValue(m_userparams.positionCont_t)); // shear dof should be centered
            m_cm_test->setControlValue(m_cm_test->scaleRefToCtrlValue(m_userparams.positionCont_n));
        }

        m_cm_lock->limits_exceeded();                
        m_cm_test->limits_exceeded();
        m_poi = Start;
    }

    void ContactMechGui::lockExtraDofs(){
        //  ??????????????? Add gradual movement to intial position?
        // ?????? Move shear first, then normal if no contact, normal then shear if there is contact?
        m_cm_lock->setControlMode(CM::Position); // won't change throughout the trial (should be position control)
        if (m_whichDof == ContactMechGui::Shear) // test shear, normal is locked
            m_cm_lock->setControlValue(0.75); // ?????? value may need to change to calibration position, 0 for test normal, 0.75 for test shear?
        else if (m_whichDof ==ContactMechGui::Normal) // test normal, shear is locked
            m_cm_lock->setControlValue(0.0); // shear dof should be centered
        m_cm_lock->limits_exceeded();
    }

    void ContactMechGui::switchControllers(){
        if (m_controller == Position){
            std::cout << "Switch from position to force control" << std::endl;
            double currF = m_cm_test->getForce();
            m_controller = Force;
            
            m_userStimulusMin = m_CMparams.forceMin;
            m_userStimulusMax = m_CMparams.forceMax;
            m_cm_test->setControlMode(CM::Force);

            if (m_whichDof == ContactMechGui::Shear){ // test shear
                m_userStimulusContact = m_userparams.forceCont_t;
            }else if (m_whichDof == ContactMechGui::Normal){ // test normal
                m_userStimulusContact = m_userparams.forceCont_n;
            }
            m_cm_test->setControlValue(m_cm_test->scaleRefToCtrlValue(currF)); // 0 for centered shear, contact pressure/position for normal force
            m_cm_test->limits_exceeded();

        }else if (m_controller == Force){
            std::cout << "Switch from force to position control" << std::endl;
            double currP = m_cm_test->getSpoolPosition();
            m_controller = Position;

            m_userStimulusMin = m_CMparams.positionMin;
            m_userStimulusMax = m_CMparams.positionMax;
            m_cm_test->setControlMode(CM::Position);
            if (m_whichDof == ContactMechGui::Shear){ // test shear
                m_userStimulusContact = m_userparams.positionCont_t;
            }else if (m_whichDof == ContactMechGui::Normal){ // test normal
                m_userStimulusContact = m_userparams.positionCont_n;
            }
            m_cm_test->setControlValue(m_cm_test->scaleRefToCtrlValue(currP)); // 0 for centered shear, contact pressure/position for normal force
            m_cm_test->limits_exceeded();
        }
        
    }

    void ContactMechGui::setTest(double N) {
        m_cm_test->setControlValue(m_cm_test->scaleRefToCtrlValue(N));
        m_cm_test->limits_exceeded();
        userLimitsExceeded();
    }

    void ContactMechGui::setLock(double N) {
        m_cm_lock->setControlValue(m_cm_lock->scaleRefToCtrlValue(N));
        m_cm_lock->limits_exceeded();
        userLimitsExceeded();
    }

    void  ContactMechGui::getFNUpdate(){
        if (m_whichDof == ContactMechGui::Shear){ // test shear
            m_Ft = getTestForce();
            m_Fn = getLockForce();
            m_deltaT = getTestPos();
            m_deltaN = getLockPos();
    
        }else if (m_whichDof == ContactMechGui::Normal){ // test normal
            m_Fn = getTestForce();
            m_Ft = getLockForce();
            m_deltaN = getTestPos();
            m_deltaT = getLockPos();
        }

    }

    void  ContactMechGui::userLimitsExceeded(){
        getFNUpdate();
        if(m_Ft > m_userparams.forceMax_t){
            LOG(Warning) << "Exceeded User Shear Force Limit, " << m_userparams.forceMax_t << " N with a value of " << getTestForce() << " N.";
            stopExp();
        } else if(m_Fn > m_userparams.forceMax_n){
            LOG(Warning) << "Exceeded User Normal Force Limit, " << m_userparams.forceMax_n << " N with a value of " << getLockForce() << " N.";
            stopExp();
        }
        
        if(m_deltaT > m_userparams.positionMax_t){
            LOG(Warning) << "Exceeded User Shear Position Limit, " << m_userparams.positionMax_t << " mm with a value of " << getTestPos() << " mm.";
            stopExp();
        }else if(m_deltaN > m_userparams.positionMax_n){
            LOG(Warning) << "Exceeded User Normal Position Limit, " << m_userparams.positionMax_n << " mm with a value of " << getLockPos() << " mm.";
            stopExp();
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
        ImGui::EndDisabled;
        ImGui::LabelText("Point of Interest", "%s", currpoi[m_poi]);
        ImGui::LabelText("Test Force", "%i", m_cm_test->getForce());
        ImGui::LabelText("Test Position", "%i", m_cm_test->getSpoolPosition());
        ImGui::LabelText("Lock Force", "%i", m_cm_lock->getForce());
        ImGui::LabelText("Lock Position", "%i", m_cm_lock->getSpoolPosition());
        
        static bool paused = false;
        if(ImGui::Button(paused ? "unpause" : "pause")) paused = !paused;
        ImGui::Separator();
        
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
        if (ImPlot::BeginPlot("##ForceOPos", NULL, NULL, ImVec2(-1,200), 0, 0, 0)) {
            ImPlot::PlotLine("Locked CM", &lockPosition.Data[0].y, &lockForce.Data[0].y, lockForce.Data.size(), lockForce.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Test CM", &testPosition.Data[0].y, &testForce.Data[0].y, testForce.Data.size(), testForce.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }

        ImGui::Separator();
        ImGui::Text("Commanded Position Plots");

        ImPlot::SetNextPlotLimitsX(t - m_history, t,  !paused ? ImGuiCond_Always : ImGuiCond_Once);
        if (ImPlot::BeginPlot("##CmdPos", NULL, NULL, ImVec2(-1,200), 0, 0, 0)) {
            ImPlot::PlotLine("Locked CM - Force", &testCmd.Data[0].x, &testCmd.Data[0].y, testCmd.Data.size(), testCmd.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Test CM - Force", &lockCmd.Data[0].x, &lockCmd.Data[0].y, lockCmd.Data.size(), lockCmd.Offset, 2*sizeof(float));
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

