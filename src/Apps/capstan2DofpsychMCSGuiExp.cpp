#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Robo.hpp>
#include <algorithm>
#include <random>
#include "CapstanModule.hpp"
#include "CMHub.hpp"
#include "PsychophysicalTesting.hpp"
#include "Util/XboxController.hpp"
#include <Mahi/Util/Logging/Log.hpp>

using namespace mahi::gui;
using namespace mahi::util;
using namespace mahi::robo;
using namespace xbox;

class PsychMCSGui : public Application {
public:

    // Method of Constant Stimuli - Psychophysical Study
    // Done in either force or position control
    // Assumes a 2 DOF device

    PsychMCSGui(int subject, PsychTest::WhichDof whichDOF, PsychTest::ControlType controller) : 
    Application(500,500,"CM Method of Constant Stimuli Study (Subject " + std::to_string(subject) + ")" ,false), 
    m_subject(subject),
    m_whichDOF(whichDOF),
    m_controller(controller),
    m_psychtest(m_subject, PsychTest::Params(), m_whichDOF, m_controller)
    { 
        m_hub.createDevice(1, 2, 2, 1, 1, Axis::AxisX, "FT06833.cal", {0,1,2,3,4,5});
        m_hub.createDevice(2, 0, 0, 0, 0, Axis::AxisZ, "FT06833.cal", {0,1,2,3,4,5});

        if (m_whichDOF == PsychTest::Shear) { // test shear direction
            m_cm_test = m_hub.getDevice(1);            
            m_cm_lock = m_hub.getDevice(2);
        }else if (m_whichDOF == PsychTest::Normal){ // test normal direction
            m_cm_test = m_hub.getDevice(2);
            m_cm_lock = m_hub.getDevice(1);
        }

        std::string cm_calibration_file = "/TactilePsychophysics/calibs/CM/subject_" + std::to_string(m_subject) + ".json";
        m_cm_test->importParams(cm_calibration_file);
        m_params = m_cm_test->getParams(); // don't need separate ones for m_cm_lock, same for each subject
        
        std::string user_calibration_file = "/TactilePsychophysics/calibs/User/subject_" + std::to_string(m_subject) + ".json";
        m_psychtest.importParams(user_calibration_file);
        m_userparams = m_psychtest.getParams();

        m_hub.start();

        if (m_whichDOF == PsychTest::Shear) { // test shear direction
            m_cm_lock->setPositionRange(m_userparams.positionMin_n, m_userparams.positionMax_n); // [mm] subject-specific
            m_cm_lock->setForceRange(m_userparams.forceMin_n, m_userparams.forceMax_n); // [N] subject-specific

            m_cm_test->setPositionRange(m_userparams.positionMin_t, m_userparams.positionMax_t); // [mm] subject-specific
            m_cm_test->setForceRange(m_userparams.forceMin_t, m_userparams.forceMax_t); // [N] subject-specific

        }else if (m_whichDOF == PsychTest::Normal){ // test normal direction
            m_cm_lock->setPositionRange(m_userparams.positionMin_t, m_userparams.positionMax_t); // [mm] subject-specific
            m_cm_lock->setForceRange(m_userparams.forceMin_t, m_userparams.forceMax_t); // [N] subject-specific
            
            m_cm_test->setPositionRange(m_userparams.positionMin_n, m_userparams.positionMax_n); // [mm] subject-specific
            m_cm_test->setForceRange(m_userparams.forceMin_n, m_userparams.forceMax_n); // [N] subject-specific
        }

        // Question ??????????? Add calibration (zeroing) for position and force 

        m_cm_test->setControlMode(CM::Force);
        m_cm_test->setControlValue(0);
        m_cm_test->enable();

        m_cm_lock->setControlMode(CM::Force);
        m_cm_lock->setControlValue(0);
        m_cm_lock->enable();

        if (!m_xbox.is_connected())
            LOG(Error) << "No Xbox controller detected!"; 

        set_frame_limit(90_Hz);
    }

    ~PsychMCSGui() {
        stopExp();
    }

    void update() override {

        ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::BeginDisabled(m_mode != PsychTest::Idle);

        // Initial Input
        static bool flag_confirm_settings = 0;

        if (!flag_confirm_settings){

            ImGui::Text("You've intiated a Method of Constant Stimuli Experiment with the following settings:");
            ImGui::BulletText("For Subject %i",  m_subject);
            //ImGui::BulletText("Testing the %d direction", m_whichDOF);
            //ImGui::BulletText("Using %s control", m_controller);
            ImGui::Separator();
            
            ImGui::SetNextItemWidth(100);
            if (ImGui::Button("Confirm Settings",ImVec2(-1,0))) {
                flag_confirm_settings = 1;
            }

            ImGui::SetNextItemWidth(100);
            if (ImGui::Button("Cancel",ImVec2(-1,0))) {
                stopExp();
            }
        }
        
        // replace above button
        if (flag_confirm_settings){
            if (ImGui::Button("Start Study",ImVec2(-1,0))){
                m_psychtest.buildStimTrials();
                start_coroutine(psychMCSStudy());
            }
        }

        ImGui::EndDisabled();

        // idle mode
        if (m_mode == PsychTest::Idle) {
            //When not in a trial, go to 0,0 position in normal and shear position ????????????????????????????????????
            // or maybe normal direction should lose contact, maybe in calibration 
            // determine contact point and return there, or do force control to find contact

            // add position and force to limits exceeded
            

            m_cm_test->setControlMode(CM::Position);
            //double cv = remap(m_psychtest.m_userStimulusContact, m_params.positionMin, m_params.positionMax, 0.0, 1.0); // ???? Question, can choose controller but not which dof
            m_cm_test->setControlValue(0.0);
            m_cm_test->limits_exceeded();

            m_cm_lock->setControlMode(CM::Position);
            //double cv = remap(m_psychtest.m_userStimulusContact, m_params.positionMin, m_params.positionMax, 0.0, 1.0);
            m_cm_lock->setControlValue(0.0);
            m_cm_lock->limits_exceeded();
        }
        ImGui::End();     
    }

    void responseWindow(PsychTest::WhichStim whichStim) {
        ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::BeginDisabled(true,1.0f);
        ImGui::Button("Which Stimulus Was Larger?", ImVec2(-1,0));
        if (whichStim == PsychTest::First)
            ImGui::PushStyleColor(ImGuiCol_Button, Blues::DeepSkyBlue);
        ImGui::Button("First", ImVec2(240,-1));
        if (whichStim == PsychTest::First)
            ImGui::PopStyleColor();
        ImGui::SameLine();
        if (whichStim == PsychTest::Second)
            ImGui::PushStyleColor(ImGuiCol_Button, Blues::DeepSkyBlue);
        ImGui::Button("Second", ImVec2(240,-1));   
        if (whichStim == PsychTest::Second)
            ImGui::PopStyleColor();
        ImGui::EndDisabled();
        ImGui::End();
    }

    Enumerator psychMCSStudy() {
        m_mode = PsychTest::MCS;
        Timestamp ts;

        // here going to (0,0) --- combine this code with IDLE mode? ???????????????????????????????????????
        // OR maybe normal direction should lose contact, maybe in calibration 
        // determine contact point and return there, or do force control to find contact
        // OR reamin at current position??

        // set controller for locked DOF ????????????? at current pos or a spot from the calibration data?
        // Question ??????????????? Add gradual movement to intial position?
        m_cm_lock->setControlMode(CM::Position); // won't change throughout the trial (should be position control)
        if (m_whichDOF == PsychTest::Shear) // test shear, normal is locked
            m_cm_lock->setControlValue(0.75); // ?????? value may need to change to calibration position, 0 for test normal, 0.75 for test shear?
        else if (m_whichDOF ==PsychTest::Normal) // test normal, shear is locked
            m_cm_lock->setControlValue(0.0); // shear dof should be centered
        m_cm_lock->limits_exceeded();

        // Set controller for testing DOF
        if (m_controller == PsychTest::Position){
            m_psychtest.m_userStimulusMin = m_params.positionMin;
            m_psychtest.m_userStimulusMax = m_params.positionMax;
            m_cm_test->setControlMode(CM::Position);
            if (m_whichDOF == PsychTest::Shear){ // test shear
                m_cm_test->setControlValue(0.0); // shear dof should be centered
                m_psychtest.m_jnd_stimulus_min = m_userparams.positionCont_t;
                m_psychtest.m_userStimulusContact = m_userparams.positionCont_t;
            }else if (m_whichDOF == PsychTest::Normal){ // test normal
                m_cm_test->setControlValue(0.0); // Question ?????? value may need to change to calibration/contact position
                m_psychtest.m_jnd_stimulus_min = m_userparams.positionCont_n;
                m_psychtest.m_userStimulusContact = m_userparams.positionCont_n;
            }
            m_cm_test->limits_exceeded();

        }else if (m_controller == PsychTest::Force){
            m_psychtest.m_userStimulusMin = m_params.forceMin;
            m_psychtest.m_userStimulusMax = m_params.forceMax;
            m_cm_test->setControlMode(CM::Force);
            if (m_whichDOF == PsychTest::Shear){ // test shear
                m_cm_test->setControlValue(0.0); // shear dof should start centered
                m_psychtest.m_jnd_stimulus_min = m_userparams.forceCont_t;
                m_psychtest.m_userStimulusContact = m_userparams.forceCont_t;
            }else if (m_whichDOF == PsychTest::Normal){ // test normal
                m_cm_test->setControlValue(0.0); // Question ?????? value may need to change to calibration/contact position
                m_psychtest.m_jnd_stimulus_min = m_userparams.forceCont_n;
                m_psychtest.m_userStimulusContact = m_userparams.forceCont_n;
            }
            m_cm_test->limits_exceeded();
        } 

        // Create data file
        std::string filename = "/TactilePsychophysics/data/MCS_jnd_subject_" + std::to_string(m_subject) + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
        Csv csv(filename);
        csv.write_row("Mode","ControlType","Trial","GenNum","Window","Level","Stimulus1","Stimulus2","Standard","Comparison","Correct","Answer","Greater","NormF1", "ShearF1", "NormP1", "ShearP1","NormF2", "ShearF2", "NormP2", "ShearP2" );
        double elapsed;
        for (int w = 0; w < m_userparams.n_msc_windows; ++w) {
            if (m_controller == PsychTest::Position){
                set_window_title("CM Method of Constant Stimuli Study (Subject " + std::to_string(m_subject) + ") (Position Control) (S" + std::to_string(w+1) + ")");
            
            }else if (m_controller == PsychTest::Force){
                set_window_title("CM Method of Constant Stimuli Study (Subject " + std::to_string(m_subject) + ") (Force Control) (S" + std::to_string(w+1) + ")");
            }
            // beginning delay
            elapsed = 0;
            while (elapsed < 1) {
                responseWindow(PsychTest::NA);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
            for (auto& trial : m_psychtest.m_stimulus_trials[w]) {
                // render first stimulus - ramp up
                elapsed = 0;
                while (elapsed < m_userparams.msc_stimulus_time) {
                    double stimVal = Tween::Linear(m_psychtest.m_jnd_stimulus_min, trial.stimulus1, (float)(elapsed / m_userparams.msc_stimulus_time));
                    setStimulus(stimVal);
                    responseWindow(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                // render first stimulus - hold stim and record position and force info
                setStimulus(trial.stimulus1);
                elapsed = 0;
                while (elapsed < m_userparams.msc_stimulus_time) {
                    responseWindow(PsychTest::First);

                    // these are overwritten each second during the hold. Any one be a good approximation anyway
                    if (m_whichDOF == PsychTest::Shear){ // test shear
                        m_stim1_normF = m_cm_lock->getForce(); 
                        m_stim1_shearF = m_cm_test->getForce();
                        m_stim1_normP = m_cm_lock->getSpoolPosition();
                        m_stim1_shearP = m_cm_test->getSpoolPosition();
                    }else if (m_whichDOF == PsychTest::Normal){ // test normal
                        m_stim1_normF = m_cm_test->getForce();
                        m_stim1_shearF = m_cm_lock->getForce();
                        m_stim1_normP = m_cm_test->getSpoolPosition();
                        m_stim1_shearP = m_cm_lock->getSpoolPosition();
                    }
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }

                // render first stimulus - ramp down
                elapsed = 0;
                while (elapsed < m_userparams.msc_stimulus_time) {
                    double stimVal = Tween::Linear(trial.stimulus1, m_psychtest.m_jnd_stimulus_min, (float)(elapsed / m_userparams.msc_stimulus_time));
                    setStimulus(stimVal);
                    responseWindow(PsychTest::First);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                setStimulus(m_psychtest.m_jnd_stimulus_min);
                // first delay
                elapsed = 0;
                while (elapsed < 0.5) {
                    responseWindow(PsychTest::NA);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // render second stimulus - ramp up
                elapsed = 0;
                while (elapsed < m_userparams.msc_stimulus_time) {
                    double stimVal = Tween::Linear(m_psychtest.m_jnd_stimulus_min, trial.stimulus2, (float)(elapsed / m_userparams.msc_stimulus_time));
                    setStimulus(stimVal);
                    responseWindow(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                // render second stimulus - hold stim and record position and force info
                setStimulus(trial.stimulus2);
                elapsed = 0;
                while (elapsed < m_userparams.msc_stimulus_time) {
                    responseWindow(PsychTest::Second);

                    if (m_whichDOF == PsychTest::Shear){ // test shear
                        m_stim2_normF = m_cm_lock->getForce();
                        m_stim2_shearF = m_cm_test->getForce();
                        m_stim2_normP = m_cm_lock->getSpoolPosition();
                        m_stim2_shearP = m_cm_test->getSpoolPosition();
                    }else if (m_whichDOF == PsychTest::Normal){ // test normal
                        m_stim2_normF = m_cm_test->getForce();
                        m_stim2_shearF = m_cm_lock->getForce();
                        m_stim2_normP = m_cm_test->getSpoolPosition();
                        m_stim2_shearP = m_cm_lock->getSpoolPosition();
                    }
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // render second stimulus - ramp down
                elapsed = 0;
                while (elapsed < m_userparams.msc_stimulus_time) {
                    double stimVal = Tween::Linear(trial.stimulus2, m_psychtest.m_jnd_stimulus_min, (float)(elapsed / m_userparams.msc_stimulus_time));
                    setStimulus(stimVal);
                    responseWindow(PsychTest::Second);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                setStimulus(m_psychtest.m_jnd_stimulus_min);

                // collect response
                while (true) {
                    int answer = -1;
                    ImGui::BeginFixed("##MainWindow", ImGui::GetMainViewport()->Pos,{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
                    ImGui::BeginDisabled(true,1.0f);
                    ImGui::PushStyleColor(ImGuiCol_Button, Blues::DeepSkyBlue);            
                    ImGui::Button("Which Stimulus Was Larger?", ImVec2(-1,0));
                    ImGui::PopStyleColor();
                    ImGui::EndDisabled();
                    if (ImGui::Button("First", ImVec2(240,-1)) || m_xbox.is_button_pressed(XboxController::A))
                        answer = 1;
                    ImGui::SameLine();
                    if (ImGui::Button("Second", ImVec2(240,-1)) || m_xbox.is_button_pressed(XboxController::B))
                        answer = 2;
                    ImGui::End();
                    if (answer != -1) {
                        trial.answer = answer;
                        trial.greater = trial.comparison == trial.answer ? 1 : 0;
                        csv.write_row(m_mode, m_controller, trial.trialnum, trial.generated_num, trial.window, trial.level, trial.stimulus1, trial.stimulus2, trial.standard, trial.comparison, trial.correct, trial.answer, trial.greater, m_stim1_normF, m_stim1_shearF, m_stim1_normP, m_stim1_shearP, m_stim2_normF, m_stim2_shearF, m_stim2_normP, m_stim2_shearP);
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
            // break
            if (w < (m_userparams.n_msc_windows - 1)) {
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
        m_mode = PsychTest::Idle;
        set_window_title("CM Method of Constant Stimuli Study (Subject " + std::to_string(m_subject) + ")");
        if (m_controller == PsychTest::Position){
                set_window_title("CM Method of Constant Stimuli Study (Subject " + std::to_string(m_subject) + ") (Position Control)");
            
        }else if (m_controller == PsychTest::Force){
            set_window_title("CM Method of Constant Stimuli Study (Subject " + std::to_string(m_subject) + ") (Force Control)");
        }
    }

    void savepsychMCSStudy() {
        Timestamp ts;
        std::string filename;
        if (m_controller == PsychTest::Position)
            std::string filename = "/TactilePsychophysics/data/MCS_subject_" + std::to_string(m_subject) + "_positioncontrol_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
        else if (m_controller == PsychTest::Force)
            std::string filename = "/TactilePsychophysics/data/MCS_subject_" + std::to_string(m_subject) + "_forcecontrol_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
        else
            std::string filename = "/TactilePsychophysics/data/MCS_subject_" + std::to_string(m_subject) + "_WHATCONTROL_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";

        Csv csv(filename);
        csv.write_row("Mode","ControlType","Trial","GenNum","Window","Level","Stimulus1","Stimulus2","Standard","Comparison","Correct","Answer","Greater","NormF1", "ShearF1", "NormP1", "ShearP1","NormF2", "ShearF2", "NormP2", "ShearP2");
        for (int w = 0; w < m_userparams.n_msc_windows; ++w) {
            for (auto& trial : m_psychtest.m_stimulus_trials[w])
                csv.write_row(m_mode, m_controller, trial.trialnum, trial.generated_num, trial.window, trial.level, trial.stimulus1, trial.stimulus2, trial.standard, trial.comparison, trial.correct, trial.answer, trial.greater, m_stim1_normF, m_stim1_shearF, m_stim1_normP, m_stim1_shearP, m_stim2_normF, m_stim2_shearF, m_stim2_normP, m_stim2_shearP);
        }
    }

    void setStimulus(double N) {
        double cv = remap(N, m_psychtest.m_userStimulusMin, m_psychtest.m_userStimulusMax, 0.0, 1.0);
        m_cm_test->setControlValue(cv);
        m_cm_test->limits_exceeded();
    }

    void stopExp(){
        m_cm_test->disable();
        m_cm_lock->disable();
        m_hub.stop();

        // ????? Question Add close gui
    }

  
    // Subject
    int m_subject = 0;

    // Experiment
    PsychTest m_psychtest;
    PsychTest::Params m_userparams;
    PsychTest::WhichDof m_whichDOF      = PsychTest::Shear;
    PsychTest::ControlType m_controller = PsychTest::Position;
    PsychTest::Mode m_mode              = PsychTest::Idle;

    double m_stim1_normF;
    double m_stim1_shearF;
    double m_stim1_normP;
    double m_stim1_shearP;

    double m_stim2_normF;
    double m_stim2_shearF;
    double m_stim2_normP;
    double m_stim2_shearP;

    // CM
    CMHub m_hub;
    std::shared_ptr<CM> m_cm_test;
    std::shared_ptr<CM> m_cm_lock;
    CM::Params m_params;

    // User Input
    XboxController m_xbox;
};

int main(int argc, char *argv[])
{
    Options options("capstan2DofPsychophysicalExp.exe","CM Method of Constant Stimuli Study"); 
    options.add_options()
        ("s,subject","Subject ID Number: -s subnum",value<int>())
        ("n,normal","Test the normal direction: -n")
        ("t,tangential","Test the tangential/shear direction: -t")
        ("p,position","Test using position control: -p")
        ("f,force","Test using position control: -f")
        ("h,help","print help");
    
    auto result = options.parse(argc, argv);

    if (result.count("help") > 0)
        print("{}",options.help());
    
    int subject_num;
    if (result.count("s"))
        subject_num = result["s"].as<int>();
    else {
        LOG(Error) << "Missing Subject Number Inputs. Exiting code.";
        print("{}",options.help());
        return 0;
    }

    PsychTest::WhichDof active_dof;
    if (result.count("n") && !result.count("t"))
        active_dof = PsychTest::Normal;
    else if (result.count("t") && !result.count("n"))
        active_dof = PsychTest::Shear;
    else{
        LOG(Error) << "Missing Dof to Test, Or Too Many Inputs. Exiting code.";
        print("{}",options.help());
        return 0;
    }

    PsychTest::ControlType active_control;
    if (result.count("p") && !result.count("f"))
        active_control = PsychTest::Position;
    else if (result.count("f") && !result.count("p"))
        active_control = PsychTest::Force;
    else{
        LOG(Error) << "Missing Controller Type, Or Too Many Inputs. Exiting code.";
        print("{}",options.help());
        return 0;
    }

    PsychMCSGui gui(subject_num, active_dof, active_control);
    gui.run();

    return 0;
}