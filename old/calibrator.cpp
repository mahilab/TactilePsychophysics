#include <Mahi/Robo/Mechatronics/AtiSensor.hpp>
#include <Mahi/Robo/Mechatronics/AIForceSensor.hpp>
#include <Mahi/Util/Logging/Log.hpp>

#include "CMGui.hpp"
#include "Util/PolyFit.hpp"
#include "Vsb.hpp"
#include "Util/Util.hpp"
#include <ETFE.hpp>

using namespace mahi::gui;
using namespace mahi::util;
using namespace mahi::daq;
using namespace mahi::robo;



static GuiLogWritter<TxtFormatter> writer;

void StyleGui();

///////////////////////////////////////////////////////////////////////////////
// INSTRUMENTED TASBI (A CM + Force Calibration Rig)
///////////////////////////////////////////////////////////////////////////////

class InstrumentedCM : public CM {
public:
    struct QueryEx : CM::Query {
        double atiForce;
        bool   useAti;
    };

    /// Constructor
    InstrumentedCM(const std::string& name, Io io, AtiSensor ati, Params params) :
        CM(name, io, params), m_ati(ati) {}

public:
    QueryEx getQueryEx(bool immediate = false) {
        TASBI_LOCK
        if (immediate) {
            QueryEx q;
            fillQuery(q);
            q.atiForce = -m_ati.get_force(Axis::AxisZ);
            q.useAti   = m_use_ati;
            return q;
        }
        return m_qex;
    }

    void zeroAti() {
        TASBI_LOCK
        m_ati.zero();
    }

    void useAti(bool use) {
        TASBI_LOCK
        m_use_ati = use;
    }

    void onUpdate(mahi::util::Time t) override {
        fillQuery(m_qex);
        m_qex.atiForce = -m_ati.get_force(Axis::AxisZ);
        m_qex.useAti   = m_use_ati;
    }

    virtual double getForce(bool filtered = true) override {
        return m_use_ati ? -m_ati.get_force(Axis::AxisZ) : CM::getForce();
    }

private:
    AtiSensor m_ati;
    QueryEx   m_qex;
    bool      m_use_ati = false;
};

struct CMPositionBW : public CMController {
    CMPositionBW(int T, int f1, int f2) : 
        m_T(T),
        m_chirp(hertz(f1), hertz(f2), seconds(T), 0.5, 0.5)   
    { 
        m_t_data.reserve(25000);
        m_i_data.reserve(25000);
        m_o_data.reserve(25000);
    }
    void update(double ctrlValue, Time t, CM& cm) override { 
        if (m_init) {
            m_offset = t;
            m_init = false;
        }
        if (!m_complete) {
            Time t_prime = t - m_offset;
            double cv = m_chirp.evaluate(t_prime);
            double position = cm.scaleCtrlValue(cv, CM::ControlMode::Position);
            cm.controlSpoolPosition(position, t);

            m_t_data.push_back(t_prime.as_seconds());
            m_i_data.push_back(position);
            m_o_data.push_back(cm.getSpoolPosition());

            if (t_prime > seconds(m_T)) {
                dumpData();
                m_complete = true;
            }
        }
        else {
            cm.controlSpoolPosition(cm.scaleCtrlValue(0, CM::ControlMode::Position), t);
        }
    }
    void dumpData() {
        Timestamp ts;
        Csv csv("bw/bw_position.csv");
        csv.write_row("Time","Input","Output");
        for (int i = 0; i < m_t_data.size(); ++i) 
            csv.write_row(m_t_data[i], m_i_data[i], m_o_data[i]);        
    }
private:
    int                 m_T;
    std::vector<double> m_t_data;
    std::vector<double> m_i_data;
    std::vector<double> m_o_data;    
    mahi::util::Chirp   m_chirp;
    bool                m_init = true;
    bool                m_complete = false;
    Time                m_offset;
};

struct CMPositionBWS : public CMController {
    CMPositionBWS(int T, int f1, int f2)  
    { 
        m_t_data.reserve(25000);
        m_i_data.reserve(25000);
        m_o_data.reserve(25000);
        m_schroeder = generateShroeder(f1,f2,1000,T);
    }

    void update(double ctrlValue, Time t, CM& cm) override {
        if (m_i < m_schroeder.size()) {
            double cv = m_schroeder[m_i];
            double position = cm.scaleCtrlValue(cv, CM::ControlMode::Position);
            cm.controlSpoolPosition(position, t);
            m_t_data.push_back(m_i * 0.001);
            m_i_data.push_back(position);
            m_o_data.push_back(cm.getSpoolPosition());
            m_i++;
            if (m_i == m_schroeder.size())
                dumpData();
        }
        else {
            cm.controlSpoolPosition(cm.scaleCtrlValue(0, CM::ControlMode::Position), t);
        }
    }

    void dumpData() {
        Csv csv("bw/bws_position.csv");
        csv.write_row("Time","Input","Output");
        for (int i = 0; i < m_t_data.size(); ++i) 
            csv.write_row(m_t_data[i], m_i_data[i], m_o_data[i]);        
    }

    std::vector<double> m_t_data;
    std::vector<double> m_i_data;
    std::vector<double> m_o_data; 
    std::vector<double> m_schroeder;
    int                 m_i = 0;
};

struct CMForceBW : public CMController {
    CMForceBW(int T, int f1, int f2) : 
        m_T(T),
        m_chirp(hertz(f1), hertz(f2), seconds(T), 0.5, 0.5)   
    { 
        m_t_data.reserve(25000);
        m_i_data.reserve(25000);
        m_o_data.reserve(25000);
    }
    void update(double ctrlValue, Time t, CM& cm) override { 
        if (m_init) {
            m_offset = t;
            m_init = false;
        }
        if (!m_complete) {
            Time t_prime = t - m_offset;
            double cv = m_chirp.evaluate(t_prime);
            double force = cm.scaleCtrlValue(cv, CM::ControlMode::Force);
            cm.controlForce(force, t);

            m_t_data.push_back(t_prime.as_seconds());
            m_i_data.push_back(force);
            m_o_data.push_back(cm.getForce());

            if (t_prime > seconds(m_T)) {
                dumpData();
                m_complete = true;
            }
        }
        else {
            cm.controlForce(cm.scaleCtrlValue(0, CM::ControlMode::Force), t);
        }
    }
    void dumpData() {
        Timestamp ts;
        Csv csv("bw/bw_force.csv");
        csv.write_row("Time","Input","Output");
        for (int i = 0; i < m_t_data.size(); ++i) 
            csv.write_row(m_t_data[i], m_i_data[i], m_o_data[i]);        
    }
private:
    int                 m_T;
    std::vector<double> m_t_data;
    std::vector<double> m_i_data;
    std::vector<double> m_o_data;  
    mahi::util::Chirp   m_chirp;
    bool                m_init = true;
    bool                m_complete = false;
    Time                m_offset;
};

struct CMForceBWS : public CMController {
    CMForceBWS(int T, int f1, int f2)  
    { 
        m_t_data.reserve(25000);
        m_i_data.reserve(25000);
        m_o_data.reserve(25000);
        m_schroeder = generateShroeder(f1,f2,1000,T);
    }

    void update(double ctrlValue, Time t, CM& cm) override {
        if (m_i < m_schroeder.size()) {
            double cv = m_schroeder[m_i];
            double force = cm.scaleCtrlValue(cv, CM::ControlMode::Force);
            cm.controlForce(force, t);
            m_t_data.push_back(m_i * 0.001);
            m_i_data.push_back(force);
            m_o_data.push_back(cm.getForce());
            m_i++;
            if (m_i == m_schroeder.size())
                dumpData();
        }
        else {
            cm.controlForce(cm.scaleCtrlValue(0, CM::ControlMode::Force), t);
        }
    }

    void dumpData() {
        Csv csv("bw/bws_force.csv");
        csv.write_row("Time","Input","Output");
        for (int i = 0; i < m_t_data.size(); ++i) 
            csv.write_row(m_t_data[i], m_i_data[i], m_o_data[i]);        
    }

    std::vector<double> m_t_data;
    std::vector<double> m_i_data;
    std::vector<double> m_o_data; 
    std::vector<double> m_schroeder;
    int                 m_i = 0;
};

///////////////////////////////////////////////////////////////////////////////
// GUI
///////////////////////////////////////////////////////////////////////////////

class CMCalibrator : public Application {
public:

    // DATA BUFFERS
    static constexpr int N_SAMPLES = 1000;

    ScrollingData1D     timeData;
    ScrollingData1D     cmCvData;
    ScrollingData1D     cmPosData;
    ScrollingData1D     cmVelData;
    ScrollingData1D     cmDfDtData;
    ScrollingData1D     cmTorData;
    ScrollingData1D     cmForceData;
    ScrollingData1D     cmForceEstData;
    ScrollingData1D     cmVoltsData;
    ScrollingData1D     nanoForceData;
    ScrollingData1D     vsbForceData;
    ScrollingData1D     vsbPosData;
    ScrollingData1D     vsbVelData;

    ScrollingData2D     fvData;
    ScrollingData2D     fvDataU;
    ScrollingData2D     fvDataD;
    ScrollingData2D     fpData;
    ScrollingData2D     pfData;

    bool                record_data = true;
    bool                record_filtered = false;
    bool                use_vsb = false;

    enum CvMode : int { Joystick = 0, Input = 1, Track = 2 };

    CvMode cvMode = Joystick;
    double cv     = 0;

    CMCalibrator() : 
        Application(), 
        m_vsb("vsb", 2000)
    {
        ImGuiIO& IO = ImGui::GetIO();
        (void)IO;
        IO.ConfigViewportsNoAutoMerge = true;
        StyleGui();

        if (MahiLogger) {
            MahiLogger->add_writer(&writer);
            MahiLogger->set_max_severity(Debug);
        }

        AIForceSensor forcesensor;
        forcesensor.set_channel(&m_hub.daq.AI[7]);
        forcesensor.set_force_calibration(0,1,0);

        AtiSensor nano17;
        nano17.load_calibration("FT06833.cal");
        nano17.set_channels(&m_hub.daq.AI[0], &m_hub.daq.AI[1], &m_hub.daq.AI[2], &m_hub.daq.AI[3],
                            &m_hub.daq.AI[4], &m_hub.daq.AI[5]);

        CM::Io io = {DOHandle(m_hub.daq.DO, 7),           DIHandle(m_hub.daq.DI, 7),
                        AOHandle(m_hub.daq.AO, 7),        EncoderHandle(m_hub.daq.encoder, 7), 
                        forcesensor,                      Axis::AxisX,
                        &m_hub.daq.velocity[7],
                        &m_hub.daq.velocity.velocities[7]};
        
        // set range of force channel

        auto params = CM::Params();
        m_cm     = std::make_shared<InstrumentedCM>("silver", io, nano17, params);
        m_hub.addDevice(0, m_cm);
        m_hub.start();
        set_frame_limit(90_Hz);  // ~oculus rift refresh rate
        m_cm->setControlMode(CM::Torque);
        m_cm->enable();
        m_vsb.setMode(Vsb::Mode::Calibrator);
        m_vsb.setCalibratorParams(15, 0.1, 1.0);
    }

    void showHubBlock(const CMHub::Query& Q) {
        ImGui::BeginDisabled(Q.status == CMHub::Running);
        if (ImGui::Button("Start", ImVec2(-1, 0)))
            m_hub.start();
        ImGui::EndDisabled();
        ImGui::BeginDisabled(Q.status != CMHub::Running);
        if (ImGui::Button("Stop", ImVec2(-1, 0)))
            m_hub.stop();
        static int Fs = 1000;
        if (ImGui::SliderInt("Sample Rate",&Fs,100,2000,"%d Hz"))
            m_hub.setSampleRate(Fs);
        ImGui::EndDisabled();
        ImGui::Separator();
        ImGui::PushItemWidth(150);
        CMGui::ShowHubQuery(Q);
        ImGui::PopItemWidth();
    }

    void showCMBlock(InstrumentedCM::QueryEx& q) {

        auto half_size = ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f -4, 0);
        
        ImGui::BeginDisabled(q.status == (int)CM::Enabled);
        if (ImGui::Button("Enable", ImVec2(-1, 0)))
            m_cm->enable();
        ImGui::EndDisabled();
        ImGui::BeginDisabled(q.status == (int)CM::Disabled);
        if (ImGui::Button("Disable", ImVec2(-1, 0)) || ImGui::IsKeyPressed(GLFW_KEY_SPACE))
            m_cm->disable();
        ImGui::EndDisabled();

        if (ImGui::Button("Zero Position", half_size))
            m_cm->zeroPosition();
        ImGui::SameLine();
        if (ImGui::Button("Zero Force", half_size))
            m_cm->zeroAti();
        // if (ImGui::Button("Set Min/Max Pos", ImVec2(-1, 0)))
        // {
        //     double Min,Max;
        //     ImMinMaxArray( &cmPosData.Data[0], cmPosData.Data.Size, &Min, &Max);
        //     m_cm->setPositionRange(Min,Max);
        // }

        if (ImGui::IsKeyPressed(GLFW_KEY_Z)) {
            m_cm->zeroAti();
            m_cm->zeroPosition();
        }

        if (ImGui::Button("Fit Position",half_size))
            start_coroutine(fitPosition());
        ImGui::SameLine();
        if (ImGui::Button("Fit Force",half_size))
            start_coroutine(fitForce());        
        

        ImGui::PushItemWidth(200);
        if (ImGui::BeginTabBar("CM Tabs")) {
            if (ImGui::BeginTabItem("Parameters")) {
                auto cfg = m_cm->getParams();
                if (ImGui::Button("Export", half_size)) {
                    auto sd = [this]() {
                        std::string path;
                        if (save_dialog(path, {{"JSON", "json"}}) == DialogResult::DialogOkay)
                            m_cm->exportParams(path);
                    };
                    std::thread thrd(sd);
                    thrd.detach();
                }
                ImGui::SameLine();
                if (ImGui::Button("Import", half_size)) {
                    auto sd = [this]() {
                        std::string path;
                        if (open_dialog(path, {{"JSON", "json"}}) == DialogResult::DialogOkay)
                            m_cm->importParams(path);
                    };
                    std::thread thrd(sd);
                    thrd.detach();
                }
                ImGui::Separator();
                if (CMGui::ShowParamsMenu(cfg))
                    m_cm->setParams(cfg);

                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Querey")) {
                if (ImGui::Button("Save Querey History", ImVec2(-1,0))) {
                    auto sd = [this]() {
                        std::string path;
                        if (save_dialog(path, {{"CSV", "csv"}}) == DialogResult::DialogOkay)
                            m_cm->dumpQueries(path);
                    };
                    std::thread thrd(sd);
                    thrd.detach();
                }
                ImGui::Separator();
                CMGui::ShowCMQuerey(q);
                ImGui::Separator();
                ImGui::LabelText("ATI Force", "%.3f", q.atiForce);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::PopItemWidth();
    }

    void showLogBlock() {

        // log window
        static ImGuiTextFilter                     filter;
        static std::unordered_map<Severity, Color> colors = {
            {None, Grays::Gray50},      {Fatal, Reds::Red}, {Error, Pinks::HotPink},
            {Warning, Yellows::Yellow}, {Info, Whites::White},  {Verbose, Greens::Chartreuse},
            {Debug, Cyans::Cyan}};

        if (ImGui::Button("Clear"))
            writer.logs.clear();
        ImGui::SameLine();
        filter.Draw("Filter", -50);
        ImGui::BeginChild("scrolling", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
        for (int i = 0; i < writer.logs.size(); ++i) {
            if (filter.PassFilter(writer.logs[i].second.c_str())) {
                ImGui::PushStyleColor(ImGuiCol_Text, colors[writer.logs[i].first]);
                ImGui::TextUnformatted(writer.logs[i].second.c_str());
                ImGui::PopStyleColor();
            }
        }
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }

    void showControlBlock(CMHub::Query& Q, InstrumentedCM::QueryEx& q) {
        auto half_size = ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f -4, 50);

        ImGui::BeginChild("LS", half_size);
        ImGui::Text("Control Mode");
        if (ImGui::RadioButton("Torque", q.ctrlMode == (int)CM::Torque))
            m_cm->setControlMode(CM::Torque);
        ImGui::SameLine();
        if (ImGui::RadioButton("Position", q.ctrlMode == (int)CM::Position))
            m_cm->setControlMode(CM::Position);
        ImGui::SameLine();
        if (ImGui::RadioButton("Force", q.ctrlMode == (int)CM::Force))
            m_cm->setControlMode(CM::Force);
        ImGui::SameLine();
        if (ImGui::RadioButton("Force2", q.ctrlMode == (int)CM::Force2))
            m_cm->setControlMode(CM::Force2);
        ImGui::SameLine();
        if (ImGui::RadioButton("Custom", q.ctrlMode == (int)CM::Custom))
            m_cm->setControlMode(CM::Custom); 
        ImGui::SameLine();
        if (ImGui::Checkbox("Use ATI", &q.useAti)) 
            m_cm->useAti(q.useAti);
        ImGui::EndChild();
        
        ImGui::SameLine();
        ImGui::BeginChild("RS", ImVec2(-1,50));
        ImGui::Text("Control Value");

        int temp = cvMode;
        ImGui::ModeSelector(&temp, {"Joystick", "Input", "Trajectory"});
        cvMode = (CvMode)temp;
        // control value
        ImGui::PushItemWidth(200);
        if (cvMode == Joystick)
            cv = getJoystickTriggerAxis();
        else if (cvMode == Input) {
            ImGui::SameLine();
            ImGui::DragDouble("Control Value", &cv, 0.01f, -1, 1);
        } else if (cvMode == Track) {
            static int    trackMode   = 0;
            static double trackPeriod = 1;
            static bool   modulate = false;
            ImGui::SameLine();
            ImGui::DragDouble("Period", &trackPeriod, 0.1f, 0, 10);
            ImGui::SameLine();
            ImGui::ModeSelector(&trackMode, {"Sine", "Square"});
            ImGui::SameLine();
            ImGui::Checkbox("Modulate", &modulate);
            cv = std::sin(2 * PI * (1.0 / trackPeriod) * Q.time) * (modulate ? std::sin(2 * PI * (4.0 / trackPeriod) * Q.time) : 1);
            if (trackMode == 1)
                cv = cv > 0 ? 1 : -1;
            cv = 0.5 + 0.5 * cv;
        }
        ImGui::PopItemWidth();
        ImGui::EndChild();

        // auto cv_rmse = q.ctrlMode == (int)CM::Position ? rmse(cmCvData.Data.Data, cmPosData.Data.Data, cmCvData.Data.Size) :
        //                q.ctrlMode == (int)CM::Force || q.ctrlMode == (int)CM::Force2 ? rmse(cmCvData.Data.Data, cmForceData.Data.Data, cmCvData.Data.Size) : 0;

        // ImGui::SameLine();
        // ImGui::Value("    RMSE",(float)cv_rmse);

        if (m_cv_gui)
            m_cm->setControlValue(cv);
    }

    void showPlotsBlock(CMHub::Query& Q, InstrumentedCM::QueryEx& q) {

        ImGui::Checkbox("Record Data", &record_data);
        ImGui::SameLine();
        ImGui::Checkbox("Use Filtered Data", &record_filtered);
        ImGui::SameLine();        
        if (ImGui::Button("Export Data")) 
            exportData();

        auto half_size = ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f -4, 400);

        ImGui::BeginChild("PLS", half_size);
        if (ImGui::BeginTabBar("TimeDomainPlots")) {
            if (ImGui::BeginTabItem("All")) {
                ImPlot::SetNextPlotLimitsX(Q.time - 10, Q.time, ImGuiCond_Always);
                ImPlot::SetNextPlotLimitsY(0, 10);
                ImPlot::SetNextPlotLimitsY(0, 100, ImGuiCond_Once, 1); 
                if (ImPlot::BeginPlot("##AllPlots", "Time [s]", "Force [N]", ImVec2(-1,-1),ImPlotFlags_YAxis2,0,0,0,0,"Position [deg]")) {
                    if (q.ctrlMode == (int)CM::Force || q.ctrlMode == (int)CM::Force2) {
                        ImPlot::SetNextLineStyle(Pinks::Pink);
                        ImPlot::PlotLine("Control Value", &timeData.Data[0], &cmCvData.Data[0], timeData.Data.size(), timeData.Offset);
                    }
                    ImPlot::SetNextLineStyle(Yellows::Yellow);
                    ImPlot::PlotLine("CM Force", &timeData.Data[0], &cmForceData.Data[0], timeData.Data.size(), timeData.Offset);
     
                    ImPlot::SetPlotYAxis(1);
                    if (q.ctrlMode == (int)CM::Position) {
                        ImPlot::SetNextLineStyle(Pinks::Pink);
                        ImPlot::PlotLine("Control Value", &timeData.Data[0], &cmCvData.Data[0], timeData.Data.size(), timeData.Offset);
                    }
                    ImPlot::SetNextLineStyle(Oranges::Orange);
                    ImPlot::PlotLine("Spool Position", &timeData.Data[0], &cmPosData.Data[0], timeData.Data.size(), timeData.Offset);

                    // ImPlot::SetNextLineStyle(Yellows::Gold);
                    // ImPlot::PlotLine("CM Volts", &timeData.Data[0], &cmVoltsData.Data[0], timeData.Data.size(), timeData.Offset);

                    // ImPlot::HideNextItem(true);
                    // ImPlot::SetPlotYAxis(2);
                    // if (q.ctrlMode == (int)CM::Torque) {
                    //     ImPlot::SetNextLineStyle(Pinks::Pink);
                    //     ImPlot::PlotLine("Control Value (Y3)", &timeData.Data[0], &cmCvData.Data[0], timeData.Data.size(), timeData.Offset);                    // }
                    // ImPlot::HideNextItem(true);
                    // ImPlot::SetNextLineStyle(Purples::Violet);
                    // ImPlot::PlotLine("Velocity", &timeData.Data[0], &cmVelData.Data[0], timeData.Data.size(), timeData.Offset);
                    // ImPlot::HideNextItem(true);
                    // ImPlot::SetNextLineStyle(Purples::Purple);
                    // ImPlot::PlotLine("dFdt", &timeData.Data[0], &cmDfDtData.Data[0], timeData.Data.size(), timeData.Offset);

                    // ImPlot::SetNextLineStyle(Purples::Violet);
                    // ImPlot::PlotLine("CM Torque", &timeData.Data[0], &cmTorData.Data[0], timeData.Data.size(), timeData.Offset);
                    ImPlot::EndPlot();
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Force")) {
                ImPlot::SetNextPlotLimitsX(Q.time - 10, Q.time, ImGuiCond_Always);
                ImPlot::SetNextPlotLimitsY(0, 10);
                if (ImPlot::BeginPlot("##ForcePlots", "Time [s]", "Force [N]", ImVec2(-1,-1))) {
                    if (q.ctrlMode == (int)CM::Force || q.ctrlMode == (int)CM::Force2) {
                        ImPlot::SetNextLineStyle(Pinks::Pink);
                        ImPlot::PlotLine("Control Value", &timeData.Data[0], &cmCvData.Data[0], timeData.Data.size(), timeData.Offset);
                    }
                    ImPlot::SetNextLineStyle(Reds::Red);
                    ImPlot::PlotLine("VSB Force", &timeData.Data[0], &vsbForceData.Data[0], timeData.Data.size(), timeData.Offset);
                    ImPlot::SetNextLineStyle(Blues::DeepSkyBlue);
                    ImPlot::PlotLine("Nano17 Force", &timeData.Data[0], &nanoForceData.Data[0], timeData.Data.size(), timeData.Offset);
                    ImPlot::SetNextLineStyle(Yellows::Yellow);
                    ImPlot::PlotLine("CM Force (Act.)", &timeData.Data[0], &cmForceData.Data[0], timeData.Data.size(), timeData.Offset);
                    ImPlot::SetNextLineStyle(Greens::Chartreuse);
                    ImPlot::PlotLine("CM Force (Est.)", &timeData.Data[0], &cmForceEstData.Data[0], timeData.Data.size(), timeData.Offset);
                    ImPlot::EndPlot();
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Position")) {
                ImPlot::SetNextPlotLimitsX(Q.time - 10, Q.time, ImGuiCond_Always);
                ImPlot::SetNextPlotLimitsY(0, 100);
                if (ImPlot::BeginPlot("##PosPlots", "Time [s]", "Position [degs]", ImVec2(-1,-1))) {
                    if (q.ctrlMode == (int)CM::Position) {
                        ImPlot::SetNextLineStyle(Pinks::Pink);
                        ImPlot::PlotLine("Control Value", &timeData.Data[0], &cmCvData.Data[0], timeData.Data.size(), timeData.Offset);
                    }
                    ImPlot::SetNextLineStyle(Oranges::Orange);
                    ImPlot::PlotLine("Spool Position", &timeData.Data[0], &cmPosData.Data[0], timeData.Data.size(), timeData.Offset);
                    ImPlot::EndPlot();
                }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("PRS",ImVec2(-1,400));
        if (ImGui::BeginTabBar("FreqDomain")) {
            static std::vector<double> x(10000);
            static std::vector<double> y(10000);
            static etfe::ETFE myetfe(10000,1000);
            m_cm->getControllerIo(x,y);
            auto& result = myetfe.estimate(x.data(), y.data());
            if (ImGui::BeginTabItem("Magnitude")) {
                ImPlot::SetNextPlotLimits(1,500,-100,10);
                if (ImPlot::BeginPlot("##Bode1","Frequency [Hz]","Magnitude [dB]",ImVec2(-1,-1))) {
                    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.250f);
                    ImPlot::PlotShaded("##Mag1",result.f.data(),result.mag.data(),(int)result.f.size(),-INFINITY);
                    ImPlot::PlotLine("##Mag2",result.f.data(),result.mag.data(),(int)result.f.size());
                    ImPlot::EndPlot();
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Amplitude")) {
                ImPlot::SetNextPlotLimits(0,500,0,0.5);
                if (ImPlot::BeginPlot("##Amp","Frequency [Hz]","Amplitude", ImVec2(-1,-1))) {
                    ImPlot::SetLegendLocation(ImPlotLocation_NorthEast);
                    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.25f);
                    ImPlot::PlotShaded("x(f)",result.f.data(),result.ampx.data(),(int)result.f.size(),-INFINITY);
                    ImPlot::PlotLine("x(f)",result.f.data(),result.ampx.data(),(int)result.f.size());
                    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.25f);
                    ImPlot::PlotShaded("y(f)",result.f.data(),result.ampy.data(),(int)result.f.size(),-INFINITY);
                    ImPlot::PlotLine("y(f)",result.f.data(),result.ampy.data(),(int)result.f.size());
                    ImPlot::EndPlot();
                }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        ImGui::EndChild();
    }

    void showCalibrationBlock() {
        static bool  use_vsb      = false;

        static std::vector<ImPlotPoint> fvFit(100), fvFitU(100), fvFitD(100), fpFit(100), pfFit(100);
        static std::vector<double> coeffU(3), coeffD(3), coeff(3), coeffFP(3), coeffPF(3);
        fitSamples(fvData, fvFit, coeff, 2);
        fitSamples(fvDataU, fvFitU, coeffU, 2);
        fitSamples(fvDataD, fvFitD, coeffD, 2);
        fitSamples(fpData, fpFit, coeffFP, 2);
        fitSamples(pfData, pfFit, coeffPF, 2);


        ImGui::Checkbox("Use VSB", &use_vsb);  
        ImGui::Separator();        

        auto half_size = ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f -4, 250);

        ImPlot::SetNextPlotLimits(0.5f, 2, 0, 10);
        if (ImPlot::BeginPlot("##FV", "CM Force Sensor [V]", "Applied [N]", half_size, ImPlotFlags_NoLegend)) {
            ImPlot::PushStyleColor(ImPlotCol_MarkerFill, Greens::Chartreuse);
            ImPlot::PushStyleColor(ImPlotCol_MarkerOutline, Greens::Chartreuse);
            ImPlot::PushStyleColor(ImPlotCol_Line, Greens::Chartreuse);
            ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 1.5f);
            ImPlot::PlotScatter("Data Up [N/V]", &fvDataU.Data[0].x, &fvDataU.Data[0].y, fvDataU.Data.size(), fvDataU.Offset, 2 * sizeof(double));
            ImPlot::PushStyleColor(ImPlotCol_Line, Greens::LimeGreen);
            ImPlot::PlotLine("Fit Up [N/V]", &fvFitU[0].x, &fvFitU[0].y, (int)fvFitU.size(), 0,2 * sizeof(double));
            ImPlot::PopStyleColor(4);
            ImPlot::PopStyleVar();
            ImPlot::PushStyleColor(ImPlotCol_MarkerFill, Oranges::Orange);
            ImPlot::PushStyleColor(ImPlotCol_MarkerOutline, Oranges::Orange);
            ImPlot::PushStyleColor(ImPlotCol_Line, Oranges::Orange);
            ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 1.5f);
            ImPlot::PlotScatter("Data Down [N/V]", &fvDataD.Data[0].x, &fvDataD.Data[0].y, fvDataD.Data.size(), fvDataD.Offset, 2 * sizeof(double));
            ImPlot::PushStyleColor(ImPlotCol_Line, Oranges::DarkOrange);
            ImPlot::PlotLine("Fit Down [N/V]", &fvFitD[0].x, &fvFitD[0].y, (int)fvFitD.size(), 0, 2 * sizeof(double));
            ImPlot::PopStyleColor(4);
            ImPlot::PopStyleVar();
            ImPlot::SetNextLineStyle(Blues::DeepSkyBlue);
            ImPlot::PlotLine("Fit Combined [N/V]", &fvFit[0].x, &fvFit[0].y, (int)fvFit.size(), 0, 2 * sizeof(double));
            ImPlot::EndPlot();
        }

        ImGui::SameLine();
        ImPlot::SetNextPlotLimits(0, 10, 0, 100);
        if (ImPlot::BeginPlot("##FP", "CM Force Sensor [N]", "CM Spool Position [deg]", half_size, ImPlotFlags_NoLegend)) {
            ImPlot::PushStyleColor(ImPlotCol_MarkerFill, Blues::DeepSkyBlue);
            ImPlot::PushStyleColor(ImPlotCol_MarkerOutline, Blues::DeepSkyBlue);
            ImPlot::PushStyleColor(ImPlotCol_Line, Blues::DeepSkyBlue);
            ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 1.0f);
            ImPlot::PlotScatter("Data [deg/N]", &fpData.Data[0].x, &fpData.Data[0].y, fpData.Data.size(), fpData.Offset, 2 * sizeof(double));
            ImPlot::PushStyleColor(ImPlotCol_Line, Blues::SkyBlue);
            ImPlot::PlotLine("Fit [deg/N]", &fpFit[0].x, &fpFit[0].y, (int)fpFit.size(), 0, 2 * sizeof(double));
            ImPlot::PopStyleColor(4);
            ImPlot::PopStyleVar();
            ImPlot::EndPlot();
        }

        half_size = ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f -4, 0);
        if (ImGui::Button("Apply Volts-Force Fit", half_size))
            m_cm->setForceCalibration(coeff[0], coeff[1], coeff[2]);
        ImGui::SameLine();
        if (ImGui::Button("Apply Force-Position Fit", half_size))
            m_cm->setPositionForceCalibration(coeffPF[0], coeffPF[1], coeffPF[2], coeffFP[0], coeffFP[1], coeffFP[2]);
    }

    void showBenchmarkBlock() {
        static int bw_t  = 10;
        static int bw_f1 = 0;
        static int bw_f2 = 15;
        ImGui::SliderInt("Bandwidth Time",&bw_t,0,20);
        ImGui::SliderInt("Bandwidth F1",&bw_f1,0,20);
        ImGui::SliderInt("Bandwidth F2",&bw_f2,0,20);
        if (ImGui::Button("Bench Position")) {
            auto c = std::make_shared<CMPositionBW>(bw_t, bw_f1, bw_f2);
            m_cm->setCustomController(c);
            m_cm->setControlMode(CM::Custom);
        }
        ImGui::SameLine();
        if (ImGui::Button("Bench Position (S)")) {
            auto c = std::make_shared<CMPositionBWS>(bw_t, bw_f1, bw_f2);
            m_cm->setCustomController(c);
            m_cm->setControlMode(CM::Custom);
        }
        ImGui::SameLine();
        if (ImGui::Button("Bench Force")) {
            auto c = std::make_shared<CMForceBW>(bw_t, bw_f1, bw_f2);
            m_cm->setCustomController(c);
            m_cm->setControlMode(CM::Custom);
        }
        ImGui::SameLine();
        if (ImGui::Button("Bench Force (S)")) {
            auto c = std::make_shared<CMForceBWS>(bw_t, bw_f1, bw_f2);
            m_cm->setCustomController(c);
            m_cm->setControlMode(CM::Custom);
        }
    }

    void showFilterBlock(InstrumentedCM::QueryEx& q) {
        auto half_size = ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f -4, 25);

        static bool paused = false;

        ImGui::BeginChild("FilterLS", half_size);
        if (ImGui::RadioButton("None", q.filtMode == (int)CM::None))
            m_cm->setForceFilterMode(CM::None);
        ImGui::SameLine();
        if (ImGui::RadioButton("Lowpass", q.filtMode == (int)CM::Lowpass))
            m_cm->setForceFilterMode(CM::Lowpass);
        ImGui::SameLine();
        if (ImGui::RadioButton("Median", q.filtMode == (int)CM::Median))
            m_cm->setForceFilterMode(CM::Median);
        ImGui::SameLine();
        if (ImGui::RadioButton("Cascade", q.filtMode == (int)CM::Cascade))
            m_cm->setForceFilterMode(CM::Cascade);
        ImGui::SameLine();
        ImGui::Checkbox("Pause",&paused);
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("FilterRS", half_size);

        static std::vector<double> x(10000);
        static std::vector<double> y(10000);

        static bool etfe_need_update = false;
        static int window         = 1;
        static int inwindow        = 3;
        static int nwindow_opts[] = {100, 200, 500, 1000, 2000, 5000, 10000};
        static int infft          = 3;
        static int nfft_opts[]    = {100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000};
        static float overlap      = 0.5f;

        ImGui::PushItemWidth(100);
        if (ImGui::Combo("NFFT", &infft, "100\0""200\0""500\0""1000\0""2000\0""5000\0""10000\0""20000\0""50000\0")) {
            inwindow = infft < inwindow ? infft : inwindow;
            etfe_need_update = true;
        }
        ImGui::SameLine();
        if (ImGui::Combo("Type", &window,"hamming\0hann\0winrect\0"))                                           
            etfe_need_update = true;
        ImGui::SameLine();
        if (ImGui::Combo("Size", &inwindow,"100\0""200\0""500\0""1000\0""2000\0""5000\0""10000\0"))               
            etfe_need_update = true;
        ImGui::SameLine();
        if (ImGui::SliderFloat("Overlap",&overlap,0,1,"%.2f"))                                                                  
            etfe_need_update = true;
        ImGui::PopItemWidth();
        ImGui::EndChild();
        ImGui::Separator();

        static etfe::ETFE myetfe(10000,1000, etfe::hamming(nwindow_opts[inwindow]), nwindow_opts[inwindow]/2, nfft_opts[infft]);        

        if (etfe_need_update) {
            infft = inwindow > infft ? inwindow : infft;
            int nwindow  = nwindow_opts[inwindow];
            int noverlap = (int)(nwindow * overlap);
            int nfft     = nfft_opts[infft];
            myetfe.setup(10000,1000,window == 0 ? etfe::hamming(nwindow) : window == 1 ? etfe::hann(nwindow) : etfe::winrect(nwindow), noverlap, nfft);
            etfe_need_update = false;
        }


        if (!paused)
            m_cm->getFilterIo(x,y);
        auto& result = myetfe.estimate(x.data(), y.data());    
        
        half_size = ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f -4, 400);

        ImGui::BeginChild("FilterPlotLS", half_size);
        ImPlot::SetNextPlotLimits(0, 10, 0, 10);
        if (ImPlot::BeginPlot("##FilterTimeDomain","Time [s]","Volts [V]",ImVec2(-1,-1))) {
            ImPlot::PlotLine("Raw",&x[0],10000,0.001);
            ImPlot::PlotLine("Filtered",&y[0],10000,0.001);
            ImPlot::EndPlot();
        }
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("FilterPlotRS", half_size);

        if (ImGui::BeginTabBar("ForceFilterFreq")) {
            if (ImGui::BeginTabItem("Magnitude")) {
                ImPlot::SetNextPlotLimits(1,500,-100,10);
                if (ImPlot::BeginPlot("##Bode1","Frequency [Hz]","Magnitude [dB]",ImVec2(-1,-1))) {
                    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.250f);
                    ImPlot::PlotShaded("##Mag1",result.f.data(),result.mag.data(),(int)result.f.size(),-INFINITY);
                    ImPlot::PlotLine("##Mag2",result.f.data(),result.mag.data(),(int)result.f.size());
                    ImPlot::EndPlot();
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Power")) {
                ImPlot::SetNextPlotLimits(0,500,0,0.5);
                if (ImPlot::BeginPlot("##Amp","Frequency [Hz]","Power Spectral Density [db/Hz]", ImVec2(-1,-1))) {
                    ImPlot::SetLegendLocation(ImPlotLocation_NorthEast);
                    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.25f);
                    ImPlot::PlotShaded("x(f)",result.f.data(),result.pxx.data(),(int)result.f.size(),-INFINITY);
                    ImPlot::PlotLine("x(f)",result.f.data(),result.pxx.data(),(int)result.f.size());
                    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.25f);
                    ImPlot::PlotShaded("y(f)",result.f.data(),result.pyy.data(),(int)result.f.size(),-INFINITY);
                    ImPlot::PlotLine("y(f)",result.f.data(),result.pyy.data(),(int)result.f.size());
                    ImPlot::EndPlot();
                }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::EndChild();
    }

    void updateDataBuffers(CMHub::Query& Q, InstrumentedCM::QueryEx& q) {
        auto   vsbq       = m_vsb.query();
        auto   vsbp       = m_vsb.getParams();
        double vsb_weight = vsbp.mass * mahi::util::G;
        double vsb_force  = vsbq.linearForceSense + vsb_weight;
        static float lastForceRaw = 0;
        if (record_data) {
            double forceRaw = record_filtered ? q.forceRawFiltered : q.forceRaw;
            double forceCon = record_filtered ? q.forceFiltered    : q.force;
            timeData.AddPoint(Q.time);
            cmCvData.AddPoint(q.ctrlValueScaled);
            cmPosData.AddPoint(q.spoolPosition);
            cmVelData.AddPoint(q.spoolVelocity);
            cmDfDtData.AddPoint(q.dFdt);
            cmTorData.AddPoint(q.motorTorqueCommand);
            nanoForceData.AddPoint(q.atiForce);
            cmForceData.AddPoint(forceCon);
            cmForceEstData.AddPoint(q.forceEst);
            cmVoltsData.AddPoint(forceRaw);
            vsbForceData.AddPoint(vsb_force);
            vsbPosData.AddPoint(vsbq.linearPosition);
            vsbVelData.AddPoint(vsbq.linearVelocity);
            fvData.AddPoint(forceRaw, use_vsb ? vsb_force : q.atiForce);
            if (forceRaw > lastForceRaw)
                fvDataU.AddPoint(forceRaw, use_vsb ? vsb_force : q.atiForce);
            else
                fvDataD.AddPoint(forceRaw, use_vsb ? vsb_force : q.atiForce);
            fpData.AddPoint(forceCon, q.spoolPosition);
            pfData.AddPoint(q.spoolPosition, forceCon);
            lastForceRaw = forceRaw;
        }
    }

    void exportData() {
        record_data = false;
        auto sd = [this]() {
            std::string path;
            if (save_dialog(path, {{"CSV", "csv"}}) == DialogResult::DialogOkay) {
                Csv csv(path);
                if (csv.is_open()) {
                    csv.write_row("Time [s]", 
                                    "CM Control Value",                                     
                                    "CM Force [N]",
                                    "CM FSC Voltage [V]",
                                    "CM Spool Position [deg]",
                                    "CM Spool Velocity [deg/s]",
                                    "CM Motor Torque [N/m]",
                                    "VSB Force [N]",
                                    "VSB Position [m]",
                                    "VSB Velocity [m/s]",
                                    "Nano 17 Force [N]");
                    int i = timeData.Offset;
                    int N = timeData.Data.Size;
                    for (int n = 0; n < N; ++n) {
                        csv.write_row(timeData.Data[i],
                                        cmCvData.Data[i],
                                        cmForceData.Data[i],
                                        cmVoltsData.Data[i],
                                        cmPosData.Data[i],
                                        cmVelData.Data[i],
                                        cmTorData.Data[i],
                                        vsbForceData.Data[i],
                                        vsbPosData.Data[i],
                                        vsbVelData.Data[i],
                                        nanoForceData.Data[i]); 
                        if (++i == timeData.Data.size())
                            i = 0;
                    }
                    csv.close();
                }
            }
            record_data = true;
        };
        std::thread thrd(sd);
        thrd.detach();
    }

    void update() override {

        // Get queries
        auto Q = m_hub.getQuery(true);
        auto q = m_cm->getQueryEx(true);

        // update buffers
        updateDataBuffers(Q,q);      
 
        // layout GUI
        static bool window_open = true;
        ImGui::Begin("CM GUI", &window_open, ImGuiWindowFlags_NoCollapse);
        ImGui::BeginChild("LeftPanel",ImVec2(400,-1));
        if (ImGui::CollapsingHeader("Hub", ImGuiTreeNodeFlags_DefaultOpen))
            showHubBlock(Q);
        if (ImGui::CollapsingHeader("CM", ImGuiTreeNodeFlags_DefaultOpen)) 
            showCMBlock(q);   
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("RightPanel");
        if (ImGui::CollapsingHeader("Controller", ImGuiTreeNodeFlags_DefaultOpen)) {
            showControlBlock(Q, q);
            ImGui::Separator();
            showPlotsBlock(Q,q);
        }        
        if (ImGui::CollapsingHeader("Sensor Calibration")) 
            showCalibrationBlock();              
        if (ImGui::CollapsingHeader("Force Filtering")) 
            showFilterBlock(q);
       if (ImGui::CollapsingHeader("Controller Benchmarking")) 
            showBenchmarkBlock();        
        if (ImGui::CollapsingHeader("Logs")) 
            showLogBlock();        
        ImGui::EndChild();
        ImGui::End(); 

        if (!window_open)
            quit();
    }

    float getJoystickTriggerAxis() {
        GLFWgamepadstate state;
        if (glfwGetGamepadState(GLFW_JOYSTICK_1, &state)) {
            float left  = state.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER];
            float right = state.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER];
            float triggers = right - left;
            return triggers + 0.15f * state.buttons[GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER];
        }
        return 0;
    }

    bool fitSamples(ScrollingData2D& samples, std::vector<ImPlotPoint>& fit, std::vector<double>& coeff,int order) {
        if (samples.Data.size() != N_SAMPLES)
            return false;
        static std::vector<double> xv(N_SAMPLES);
        static std::vector<double> yv(N_SAMPLES);
        double                     xMin = INF, xMax = -INF;
        for (int i = 0; i < samples.Data.size(); ++i) {
            xv[i] = samples.Data[i].x;
            if (xv[i] > xMax)
                xMax = xv[i];
            if (xv[i] < xMin)
                xMin = xv[i];
            yv[i] = samples.Data[i].y;
        }
        polyFit(xv, yv, coeff, order);
        for (int i = 0; i < 100; ++i) {
            double x = xMin + (xMax - xMin) * 0.01 * i;
            double y = polyEval(x, coeff);
            fit[i].x = (float)x;
            fit[i].y = (float)y;
        }
        return true;
    }

    bool m_cv_gui = true;

    Enumerator fitPosition() {    
        m_cv_gui = false;
        double max_pos;
        m_cm->setControlMode(CM::Torque);
        m_cm->setControlValue(-1);
        co_yield yield_time(100_ms);
        m_cm->setControlValue(0.15);
        co_yield yield_time(2_s);
        m_cm->zeroPosition();
        m_cm->setControlValue(1.0);
        co_yield yield_time(2_s);
        max_pos = m_cm->getQuery().spoolPosition;
        m_cm->setPositionRange(0, max_pos);
        m_cm->setControlMode(CM::Position);
        m_cv_gui = true;
    }

    Enumerator fitForce() {
        m_cv_gui = false;
        std::vector<double> torques = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
        std::vector<double> forces  = {1.168, 3.638, 5.304, 6.742, 8.088, 9.294, 10.314, 11.362, 12.313};
        std::vector<double> volts   = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        m_cm->setControlMode(CM::Torque);

        for (int i = 0; i < torques.size(); ++i) {
            m_cm->setControlValue(-1);
            co_yield yield_time(100_ms);
            m_cm->setControlValue(torques[i]);
            co_yield yield_time(1_s);
            auto q = m_cm->getQueryEx();
            auto f = q.atiForce;
            auto v = q.forceRaw;
            // forces[i] = f;
            volts[i] = v;
            LOG(Info) << torques[i] << " = " << f << " N = " << v << " V";
        }
        std::vector<double> coeff(3);
        polyFit(volts,forces, coeff, 2);
        m_cm->setForceCalibration(coeff[0], coeff[1], coeff[2]);
        m_cv_gui = true;
    }

    CMHub                           m_hub;
    std::shared_ptr<InstrumentedCM> m_cm;
    Vsb                                m_vsb;
};

///////////////////////////////////////////////////////////////////////////////
// MAIN
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char const* argv[]) {
    CMCalibrator app;
    app.run();
    LOG(Info) << "Program Ended";
    return 0;
}

void StyleGui() {
    
    ImVec4 dark_accent  = ImVec4(0.85f, 0.37f, 0.00f, 1.00f);
    ImVec4 light_accent = ImVec4(1.00f, 0.63f, 0.00f, 1.00f);

    auto& style = ImGui::GetStyle();
    style.WindowPadding = {6,6};
    style.FramePadding  = {6,3};
    style.CellPadding   = {6,3};
    style.ItemSpacing   = {6,6};
    style.ItemInnerSpacing = {6,6};
    style.ScrollbarSize = 16;
    style.GrabMinSize = 8;
    style.WindowBorderSize = style.ChildBorderSize = style.PopupBorderSize = style.TabBorderSize = 0;
    style.FrameBorderSize = 1;
    style.WindowRounding = style.ChildRounding = style.PopupRounding = style.ScrollbarRounding = style.GrabRounding = style.TabRounding = 4;


    ImVec4* colors = ImGui::GetStyle().Colors;
    colors[ImGuiCol_Text]                   = ImVec4(0.89f, 0.89f, 0.92f, 1.00f);
    colors[ImGuiCol_TextDisabled]           = ImVec4(0.38f, 0.45f, 0.64f, 1.00f);
    colors[ImGuiCol_WindowBg]               = ImVec4(0.20f, 0.21f, 0.27f, 1.00f);
    colors[ImGuiCol_ChildBg]                = ImVec4(0.20f, 0.21f, 0.27f, 0.00f);
    colors[ImGuiCol_PopupBg]                = ImVec4(0.20f, 0.21f, 0.27f, 1.00f);
    colors[ImGuiCol_Border]                 = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_BorderShadow]           = ImVec4(0.00f, 0.00f, 0.00f, 0.06f);
    colors[ImGuiCol_FrameBg]                = ImVec4(1.00f, 1.00f, 1.00f, 0.02f);
    colors[ImGuiCol_FrameBgHovered]         = light_accent;
    colors[ImGuiCol_FrameBgActive]          = light_accent;
    colors[ImGuiCol_TitleBg]                = dark_accent;
    colors[ImGuiCol_TitleBgActive]          = dark_accent;
    colors[ImGuiCol_TitleBgCollapsed]       = dark_accent;
    colors[ImGuiCol_MenuBarBg]              = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]            = ImVec4(0.20f, 0.21f, 0.27f, 1.00f);
    colors[ImGuiCol_ScrollbarGrab]          = ImVec4(0.89f, 0.89f, 0.93f, 0.27f);
    colors[ImGuiCol_ScrollbarGrabHovered]   = ImVec4(0.89f, 0.89f, 0.93f, 0.55f);
    colors[ImGuiCol_ScrollbarGrabActive]    = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
    colors[ImGuiCol_CheckMark]              = dark_accent;
    colors[ImGuiCol_SliderGrab]             = dark_accent;
    colors[ImGuiCol_SliderGrabActive]       = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
    colors[ImGuiCol_Button]                 = dark_accent;
    colors[ImGuiCol_ButtonHovered]          = light_accent;
    colors[ImGuiCol_ButtonActive]           = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    colors[ImGuiCol_Header]                 = dark_accent;
    colors[ImGuiCol_HeaderHovered]          = light_accent;
    colors[ImGuiCol_HeaderActive]           = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    colors[ImGuiCol_Separator]              = dark_accent;
    colors[ImGuiCol_SeparatorHovered]       = light_accent;
    colors[ImGuiCol_SeparatorActive]        = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    colors[ImGuiCol_ResizeGrip]             = dark_accent;
    colors[ImGuiCol_ResizeGripHovered]      = light_accent;
    colors[ImGuiCol_ResizeGripActive]       = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    colors[ImGuiCol_Tab]                    = ImVec4(1.00f, 1.00f, 1.00f, 0.02f);
    colors[ImGuiCol_TabHovered]             = light_accent;
    colors[ImGuiCol_TabActive]              = dark_accent;
    colors[ImGuiCol_TabUnfocused]           = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
    colors[ImGuiCol_TabUnfocusedActive]     = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
    colors[ImGuiCol_DockingPreview]         = ImVec4(0.85f, 0.85f, 0.85f, 0.28f);
    colors[ImGuiCol_DockingEmptyBg]         = ImVec4(0.38f, 0.38f, 0.38f, 1.00f);
    colors[ImGuiCol_PlotLines]              = light_accent;
    colors[ImGuiCol_PlotLinesHovered]       = light_accent;
    colors[ImGuiCol_PlotHistogram]          = ImVec4(0.80f, 0.80f, 0.83f, 1.00f);
    colors[ImGuiCol_PlotHistogramHovered]   = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    colors[ImGuiCol_TableHeaderBg]          = ImVec4(0.19f, 0.19f, 0.20f, 1.00f);
    colors[ImGuiCol_TableBorderStrong]      = ImVec4(0.31f, 0.31f, 0.35f, 1.00f);
    colors[ImGuiCol_TableBorderLight]       = ImVec4(0.23f, 0.23f, 0.25f, 1.00f);
    colors[ImGuiCol_TableRowBg]             = ImVec4(0.23f, 0.23f, 0.25f, 1.00f);
    colors[ImGuiCol_TableRowBgAlt]          = ImVec4(1.00f, 1.00f, 1.00f, 0.07f);
    colors[ImGuiCol_TextSelectedBg]         = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    colors[ImGuiCol_DragDropTarget]         = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
    colors[ImGuiCol_NavHighlight]           = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
    colors[ImGuiCol_NavWindowingHighlight]  = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
    colors[ImGuiCol_NavWindowingDimBg]      = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);
    colors[ImGuiCol_ModalWindowDimBg]       = ImVec4(1.00f, 0.98f, 0.95f, 0.73f);

    ImVec4* pcolors = ImPlot::GetStyle().Colors;
    pcolors[ImPlotCol_Line]          = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_Fill]          = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_MarkerOutline] = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_MarkerFill]    = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_ErrorBar]      = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_FrameBg]       = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_PlotBg]        = ImVec4(0.07f, 0.07f, 0.10f, 0.00f);
    pcolors[ImPlotCol_PlotBorder]    = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    pcolors[ImPlotCol_LegendBg]      = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_LegendBorder]  = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_LegendText]    = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_TitleText]     = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_InlayText]     = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_XAxis]         = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_XAxisGrid]     = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_YAxis]         = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_YAxisGrid]     = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_YAxis2]        = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_YAxisGrid2]    = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_YAxis3]        = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_YAxisGrid3]    = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_Selection]     = ImVec4(0.000f, 0.571f, 1.000f, 1.000f);
    pcolors[ImPlotCol_Query]         = IMPLOT_AUTO_COL;
    pcolors[ImPlotCol_Crosshairs]    = IMPLOT_AUTO_COL;

    auto& pstyle = ImPlot::GetStyle();
    pstyle.PlotPadding = pstyle.LegendPadding = {12,12};
    pstyle.LabelPadding = pstyle.LegendInnerPadding = {6,6};
    pstyle.FitPadding   = {0,0.1f};
    pstyle.LegendSpacing = {2,2};

    auto& io = ImGui::GetIO();
    io.Fonts->Clear();
    ImFontConfig font_cfg;
    font_cfg.PixelSnapH           = true;
    font_cfg.OversampleH          = 1;
    font_cfg.OversampleV          = 1;
    font_cfg.FontDataOwnedByAtlas = false;
    strcpy(font_cfg.Name, "Roboto Bold");
    io.Fonts->AddFontFromMemoryTTF(Roboto_Bold_ttf, Roboto_Bold_ttf_len, 15.0f, &font_cfg);

}