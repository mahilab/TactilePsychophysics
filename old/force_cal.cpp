#include <mahi/gui.hpp>
#include <MEL/Devices/AtiSensor.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Devices/Windows/XboxController.hpp>
#include <MEL/Math.hpp>
#include <MEL/Core.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <MEL/Logging/Log.hpp>
#include "CapstanModule.hpp"
#include <MEL/Utility/RingBuffer.hpp>


using namespace mahi::gui;
using namespace mahi::daq;
using namespace mahi::util;

QuanserOptions qOptions() {
    QuanserOptions options;
    options.set_update_rate(QuanserOptions::UpdateRate::Normal);
    for (uint32 i = 0; i < 8; ++i)
        options.set_encoder_direction(i, QuanserOptions::EncoderDirection::Reversed);
    return options;
}

// Inherit from Application
class ForceCal : public Application {
public:

    enum ControlMode {
        Xbox = 0,
        Torque,
        Position,
        Force
    };

    struct PidGains {
        double kp = 0, ki = 0, kd = 0;
    };

    
    ForceCal() : Application(1000,500,"CM Force Calibration"),
        q8(qOptions()),
        cm("cm", q8.DO[7], q8.DI[7], q8.AO[7], q8.AI[6], q8.AI[7], q8.encoder[7], CM::Config()),
        V(500), F(500),
        traj(Waveform::Sin, hertz(0.5))
    {
        // ImGui::DisableViewports();
        plot1.xAxis.minimum = 0;    plot1.xAxis.maximum = 10;
        plot1.yAxis.minimum = -1;   plot1.yAxis.maximum = 15;
        plot1.xAxis.lockMax = true; plot1.xAxis.lockMin = true;
        plot1.xAxis.showTickLabels = false;
        items1.resize(4);
        items1[0].label = "Nano17 Z [N]"; items1[0].color = Blues::DeepSkyBlue;
        items1[1].label = "CM  Z [V]"; items1[1].color = Yellows::Yellow;
        items1[2].label = "CM  Z [N]"; items1[2].color = Greens::Chartreuse;
        items1[3].label = "Trajectory"; items1[3].color = Pinks::Pink;
        for (int i = 0; i < items1.size(); ++i)
            items1[i].data.reserve(20000);

        items3.resize(3);
        items3[0].label = "dF/dT"; items3[0].color = Cyans::Cyan;
        items3[1].label = "dF/dT (filt)"; items3[1].color = Oranges::DarkOrange ;
        items3[2].label = "Torque Command"; items3[2].color = Greens::Chartreuse;

        for (int i = 0; i < items3.size(); ++i)
            items3[i].data.reserve(20000);

        items2.resize(2);
        items2[0].type = ImGui::PlotItem::Type::Scatter;
        items2[0].size = 1.5;
        items2[0].label = "Data [N/V]";
        items2[0].color = Greens::Chartreuse;
        items2[1].type = ImGui::PlotItem::Type::Line;
        items2[1].label = "Fit [N/V]";
        items2[1].color = Greens::LimeGreen;
        items2[1].data.resize(2);
        plot2.xAxis.minimum = 0.5;
        plot2.xAxis.maximum = 2;
        plot2.yAxis.minimum = 0;
        plot2.yAxis.maximum = 10;        


        butt.configure(2, hertz(100), hertz(1000));

        thrd = std::thread(&ForceCal::controlThread, this);
     }

    ~ForceCal() {
        stop = true;
        if (thrd.joinable())
            thrd.join();
    }

    void update() override {  
        {
            // LOCK
            std::lock_guard<std::mutex> lock(mtx);  
            ImGui::PlotItemBufferPoint(items1[0], threadT, atiZ, 1000);
            ImGui::PlotItemBufferPoint(items1[1], threadT, cmZ, 1000);
            ImGui::PlotItemBufferPoint(items1[2], threadT, fitm * cmZ + fitb, 1000);
            ImGui::PlotItemBufferPoint(items2[0], cmZ, atiZ, 500);

            ImGui::PlotItemBufferPoint(items3[0], threadT, atiZd, 1000);
            ImGui::PlotItemBufferPoint(items3[1], threadT, atiZd_filt, 1000);
            ImGui::PlotItemBufferPoint(items3[2], threadT, torqueCom, 1000);

            plot1.xAxis.maximum = threadT;
            plot1.xAxis.minimum = threadT - 10;
            plot3.xAxis.maximum = threadT;
            plot3.xAxis.minimum = threadT - 10;

            traj.amplitude = std::abs((maxs[(int)state] - mins[(int)state]) * 0.5);
            traj.offset = mins[(int)state] + (maxs[(int)state] - mins[(int)state]) * 0.5;
            traj.period = seconds(1.0 / frqs[(int)state]);

            ImGui::PlotItemBufferPoint(items1[3], threadT, traj.evaluate(seconds(threadT)), 1000);
        }    
        auto [w,h] = getWindowSize();
        ImVec2 pos = ImGui::GetMainViewport()->Pos;
        ImGui::BeginFixed("Example", {pos.x,pos.y}, ImVec2(w,h), ImGuiWindowFlags_NoTitleBar);
        if (ImGui::Button("Zero ATI Nano17"))
            nano17.zero();
        ImGui::SameLine();
        static bool collectData = false;
        ImGui::Checkbox("Collect Data", &collectData);
        if (collectData) {
            for (int i = 0; i < 500; ++i) {
                V[i] = items2[0].data[i].x;
                F[i] = items2[0].data[i].y;
                auto mb = linear_regression(V,F);
                fitm = mb[0];
                fitb = mb[1];
                items2[1].data[0] = ImVec2(0, fitb);
                items2[1].data[1] = ImVec2(2, fitm * 2 + fitb);
            }
        }

        if (ImGui::RadioButton("Xbox", state == Xbox))
            state = Xbox;
        ImGui::SameLine();
        if (ImGui::RadioButton("Torque", state == Torque))
            state = Torque;
        ImGui::SameLine();
        if (ImGui::RadioButton("Position", state == Position))
            state = Position;
        ImGui::SameLine();
        if (ImGui::RadioButton("Force", state == Force))
            state = Force;

        if (state != Xbox) {
            ImGui::SameLine();
            ImGui::PushItemWidth(100);
            ImGui::DragDouble("Min", &mins[(int)state], 0.1);
            ImGui::SameLine();
            ImGui::DragDouble("Max", &maxs[(int)state], 0.1);
            ImGui::SameLine();
            ImGui::DragDouble("Frequency", &frqs[(int)state], 0.1);
            ImGui::PopItemWidth();
        }

        PidGains tmpGu = gainsUp;
        PidGains tmpGd = gainsDown;
        ImGui::DragDouble3("Gains U", &tmpGu.kp, 0.001);
        ImGui::DragDouble3("Gains D", &tmpGd.kp, 0.001);
        gainsUp = tmpGu;
        gainsDown = tmpGd;

        float wavail = ImGui::GetContentRegionAvail().x;
        float havail = ImGui::GetContentRegionAvail().y;
        ImGui::Plot("Force", plot1, items1, {wavail/2,havail/2});
        ImGui::SameLine();
        ImGui::Plot("Force Derivative", plot3, items3, {-1,havail/2});
        ImGui::Plot("Scatter", plot2, items2, {-1,-1});
        ImGui::End();


        auto q = cm.getQuery();
        ImGui::Begin("CM Info");
        ImGui::LabelText("Status", q.status == CM::Enabled ? "Enabled" : "Disabled");
        ImGui::LabelText("Control Value", "%f", q.ctrlValue);
        ImGui::LabelText("Spool Position", "%f", q.spoolPosition);
        ImGui::End();


        sleep(milliseconds(20));
    }

    void controlThread() {
        q8.open();
        q8.enable();
        nano17.load_calibration("FT19612.cal");
        nano17.set_channels(q8.AI[{0,1,2,3,4,5}]);
        q8.update_input();
        nano17.zero();
        cm.setControlMode(CM::Torque);
        XboxController xbox;
        if (!xbox.is_connected())
            LOG(Error) << "No Xbox controller detected!";       

        Timer timer(hertz(250));
        Time tm;
        while (!stop) {
            q8.update_input();  
            auto q = cm.getQuery();
            {          
                // LOCK
                std::lock_guard<std::mutex> lock(mtx);
                threadT = tm.as_seconds();
                cmZ =  q8.AI.get_value(7);
                atiZ =  -nano17.get_force(Axis::AxisZ);
                atiZd = fd.update(atiZ, tm);
                atiZd_filt = butt.update(atiZd, tm);           
                if (xbox.is_button_pressed(XboxController::A) && !cm.is_enabled())
                    cm.enable();
                else if (xbox.is_button_pressed(XboxController::B) && cm.is_enabled())
                    cm.disable();

                if (state == Xbox) {
                    double torque = (xbox.get_axis(XboxController::RT) - xbox.get_axis(XboxController::LT));
                    if (xbox.is_button_pressed(XboxController::RB))
                        torque = 0.3f;
                    if (xbox.is_button_pressed(XboxController::Y))
                        cm.zeroPosition();
                    cm.setControlValue(torque);
                }
                else if (state == Force) {
                    PidGains tmpGu = gainsUp;
                    pidUp.kp = tmpGu.kp;
                    pidUp.ki = tmpGu.ki;
                    pidUp.kd = tmpGu.kd;

                    PidGains tmpGd = gainsDown;
                    pidDown.kp = tmpGd.kp;
                    pidDown.ki = tmpGd.ki;
                    pidDown.kd = tmpGd.kd;

                    double xref = traj(tm);
                    double torque;
                    if (atiZd > 0)
                        torque = pidUp.calculate(xref, atiZ, atiZd_filt, tm);      
                    else   
                        torque = pidDown.calculate(xref, atiZ, atiZd_filt, tm);      
                    torqueCom = torque;
                    cm.setControlValue(torque);    
                }
            }
            cm.update(tm);
            q8.update_output();
            tm = timer.wait();
        }
        q8.disable();
        q8.close();
    }


    Q8Usb q8; 
    AtiSensor nano17;   
    std::mutex mtx;
    std::thread thrd;
    CM cm;
    
    // LOCK FREE
    std::atomic_bool stop = false;
    std::atomic<ControlMode> state = ControlMode::Xbox;
    PidController pidUp, pidDown;
    Differentiator fd;
    Butterworth butt;
    std::atomic<PidGains> gainsUp, gainsDown;

    // SHARED
    double threadT;
    double cmZ;
    double atiZ;
    double atiZd;
    double atiZd_filt;
    double fitm = 1;
    double fitb = 0;  
    double torqueCom;
    Waveform traj;  

    std::vector<double> V, F;

    // NOT SHARED
    ImGui::PlotInterface plot1, plot2, plot3;
    std::vector<ImGui::PlotItem> items1, items2, items3;


    std::vector<double> mins = {0, -1,0,1};
    std::vector<double> maxs = {0, 1,50,6};
    std::vector<double> frqs = {0, 0.5, 0.5, 0.5};
};



int main() {
    ForceCal app;
    std::cout << app.state.is_always_lock_free << std::endl;
    app.run();
    return 0;
}