#include <iostream>
#include <Mahi/Daq.hpp>
#include <Mahi/Gui.hpp>
#include <Mahi/Robo.hpp>
#include <thread>
#include <mutex>
#include <Mahi/Util.hpp>

using namespace mahi::gui;
using namespace mahi::util;
using namespace mahi::robo;
using namespace mahi::daq;


struct RollingBuffer {
    float Span;
    ImVector<ImVec2> Data;
    RollingBuffer() {
        Span = 10.0f;
        Data.reserve(2000);
    }
    void AddPoint(float x, float y) {
        float xmod = fmodf(x, Span);
        if (!Data.empty() && xmod < Data.back().x)
            Data.shrink(0);
        Data.push_back(ImVec2(xmod, y));
    }
};

class MyGui : public Application
{
public:
     MyGui(): Application(500, 500, "MyGui")
    {
        q8.enable();
        q8.encoder.units[0] = 2 * PI / (1024 * 35);
        q8.encoder.units[1] = 2 * PI / (1024 * 35);
        control_thread = std::thread(&MyGui::control_loop, this);

    }

    ~MyGui(){
        q8.disable();
        q8.close();
        stop = true;
        control_thread.join();
    }

    void update() override
    {
        q8.read_all();
        ImGui::Begin("my widget");

        {   std::lock_guard<std::mutex> lock(mtx);
            
            if(ImGui::Button("Enable")){
                q8.DO[0] = TTL_HIGH;
                q8.AO[0] = 0;

                q8.DO[2] = TTL_HIGH;
                q8.AO[1] = 0;
            }

            ImGui::SameLine();

            if(ImGui::Button("Disable")) {
                q8.DO[0] = TTL_LOW;
                q8.AO[0] = 0;

                q8.DO[2] = TTL_LOW;
                q8.AO[1] = 0;
            }

            if(ImGui::Button("Zero Encoder")){
                q8.encoder.zero(0);
                q8.encoder.zero(1);
            }

            ImGui::DragDouble("Kp", &kp, 0.01f, 0, 2.0);
            ImGui::DragDouble("Kd", &kd, 0.001f, 0, 0.2);

            if (ImGui::Checkbox("Follow Sine", &followSine))
                toff = time();

            if (followSine) {
                x_ref1 = 3 * std::sin(2 * PI * 0.25 * time().as_seconds() - toff.as_seconds());
                x_ref2 = 3 * std::cos(2 * PI * 0.25 * time().as_seconds() - toff.as_seconds());
            }
            else {
                ImGui::DragDouble("X Ref Normal", &x_ref1, 0.1f, 0, 3);
                ImGui::DragDouble("X Ref Shear", &x_ref2, 0.1f, -3, 3);
            }

        }

        ImGui::PushItemWidth(100);
        ImGui::LabelText("normal motor encoder counts", "%d", q8.encoder[0]);
        ImGui::LabelText("normal motor encoder position", "%f", q8.encoder.positions[0]);
        ImGui::LabelText("normal motor encoder velocity", "%f", q8.velocity.velocities[0]);
        ImGui::LabelText("normal motor translational commanded torque", "%f", torque1);
        ImGui::Spacing(); 
        ImGui::LabelText("shear motor encoder counts", "%d", q8.encoder[1]);
        ImGui::LabelText("shear motor encoder position", "%f", q8.encoder.positions[1]);
        ImGui::LabelText("shear motor encoder velocity", "%f", q8.velocity.velocities[1]);
        ImGui::LabelText("shear motor translational commanded torque", "%f", torque2);
        ImGui::PopItemWidth();

        ImGui::SliderFloat("History",&history,1,30,"%.1f s");
        pdata1.Span = history;
        pdata2.Span = history;

        static ImPlotAxisFlags pt_axis = ImPlotAxisFlags_NoTickLabels;
        ImPlot::SetNextPlotLimitsX(0, history, ImGuiCond_Always);
        if (ImPlot::BeginPlot("##Rolling", NULL, NULL, ImVec2(-1,150), 0, 0, 0)) {
            ImPlot::PlotLine("Motor 1 Encoder Position", &pdata1.Data[0].x, &pdata1.Data[0].y, pdata1.Data.size(), 0, 2 * sizeof(float));
            ImPlot::PlotLine("Motor 2 Encoder Position", &pdata2.Data[0].x, &pdata2.Data[0].y, pdata2.Data.size(), 0, 2 * sizeof(float));
            ImPlot::EndPlot();
        }

        ImGui::End();
        q8.write_all();
    }

    void control_loop() {
        Timer timer(hertz(1000)); //= Timer(1000_Hz); 
        Time t = Time::Zero;    
        while(!stop) {
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    pos1 = q8.encoder.positions[0];
                    vel1 = q8.velocity.velocities[0];

                    pos2 = q8.encoder.positions[1];
                    vel2 = q8.velocity.velocities[1];
                }

                torque1 = (kp/1e3) * (x_ref1 - pos1) + (kd/1e3) * (0 - vel1);
                double amps1 = torque1 * motor_kt_inv;
                double volts1 = amps1 * amp_gain_inv;            
                
                torque2 = (kp/1e3) * (x_ref2 - pos2) + (kd/1e3) * (0 - vel2);
                double amps2 = torque2 * motor_kt_inv;
                double volts2 = amps2 * amp_gain_inv;
                
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    q8.AO[0] = volts1;
                    q8.AO[1] = volts2;
                    //double curr1 = q8.AO[3];
                    //double curr2 = q8.AO[4];
                }
                
                //t += ImGui::GetIO().DeltaTime;
                pdata1.AddPoint(t.as_seconds(), pos1* 1.0f);
                pdata2.AddPoint(t.as_seconds(), pos2* 1.0f);
                
            t = timer.wait();
        }
    }

    Q8Usb q8;

    double kp = 1;
    double kd = .1;

    double pos1 = 0;
    double pos2 = 0;
    double vel1 = 0;
    double vel2 = 0;

    //double torque = 0;
    double x_ref1 = 0;
    double x_ref2 = 0;

    double torque1 = 0;
    double torque2 = 0;

    bool followSine = false;
    Time toff = Time::Zero;

    const double amp_gain_inv = 10 / 1.35;  // V/A
    const double motor_kt_inv = 1.0 / 0.0146; // A/mNm

    RollingBuffer pdata1, pdata2;
    float t = 0;
    float history = 10.0f;

    std::thread control_thread;
    std::atomic_bool stop = false;
    std::mutex mtx;

};

int main(int, char **)
{
    MyGui gui;
    gui.run();
    return 0;
}



