#include <iostream>
#include "CMHub.hpp"
#include "CapstanModule.hpp"
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
        //&hub.daq.enable(); // does the hub start take care of this?
        hub.createDevice(id_n, 0, 0, 0, 0, Axis::AxisZ, "FT06833.cal", {0,1,2,3,4,5},0); 
        cm_n = hub.getDevice(id_n); 

        hub.createDevice(id_t, 2, 2, 1, 1, Axis::AxisX, "FT06833.cal", {0,1,2,3,4,5},0); 
        cm_t = hub.getDevice(id_t);

        // start hub (runs asynchronously in another thread)
        hub.start();

        // initialize normal capstan module
        cm_n->zeroPosition(); //makes wherever it is when the program starts the zero position
        cm_n->setControlMode(CM::ControlMode::Position);
        cm_n->setPosCtrlCmdSign(1);
        cm_n->setForceSenseSign(1);
        cm_n->setPositionSenseSign(0);
        cm_n->setVelocityMax(50, 1);
        cm_n->setTorqueMax(0.75,1);
        cm_n->setPositionRange(posMin, posMax); //(0, 65); //[mm]
        cm_n->setPositionGains(1.0/1e3,0.1/1e3);
        cm_n->setControlValue(0.0);
    
        // initialize tangential capstan module
        cm_t->zeroPosition(); //makes wherever it is when the program starts the zero position
        cm_t->setControlMode(CM::ControlMode::Position);
        cm_t->setPosCtrlCmdSign(1);
        cm_t->setForceSenseSign(0);
        cm_t->setPositionSenseSign(0);
        cm_t->setVelocityMax(50, 1);
        cm_t->setTorqueMax(0.75,1);
        cm_t->setPositionRange(posMin, posMax); //(0, 65); //[mm]
        cm_t->setPositionGains(1.0/1e3,0.1/1e3);
        cm_t->setControlValue(0.0);
    }

    ~MyGui(){
        cm_n->disable();
        cm_t->disable();
        hub.stop();
    }

    void update() override
    {
        ImGui::Begin("my widget");

        if(ImGui::Button("Enable")){
            cm_n->enable();
            cm_t->enable();
        }

        ImGui::SameLine();

        if(ImGui::Button("Disable")) {
            cm_n->disable();
            cm_t->disable();
        }

        if(ImGui::Button("Zero Encoder")){
            cm_n->zeroPosition();
            cm_t->zeroPosition();
        }

        ImGui::DragDouble("Kp", &kp, 0.01f, 0, 2.0);
        ImGui::DragDouble("Kd", &kd, 0.001f, 0, 0.2);

        if (ImGui::Checkbox("Follow Sine", &followSine))
            toff = time();

        if (ImGui::Button("Stay in Current Position"))
            stayPut = true;
        
        
        if (followSine) {
            x_ref1 = 0.3 * std::sin(2 * PI * 0.25 * time().as_seconds() - toff.as_seconds());
            x_ref2 = 0.3 * std::cos(2 * PI * 0.25 * time().as_seconds() - toff.as_seconds());
        }else if (stayPut){
            x_ref1 = cm_n->getSpoolPosition();
            x_ref2 = cm_t->getSpoolPosition();
            stayPut = false;
        }else {
            ImGui::DragDouble("X Ref Normal", &x_ref1, 0.1f, posMin, posMax);
            ImGui::DragDouble("X Ref Shear", &x_ref2, 0.1f, posMin, posMax);
        }

        cm_n->setPositionGains(kp/1e3,kd/1e3);
        cm_n->setControlValue(cm_n->scaleRefToCtrlValue(x_ref1));
        cm_n->limits_exceeded();

        cm_t->setPositionGains(kp/1e3,kd/1e3);
        cm_t->setControlValue(cm_t->scaleRefToCtrlValue(x_ref2));
        cm_t->limits_exceeded();

        double pos1 = cm_n->getMotorPosition();
        double pos2 = cm_t->getMotorPosition();
        double torque1 = (kp/1e3) * (x_ref1 - hub.daq.encoder.positions[0]) + (kd/1e3) * (0 - hub.daq.velocity.velocities[0]);
        // std::cout << std::endl;
        // std::cout << "enc counts " << cm_n->getEncoderCounts() << " | " << hub.daq.encoder[0] << std::endl;
        // std::cout << "position " << cm_n->getMotorPosition() << " | " << hub.daq.encoder.positions[0] << std::endl;
        // std::cout << "velocity " << cm_n->getMotorVelocity() << " | " << hub.daq.velocity.velocities[0] << std::endl;
        // std::cout << "torque " << cm_n->getMotorTorqueCommand() << " | " << torque1 << std::endl;
        // std::cout << "cv " << cm_n->scaleRefToCtrlValue(x_ref1) << std::endl;

        ImGui::PushItemWidth(100);
        ImGui::LabelText("normal motor encoder counts", "%d", cm_n->getEncoderCounts());
        ImGui::LabelText("normal motor encoder position", "%f", pos1);
        ImGui::LabelText("normal motor encoder velocity", "%f", cm_n->getMotorVelocity());
        ImGui::LabelText("normal motor translational commanded torque", "%f", cm_n->getMotorTorqueCommand());
        ImGui::Spacing(); 
        ImGui::LabelText("shear motor encoder counts", "%d", cm_t->getEncoderCounts());
        ImGui::LabelText("shear motor encoder position", "%f", pos2);
        ImGui::LabelText("shear motor encoder velocity", "%f", cm_t->getMotorVelocity());
        ImGui::LabelText("shear motor translational commanded torque", "%f", cm_t->getMotorTorqueCommand());
        ImGui::PopItemWidth();

        t += ImGui::GetIO().DeltaTime;
        pdata1.AddPoint(t, pos1*1.0f);
        pdata2.AddPoint(t, pos2*1.0f);
        //std::cout << std::endl;
        //std::cout << pdata1.Data[0].x << " | " << pdata2.Data[0].x << " || " << t << std::endl;
        //std::cout << "pdata1/2" << pdata1.Data[0].y << " | " << pdata2.Data[0].y << " || pos1/2: " << pos1 << " | " << pos2 << std::endl;
        
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
    }


    CMHub hub;
    std::shared_ptr<CM> cm_n; 
    std::shared_ptr<CM> cm_t; 

    int id_n = 0;
    int id_t = 1;

    double kp = 1;
    double kd = .1;

    double x_ref1 = 0;
    double x_ref2 = 0;

    double posMax = 5.5;
    double posMin = -5.5;

    // Gui and reference path variables
    bool followSine = false;
    bool stayPut = false;
    Time toff = Time::Zero;

    RollingBuffer pdata1, pdata2;
    float t = 0;
    float history = 10.0f;
};

int main(int, char **)
{
    MyGui gui;
    gui.run();
    return 0;
}



