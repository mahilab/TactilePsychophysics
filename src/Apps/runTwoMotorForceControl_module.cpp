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


struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
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
        cm_n->setControlMode(CM::ControlMode::Force);
        cm_n->setForceCtrlCmdSign(1);
        cm_n->setForceSenseSign(1);
        cm_n->setPositionSenseSign(0);
        cm_n->setVelocityMax(50, 1);
        cm_n->setTorqueMax(0.75,1);
        cm_n->setForceRange(forceMin, forceMax); //(-2, 2); //[N]
        cm_n->setForceGains(500.0/1e6,0.0,15.0/1e6);
        cm_n->setControlValue(0.0);
    
        // initialize tangential capstan module
        cm_t->zeroPosition(); //makes wherever it is when the program starts the zero position
        cm_t->setControlMode(CM::ControlMode::Force);
        cm_t->setForceCtrlCmdSign(0);
        cm_t->setForceSenseSign(0);
        cm_t->setPositionSenseSign(0);
        cm_t->setVelocityMax(50, 1);
        cm_t->setTorqueMax(0.75,1);
        cm_t->setForceRange(forceMin, forceMax); //(-2, 2); //[N]
        cm_t->setForceGains(500.0/1e6,0.0,15.0/1e6);
        cm_t->setControlValue(0.0);
    }

    ~MyGui(){
        cm_n->disable();
        cm_t->disable();
        hub.stop();
    }

    void update() override
    {
        //ImGui::Begin("my widget");
        ImGui::BeginFixed("##Force Control with Module Code", {0,0}, {width, height}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings);

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

        ImGui::SameLine();

        if (ImGui::Button("Zero Force")){
            cm_n->zeroForce();
            cm_t->zeroForce();
        }

        ImGui::DragDouble("Kp1 Normal", &kp1, 1.0f, 0, 700);
        ImGui::DragDouble("Kd1 Normal", &kd1, 0.1f, 0, 30);
        ImGui::DragDouble("Force Kff Normal", &forceKff1, 0.001, -1, 1);

        ImGui::DragDouble("Kp2 Shear", &kp2, 1.01f, 0, 700);
        ImGui::DragDouble("Kd2 Shear", &kd2, 0.1f, 0, 30);
        ImGui::DragDouble("Force Kff Shear", &forceKff2, 0.001, -1, 1);

        if (ImGui::Checkbox("Follow Sine", &followSine))
            toff = time();

        if (followSine) {
            f_ref1 = 3.0 * std::sin(2 * PI * 0.25 * time().as_seconds() - toff.as_seconds());
            f_ref2 = 3.0 * std::cos(2 * PI * 0.25 * time().as_seconds() - toff.as_seconds());
        }
        else {
            ImGui::DragDouble("X Ref Normal", &f_ref1, 0.1f, forceMin, forceMax);
            ImGui::DragDouble("X Ref Shear", &f_ref2, 0.1f, forceMin, forceMax);
        }

        cm_n->setForceGains(kp1/1e6,0,kd1/1e6);
        cm_n->setControlValue(cm_n->scaleRefToCtrlValue(f_ref1));
        cm_n->limits_exceeded();

        cm_t->setForceGains(kp2/1e6,0,kd2/1e6);
        cm_t->setControlValue(cm_t->scaleRefToCtrlValue(f_ref2));
        cm_t->limits_exceeded();

        queryN = cm_n->getQuery();
        queryT = cm_t->getQuery();

        double f_act1 = cm_n->getForce(0);
        double f_act2 = cm_t->getForce(0);
        double dfdt1 =  cm_n->getForce(CM::Lowpass);
        double dfdt2 =  cm_t->getForce(CM::Lowpass);

        double torque1 = -((kp1/1e6) * (f_ref1 - f_act1) + (kd1/1e6) * (0 - hub.daq.velocity.velocities[0]));
        std::cout << std::endl;
        std::cout << "enc counts " << cm_n->getEncoderCounts() << " | " << hub.daq.encoder[0] << std::endl;
        std::cout << "force " << cm_n->getForce() << std::endl;
        std::cout << "velocity " << hub.daq.encoder.positions[0] << std::endl;
        std::cout << "dfdt " << dfdt1 << std::endl;
        std::cout << "torque " << cm_n->getMotorTorqueCommand() << " | " << torque1 << std::endl;
        std::cout << "cv " << cm_n->scaleRefToCtrlValue(f_ref1) << std::endl;

        ImGui::PushItemWidth(100);
        ImGui::Text("Motor 1 - Normal Dir - Encoder Info");
        ImGui::LabelText("normal motor encoder counts", "%d", cm_n->getEncoderCounts());
        ImGui::LabelText("normal motor encoder position", "%f", cm_n->getMotorPosition());
        ImGui::LabelText("normal motor encoder velocity", "%f", cm_n->getMotorVelocity());
        ImGui::Spacing(); 

        ImGui::Text("Motor 2 - Shear Dir - Encoder Info");
        ImGui::LabelText("shear motor encoder counts", "%d", cm_t->getEncoderCounts());
        ImGui::LabelText("shear motor encoder position", "%f", cm_t->getMotorPosition());
        ImGui::LabelText("shear motor encoder velocity", "%f", cm_t->getMotorVelocity());
        ImGui::Spacing();

        // ImGui::Text("Motor 1 - Normal Dir - Spool Info");
        // ImGui::LabelText("normal motor translational position", "%f", cm_n->getSpoolPosition());
        // ImGui::LabelText("normal motor translational velocity", "%f", cm_n->getSpoolVelocity());
        // ImGui::LabelText("normal motor translational commanded torque", "%f", cm_n->getMotorTorqueCommand());
        // ImGui::Spacing(); 

        // ImGui::Text("Motor 2 - Shear Dir - Spool Info");
        // ImGui::LabelText("shear motor translational position", "%f", cm_t->getSpoolPosition());
        // ImGui::LabelText("shear motor translational velocity", "%f", cm_t->getSpoolVelocity());
        // ImGui::LabelText("shear motor translational commanded torque", "%f", cm_t->getMotorTorqueCommand());
        // ImGui::Spacing(); 

        ImGui::Text("ATI Forces");
        ImGui::LabelText("ati z force - motor 1", "%f", f_act1);
        ImGui::LabelText("ati x force - motor 2", "%f", f_act2);
        ImGui::PopItemWidth();

        t += ImGui::GetIO().DeltaTime;
        fdata1.AddPoint(t, f_act1* 1.0f);
        fdata2.AddPoint(t, f_act2* 1.0f);
        dfdtData1.AddPoint(t, dfdt1* 1.0f);
        dfdtData2.AddPoint(t, dfdt2* 1.0f);

        ImGui::SliderFloat("History",&history,1,30,"%.1f s");

        static ImPlotAxisFlags ft_axis = ImPlotAxisFlags_NoTickLabels;
        ImPlot::SetNextPlotLimitsX(t - history, t, ImGuiCond_Always);
        if (ImPlot::BeginPlot("##Force", NULL, NULL, ImVec2(-1,200), 0, 0, 0)) {
            ImPlot::PlotLine("ATI Z - Motor 1 Normal", &fdata1.Data[0].x, &fdata1.Data[0].y, fdata1.Data.size(), fdata1.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("ATI X - Motor 2 Shear", &fdata2.Data[0].x, &fdata2.Data[0].y, fdata2.Data.size(), fdata2.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }

        ImGui::Separator();

        ImPlot::SetNextPlotLimitsX(t - history, t, ImGuiCond_Always);
        if (ImPlot::BeginPlot("##dFdt", NULL, NULL, ImVec2(-1,200), 0, 0, 0)) {
            ImPlot::PlotLine("dFdt Z - Motor 1 Normal", &dfdtData1.Data[0].x, &dfdtData1.Data[0].y, dfdtData1.Data.size(), dfdtData1.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("dFdt X - Motor 2 Shear", &dfdtData2.Data[0].x, &dfdtData2.Data[0].y, dfdtData2.Data.size(), dfdtData2.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }
        ImGui::End();
    }


    CMHub hub;
    float width = 500;
    float height = 1000;    
    std::shared_ptr<CM> cm_n; 
    std::shared_ptr<CM> cm_t; 

    int id_n = 0;
    int id_t = 1;

    double kp1 = 500;
    double kd1 = 15;
    double forceKff1 = 0;

    double kp2 = 500;
    double kd2 = 15;
    double forceKff2 = 0;

    //double torque = 0;
    double f_ref1 = 0;
    double f_ref2 = 0;

    double f_act1 = 0;
    double f_act2 = 0;

    double forceMax = 2.0;
    double forceMin = -2.0;

    CM::Query queryN;
    CM::Query queryT;

    // Gui and reference path variables
    bool followSine = false;
    Time toff = Time::Zero;

    ScrollingBuffer fdata1, fdata2, dfdtData1, dfdtData2;
    float t = 0;
    float history = 30.0f;
};

int main(int, char **)
{
    MyGui gui;
    gui.run();
    return 0;
}



