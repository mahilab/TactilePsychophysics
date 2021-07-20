#include <iostream>
#include <Mahi/Daq.hpp>
#include <Mahi/Gui.hpp>
#include <Mahi/Robo.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Robo/Mechatronics/AtiSensor.hpp>

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

bool velocity_limit_exceeded(double m_velocity, bool has_velocity_limit_, double velocityMax) {
    bool exceeded = false;
    if (has_velocity_limit_ && abs(m_velocity) > velocityMax) {
        LOG(Warning) << "Capstan Module velocity exceeded the velocity limit " << velocityMax << " with a value of " << m_velocity;
        exceeded = true;
        ImGui::Button("Disable");
    }
    return exceeded;
}

bool torque_limit_exceeded(double tor, bool has_torque_limit_, double torqueMax) {
    bool exceeded = false;
    if (has_torque_limit_ && abs(tor) > torqueMax) {
        LOG(Warning) << "Capstan Module command torque exceeded the torque limit " << torqueMax << " with a value of " << tor;
        exceeded = true;
        ImGui::Button("Disable");
    }
    return exceeded;
}

class MyGui : public Application
{
public:
     MyGui(): 
        Application(500, 500, "MyGui")
    {
        q8.enable();
        q8.encoder.units[0] = 2 * mahi::util::PI / (1024 * 35);
        q8.encoder.units[1] = 2 * mahi::util::PI / (1024 * 35);

        nano17.load_calibration("FT06833.cal");
        nano17.set_channels(&q8.AI[0], &q8.AI[1], &q8.AI[2], &q8.AI[3], &q8.AI[4], &q8.AI[5]);
    }

    ~MyGui(){
        q8.disable();
        q8.close();
    }

    void update() override
    {
        q8.read_all();
        ImGui::Begin("my widget");
        
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

        ImGui::SameLine();

        if (ImGui::Button("Zero Force"))
            nano17.zero();

        ImGui::DragDouble("Kp1 Normal", &kp1, 0.01f, 0, 10);
        ImGui::DragDouble("Kd1 Normal", &kd1, 0.001f, 0, 1);
        ImGui::DragDouble("Force Kff Normal", &forceKff1, 0.001, -1, 1);

        ImGui::DragDouble("Kp2 Shear", &kp2, 0.01f, 0, 10);
        ImGui::DragDouble("Kd2 Shear", &kd2, 0.001f, 0, 1);
        ImGui::DragDouble("Force Kff Shear", &forceKff2, 0.001, -1, 1);

        if (ImGui::Checkbox("Follow Sine", &followSine))
            toff = time();

        if (followSine) {
            f_ref1 = 3 * std::sin(2 * mahi::util::PI * 0.25 * time().as_seconds() - toff.as_seconds());
            f_ref2 = 3 * std::cos(2 * mahi::util::PI * 0.25 * time().as_seconds() - toff.as_seconds());
        }
        else {
            ImGui::DragDouble("F Ref Normal", &f_ref1, 0.1f, 0, 3);
            ImGui::DragDouble("F Ref Shear", &f_ref2, 0.1f, -3, 3);
        }

        /* double pos1 = q8.encoder.positions[0];
        double vel1 = q8.velocity.velocities[0];
        double torque1 = kp1 * (f_ref1 - pos1) + kd1 * (0 - vel1);
        double amps1 = torque1 * motor_kt_inv;
        double volts1 = amps1 * amp_gain_inv;
        q8.AO[0] = volts1;

        double pos2 = q8.encoder.positions[1];
        double vel2 = q8.velocity.velocities[1];
        double torque2 = kp2 * (f_ref2 - pos2) + kd2 * (0 - vel2);
        double amps2 = torque2 * motor_kt_inv;
        double volts2 = amps2 * amp_gain_inv;
        q8.AO[1] = volts2;
        */

        // Motor 1 - Normal Force
        double f_act1 = nano17.get_force(Axis::AxisZ);
        double v_enc1  = q8.velocity.velocities[0];
        double v_spool1  = v_enc1 * gearRatio;
        double p_enc1 = q8.encoder.positions[0];
        double p_spool1 = p_enc1 * gearRatio;;

        double torque1 = kp1 * (f_ref1 - f_act1) + kd1 * (0 - v_spool1);
        // ff term
        double torque_ff1 = motorStallTorque * forceKff1;
        torque1 += torque_ff1 * f_ref1;
        // prevent unwinding when controller wants to do less force the the weight of cm
        if (p_spool1 < -10 && torque1 < 0)
            torque1 = 0;
        //if (m_params.filterOutputValue)
        //    torque1 = m_outputFilter.update(torque);
        double amps1 = torque1 * motor_kt_inv; // divide in other code / m_params.motorTorqueConstant;
        double volts1 = amps1 * amp_gain_inv; //  divide in other code / m_params.commandGain;
        q8.AO[0] = volts1;
        
        // Motor 2 - Shear Force
        double f_act3 = nano17.get_force(Axis::AxisY);
        double f_act2 = nano17.get_force(Axis::AxisX);
        double v_enc2  = q8.velocity.velocities[0];
        double v_spool2  = v_enc2 * gearRatio;
        double p_enc2 = q8.encoder.positions[0];
        double p_spool2 = p_enc2 * gearRatio;;

        double torque2 = kp2 * (f_ref2 - f_act2) + kd2 * (0 - v_spool2);
        // ff term
        double torque_ff2 = motorStallTorque * forceKff2;
        torque2 += torque_ff2 * f_ref2;
        // prevent unwinding when controller wants to do less force the the weight of cm
        if (p_spool2 < -10 && torque2 < 0)
            torque2 = 0;
        //if (m_params.filterOutputValue)
        //    torque2 = m_outputFilter.update(torque);
        double amps2 = torque2 * motor_kt_inv; // divide in other code / m_params.motorTorqueConstant;
        double volts2 = amps2 * amp_gain_inv; //  divide in other code / m_params.commandGain;
        q8.AO[0] = volts2;


        //double curr1 = q8.AO[3];
        //double curr2 = q8.AO[4];

        //ImGui::DragDouble("Motor Torque", &torque, 0.01f, -0.5, 0.5, "%.3f mNm");
        
        ImGui::PushItemWidth(100);
        ImGui::Text("Motor 1 - Normal Dir - Encoder Info");
        ImGui::LabelText("normal motor encoder counts", "%d", q8.encoder[0]);
        ImGui::LabelText("normal motor encoder position", "%f", q8.encoder.positions[0]);
        ImGui::LabelText("normal motor encoder velocity", "%f", q8.velocity.velocities[0]);
        ImGui::Spacing(); 

        ImGui::Text("Motor 2 - Shear Dir - Encoder Info");
        ImGui::LabelText("shear motor encoder counts", "%d", q8.encoder[1]);
        ImGui::LabelText("shear motor encoder position", "%f", q8.encoder.positions[1]);
        ImGui::LabelText("shear motor encoder velocity", "%f", q8.velocity.velocities[1]);
        ImGui::Spacing();

        ImGui::Text("Motor 1 - Normal Dir - Spool Info");
        ImGui::LabelText("normal motor translational position", "%f", p_spool1);
        ImGui::LabelText("normal motor translational velocity", "%f", v_spool1);
        ImGui::Spacing(); 

        ImGui::Text("Motor 2 - Shear Dir - Spool Info");
        ImGui::LabelText("shear motor translational position", "%f", p_spool2);
        ImGui::LabelText("shear motor translational velocity", "%f", v_spool2);
        ImGui::Spacing(); 

        ImGui::Text("ATI Forces");
        ImGui::LabelText("ati z force - motor 1", "%f", nano17.get_force(Axis::AxisZ));
        ImGui::LabelText("ati x force - motor 2???", "%f", nano17.get_force(Axis::AxisX));
        ImGui::LabelText("ati y force", "%f", nano17.get_force(Axis::AxisY));
        ImGui::PopItemWidth();

        t += ImGui::GetIO().DeltaTime;
        pdata1.AddPoint(t, f_act1* 1.0f);
        pdata2.AddPoint(t, f_act2* 1.0f);
        pdata3.AddPoint(t, f_act3* 1.0f);

        ImGui::SliderFloat("History",&history,1,30,"%.1f s");
        pdata1.Span = history;
        pdata2.Span = history;
        pdata3.Span = history;

        static ImPlotAxisFlags pt_axis = ImPlotAxisFlags_NoTickLabels;
        ImPlot::SetNextPlotLimitsX(0, history, ImGuiCond_Always);
        if (ImPlot::BeginPlot("##Rolling", NULL, NULL, ImVec2(-1,150), 0, 0, 0)) {
            ImPlot::PlotLine("ATI Z - Motor 1 Normal", &pdata1.Data[0].x, &pdata1.Data[0].y, pdata1.Data.size(), 0, 2 * sizeof(float));
            ImPlot::PlotLine("ATI X - Motor 2 Shear??", &pdata2.Data[0].x, &pdata2.Data[0].y, pdata2.Data.size(), 0, 2 * sizeof(float));
            ImPlot::PlotLine("ATI Y", &pdata3.Data[0].x, &pdata3.Data[0].y, pdata3.Data.size(), 0, 2 * sizeof(float));
            ImPlot::EndPlot();
        }

        ImGui::End();
        q8.write_all();
    }

    Q8Usb q8;
    AtiSensor nano17;

    double kp1 = 10;
    double kd1 = 1;
    double forceKff1 = 1;

    double kp2 = 10;
    double kd2 = 1;
    double forceKff2 = 1;

    //double torque = 0;
    double f_ref1 = 0;
    double f_ref2 = 0;

    bool followSine = false;
    Time toff = Time::Zero;

    const double amp_gain_inv = 10 / 1.35;  // V/A
    const double motor_kt_inv = 1.0 / 14.6; // A/mNm
    double motorStallTorque    = 0.0197;        // [Nm]
    double gearRatio           = 0.332*25.4*mahi::util::PI/180.0;    // [mm/deg] from spool pitch diameter (.332") and capstan radius if applicable, converted to mm

    bool   has_velocity_limit_ = 1;
    bool   has_torque_limit_   = 1;
    double velocityMax         = 100; // [mm/s] ????
    double torqueMax           = 10; // [Nm] ?????

    RollingBuffer pdata1, pdata2, pdata3;
    float t = 0;
    float history = 10.0f;

};

int main(int, char **)
{
    MyGui gui;
    gui.run();
}



