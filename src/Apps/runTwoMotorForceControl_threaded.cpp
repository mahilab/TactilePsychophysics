#include <iostream>
#include <Mahi/Daq.hpp>
#include <Mahi/Gui.hpp>
#include <Mahi/Robo.hpp>
#include <thread>
#include <mutex>
#include <Mahi/Util.hpp>
#include <Mahi/Robo/Mechatronics/AtiSensor.hpp>

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

bool velocity_limit_exceeded(double m_velocity, bool has_velocity_limit_, double velocityMax) {
    bool exceeded = false;
    if (has_velocity_limit_ && abs(m_velocity) > velocityMax) {
        LOG(Warning) << "Capstan Module velocity exceeded the velocity limit " << velocityMax << " with a value of " << m_velocity;
        exceeded = true;
        
    }
    return exceeded;
}

bool force_limit_exceeded(double force, bool has_force_limit_, double forceMax) {
    bool exceeded = false;
    if (has_force_limit_ && abs(force) > forceMax) {
        LOG(Warning) << "Capstan Module command force exceeded the force limit " << forceMax << " with a value of " << force;
        exceeded = true;
        
    }
    return exceeded;
}

bool torque_limit_exceeded(double tor, bool has_torque_limit_, double torqueMax) {
    bool exceeded = false;
    if (has_torque_limit_ && abs(tor) > torqueMax) {
        LOG(Warning) << "Capstan Module command torque exceeded the torque limit " << torqueMax << " with a value of " << tor;
        exceeded = true;
        
    }
    return exceeded;
}

class MyGui : public Application
{
public:
     MyGui(): Application(500, 500, "MyGui")
    {
        q8.enable();
        q8.encoder.units[0] = 2 * mahi::util::PI / (1024 * 35);
        q8.encoder.units[1] = 2 * mahi::util::PI / (1024 * 35);
        q8.encoder.zero(0);
        q8.encoder.zero(1);

        nano17.load_calibration("FT06833.cal");
        nano17.set_channels(&q8.AI[0], &q8.AI[1], &q8.AI[2], &q8.AI[3], &q8.AI[4], &q8.AI[5]);
        q8.read_all();
        nano17.zero();
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
        ImGui::Begin("my widget", &open);

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

            ImGui::SameLine();

            if (ImGui::Button("Zero Force"))
                nano17.zero();

            ImGui::DragDouble("Kp1 Normal", &kp1, 1.0f, 0, 300);
            ImGui::DragDouble("Kd1 Normal", &kd1, 0.1f, 0, 30);
            ImGui::DragDouble("Force Kff Normal", &forceKff1, 0.001, -1, 1);

            ImGui::DragDouble("Kp2 Shear", &kp2, 1.01f, 0, 300);
            ImGui::DragDouble("Kd2 Shear", &kd2, 0.1f, 0, 30);
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

        }

        // ImGui::PushItemWidth(100);
        // ImGui::Text("Motor 1 - Normal Dir - Encoder Info");
        // ImGui::LabelText("normal motor encoder counts", "%d", q8.encoder[0]);
        // ImGui::LabelText("normal motor encoder position", "%f", q8.encoder.positions[0]);
        // ImGui::LabelText("normal motor encoder velocity", "%f", q8.velocity.velocities[0]);
        // ImGui::Spacing(); 

        // ImGui::Text("Motor 2 - Shear Dir - Encoder Info");
        // ImGui::LabelText("shear motor encoder counts", "%d", q8.encoder[1]);
        // ImGui::LabelText("shear motor encoder position", "%f", q8.encoder.positions[1]);
        // ImGui::LabelText("shear motor encoder velocity", "%f", q8.velocity.velocities[1]);
        // ImGui::Spacing();

        // ImGui::Text("Motor 1 - Normal Dir - Spool Info");
        // ImGui::LabelText("normal motor translational position", "%f", p_spool1);
        // ImGui::LabelText("normal motor translational velocity", "%f", v_spool1);
        // ImGui::LabelText("normal motor translational commanded torque", "%f", torque1);
        // ImGui::LabelText("normal motor translational commanded volts", "%f", volts1);
        // ImGui::Spacing(); 

        // ImGui::Text("Motor 2 - Shear Dir - Spool Info");
        // ImGui::LabelText("shear motor translational position", "%f", p_spool2);
        // ImGui::LabelText("shear motor translational velocity", "%f", v_spool2);
        // ImGui::LabelText("shear motor translational commanded torque", "%f", torque2);
        // ImGui::LabelText("normal motor translational commanded volts", "%f", volts2);
        // ImGui::Spacing(); 

        ImGui::Text("ATI Forces");
        ImGui::LabelText("ati z force - motor 1", "%f", f_act1);
        ImGui::LabelText("ati x force - motor 2???", "%f", f_act2);
        ImGui::LabelText("ati y force", "%f", f_act3);
        ImGui::PopItemWidth();

        t += ImGui::GetIO().DeltaTime;
        fdata1.AddPoint(t, f_act1* 1.0f);
        fdata2.AddPoint(t, f_act2* 1.0f);
        fdata3.AddPoint(t, f_act3* 1.0f);

        ImGui::SliderFloat("History",&history,1,30,"%.1f s");

        static ImPlotAxisFlags ft_axis = ImPlotAxisFlags_NoTickLabels;
        ImPlot::SetNextPlotLimitsX(t - history, t, ImGuiCond_Always);
        if (ImPlot::BeginPlot("##Scrolling", NULL, NULL, ImVec2(-1,-1), 0, 0, 0)) {
            ImPlot::PlotLine("ATI Z - Motor 1 Normal", &fdata1.Data[0].x, &fdata1.Data[0].y, fdata1.Data.size(), fdata1.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("ATI X - Motor 2 Shear", &fdata2.Data[0].x, &fdata2.Data[0].y, fdata2.Data.size(), fdata2.Offset, 2*sizeof(float));
            ImPlot::PlotLine("ATI Y", &fdata3.Data[0].x, &fdata3.Data[0].y, fdata3.Data.size(), fdata3.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }

        ImGui::End();
        q8.write_all();

        if (!open){
            quit();    
        }
    }

    void control_loop() {
        Timer timer(hertz(1000));
        Time t = Time::Zero;    
        while(!stop) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                // Motor 1 - Normal Force ////////////////////////////////////
                f_act1 = -nano17.get_force(Axis::AxisZ);
                v_enc1  = q8.velocity.velocities[0];
                p_enc1 = q8.encoder.positions[0];
                //double curr1 = q8.AO[3];
            }

            v_spool1  = v_enc1 * gearRatio;
            p_spool1 = p_enc1 * gearRatio;

            torque1 = -((kp1/1e6) * (f_ref1 - f_act1) + (kd1/1e6) * (0 - v_spool1));
        
            // ff term
            torque_ff1 = motorStallTorque * (forceKff1/1e6);
            torque1 += torque_ff1 * f_ref1;
            // prevent unwinding when controller wants to do less force the the weight of cm
            //if (p_spool1 < -10 && torque1 < 0)
            //    torque1 = 0;
            //if (m_params.filterOutputValue)
            //    torque1 = m_outputFilter.update(torque);
            amps1 = torque1 * motor_kt_inv; // divide in other code / m_params.motorTorqueConstant;
            volts1 = amps1 * amp_gain_inv; //  divide in other code / m_params.commandGain;
            
            {
                std::lock_guard<std::mutex> lock(mtx);
                q8.AO[0] = volts1;
            }


            if(velocity_limit_exceeded(v_spool1, has_velocity_limit_, velocityMax) ||
            force_limit_exceeded(f_act1, has_force_limit_, forceMax) || torque_limit_exceeded(torque1, has_torque_limit_, torqueMax)){
                open = false;
            }

            // Motor 2 - Shear Force ////////////////////////////////////////
            {    
                std::lock_guard<std::mutex> lock(mtx);
                f_act2 = -nano17.get_force(Axis::AxisX);
                v_enc2  = q8.velocity.velocities[0];
                p_enc2 = q8.encoder.positions[0];
                //double curr2 = q8.AO[4];
            }

            v_spool2  = v_enc2 * gearRatio;
            p_spool2 = p_enc2 * gearRatio;

            torque2 = -((kp2/1e6) * (f_ref2 - f_act2) + (kd2/1e6) * (0 - v_spool2));
            // ff term
            torque_ff2 = motorStallTorque * (forceKff2/1e6);
            torque2 += torque_ff2 * f_ref2;
            // prevent unwinding when controller wants to do less force the the weight of cm
            if (p_spool2 < -10 && torque2 < 0)
                torque2 = 0;
            //if (m_params.filterOutputValue)
            //    torque2 = m_outputFilter.update(torque);
            amps2 = torque2 * motor_kt_inv; // divide in other code / m_params.motorTorqueConstant;
            volts2 = amps2 * amp_gain_inv; //  divide in other code / m_params.commandGain;
            
            {
                std::lock_guard<std::mutex> lock(mtx);
                q8.AO[1] = volts2;
            }

            if(velocity_limit_exceeded(v_spool2, has_velocity_limit_, velocityMax) ||
            force_limit_exceeded(f_act2, has_force_limit_, forceMax) || torque_limit_exceeded(torque1, has_torque_limit_, torqueMax)){
                open = false;
            }

            // Out of haptic plane - Orthogonal Force ////////////////////////
            {
                std::lock_guard<std::mutex> lock(mtx);
                f_act3 = -nano17.get_force(Axis::AxisY);
            }
            if(force_limit_exceeded(f_act3, has_force_limit_, forceMax)){
                open = false;
            }
            t = timer.wait();
        }
    }

 bool open = true;

    Q8Usb q8;
    AtiSensor nano17;

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
    double f_act3 = 0;

    double v_enc1  = 0;
    double v_spool1  = 0;
    double p_enc1 = 0;
    double p_spool1 = 0;
    double torque_ff1 = 0;
    double torque1 = 0;
    double volts1 = 0;
    double amps1 = 0;

    double v_enc2  = 0;
    double v_spool2  = 0;
    double p_enc2 = 0;
    double p_spool2 = 0;
    double torque_ff2 = 0;
    double torque2 = 0;
    double volts2 = 0;
    double amps2 = 0;


    bool followSine = false;
    Time toff = Time::Zero;

    const double amp_gain_inv = 10 / 1.35;  // V/A
    const double motor_kt_inv = 1.0 / 0.0146; // A/Nm
    double motorStallTorque    = 0.0197;    // [Nm]
    double gearRatio           = 0.332*25.4*mahi::util::PI/180.0;    // [mm/deg] from spool pitch diameter (.332") and capstan radius if applicable, converted to mm

    bool   has_velocity_limit_ = 1;
    bool   has_force_limit_   = 1;
    bool   has_torque_limit_   = 1;
    double velocityMax         = 100; // [mm/s] ????
    double forceMax           = 20; // [N]
    double torqueMax          = 1; //[Nm]

    ScrollingBuffer fdata1, fdata2, fdata3;
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



