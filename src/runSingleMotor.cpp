#include <iostream>
#include "Robot.hpp"

class MyGui : public Application
{
public:
     MyGui(): 
        Application(500, 500, "MyGui")
    {
        q8.enable();
        q8.encoder.units[0] = 2 * PI / (1024 * 35);
        q8.encoder.units[1] = 2 * PI / (1024 * 35);

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

        ImGui::DragDouble("Kp", &kp, 0.01f, 0, 10);
        ImGui::DragDouble("Kd", &kd, 0.001f, 0, 1);

        if (ImGui::Checkbox("Follow Sine", &followSine))
            toff = time();

        if (followSine) {
            x_ref1 = 3 * std::sin(2 * PI * 0.25 * time().as_seconds() - toff.as_seconds());
            x_ref2 = 3 * std::cos(2 * PI * 0.25 * time().as_seconds() - toff.as_seconds());
        }
        else {
            ImGui::DragDouble("X Ref Normal", &x_ref1, 0.1f, -3, 3);
            ImGui::DragDouble("X Ref Shear", &x_ref2, 0.1f, -3, 3);
        }

        double pos1 = q8.encoder.positions[0];
        double vel1 = q8.velocity.velocities[0];
        double torque1 = kp * (x_ref1 - pos1) + kd * (0 - vel1);
        double amps1 = torque1 * motor_kt_inv;
        double volts1 = amps1 * amp_gain_inv;
        q8.AO[0] = volts1;

        double pos2 = q8.encoder.positions[1];
        double vel2 = q8.velocity.velocities[1];
        double torque2 = kp * (x_ref2 - pos2) + kd * (0 - vel2);
        double amps2 = torque2 * motor_kt_inv;
        double volts2 = amps2 * amp_gain_inv;
        q8.AO[1] = volts2;

        //ImGui::DragDouble("Motor Torque", &torque, 0.01f, -0.5, 0.5, "%.3f mNm");


        
        ImGui::PushItemWidth(100);
        ImGui::LabelText("normal motor encoder counts", "%d", q8.encoder[0]);
        ImGui::LabelText("normal motor encoder position", "%f", q8.encoder.positions[0]);
        ImGui::LabelText("normal motor encoder velocity", "%f", q8.velocity.velocities[0]);
        ImGui::LabelText("shear motor encoder counts", "%d", q8.encoder[1]);
        ImGui::LabelText("shear motor encoder position", "%f", q8.encoder.positions[1]);
        ImGui::LabelText("shear motor encoder velocity", "%f", q8.velocity.velocities[1]);
        ImGui::PopItemWidth();

        ImGui::End();
        q8.write_all();
    }

    Q8Usb q8;

    double kp = 1;
    double kd = 0.1;

    //double torque = 0;
    double x_ref1 = 0;
    double x_ref2 = 0;

    bool followSine = false;
    Time toff = Time::Zero;

    const double amp_gain_inv = 10 / 1.35;  // V/A
    const double motor_kt_inv = 1.0 / 14.6; // A/mNm

};

int main(int, char **)
{
    MyGui gui;
    gui.run();
}
