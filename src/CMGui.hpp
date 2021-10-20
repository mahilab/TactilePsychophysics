#pragma once
#include "CapstanModule.hpp"
#include "CMHub.hpp"
#include <mahi/gui.hpp>

namespace CMGui
{

inline bool ShowParamsMenu(CM::Params &p)
{
    double posGains[2] = {p.positionKp * 10e6, p.positionKd * 10e6};
    double frcGains[3] = {p.forceKp * 10e6, p.forceKi * 10e6, p.forceKd * 10e6};
    bool changed = false;
    if (ImGui::DragDouble("Motor Nominal Torque", &p.motorNominalTorque, 0.000001f, 0, 0.01, "%.6f Nm"))
        changed = true;
    if (ImGui::DragDouble("Motor Stall Torque", &p.motorStallTorque, 0.000001f, 0, 0.01, "%.6f Nm"))
        changed = true;
    if (ImGui::DragDouble("Motor Nominal Current", &p.motorNominalCurrent, 0.001f, 0, 1, "%.3f A"))
        changed = true;
    if (ImGui::DragDouble("Motor Stall Current", &p.motorStallCurrent, 0.001f, 0, 1, "%.3f A"))
        changed = true;
    if (ImGui::DragDouble("Motor Torque Constant", &p.motorTorqueConstant, 0.000001f, 0, 1, "%.6f Nm/A"))
        changed = true;
    if (ImGui::DragDouble("Motor Nominal Speed", &p.motorNominalSpeed, 1, 0, 100000, "%.1f deg/s"))
        changed = true;
    if (ImGui::DragDouble("Motor Max Speed", &p.motorMaxSpeed, 1, 0, 100000, "%.1f deg/s"))
        changed = true;
    if (ImGui::DragDouble("Gear Ratio", &p.gearRatio, 0.001f, 0, 1))
        changed = true;
    if (ImGui::DragDouble("Degrees/Count", &p.degPerCount, 0.001f, 0, 10))
        changed = true;
    if (ImGui::DragDouble("Command Gain", &p.commandGain, 0.000001f, 0, 1, "%.6f A/V"))
        changed = true;
    if (ImGui::DragDoubleRange2("Position Range", &p.positionMin, &p.positionMax, 1, -200, 200, "%.1f deg"))
        changed = true;
    if (ImGui::DragDouble2("Position PD", posGains, 0.1f, 0, 1000, "%.3f"))
        changed = true;
    if (ImGui::DragDoubleRange2("Force Range", &p.forceMin, &p.forceMax, 1, -20, 20, "%.1f N"))
        changed = true;
    if (ImGui::DragDouble3("Force PID", frcGains, 0.1f, 0, 1000, "%.3f"))
        changed = true;
    if (ImGui::DragDouble("Force Kff", &p.forceKff, 0.001, -1, 1))
        changed = true;
    if (ImGui::DragScalarN("Force Calibration", ImGuiDataType_Double, &p.forceCalibA, 3, 0.001f, 0, 0, "%.3f", 1.0f))
        changed = true;
    if (ImGui::DragScalarN("Position-Force Calibration", ImGuiDataType_Double, &p.posToFrcCalibA, 3, 0.001f, 0, 0, "%.3f", 1.0f))
        changed = true;
    if (ImGui::DragScalarN("Force-Position Calibration", ImGuiDataType_Double, &p.frcToPosCalibA, 3, 0.001f, 0, 0, "%.3f", 1.0f))
        changed = true;
    if (ImGui::DragDouble("Force Filter Cutoff Wn", &p.forceFilterCutoff, 0.001f, 0, 1))
        changed = true;
    if (ImGui::DragInt("Force Filter Samples", &p.forceFilterN, 1, 3, 1000))
        changed = true;
    if (ImGui::DragDouble("CV Filter Cutoff Wn", &p.cvFilterCutoff, 0.001f, 0, 1))
        changed = true;
    if (ImGui::Checkbox("Filter Control Value", &p.filterControlValue))
        changed = true;
    if (ImGui::DragDouble("Output Cutoff Wn", &p.outputFilterCutoff, 0.001f, 0, 1))
        changed = true;
    if (ImGui::Checkbox("Filter Output Value", &p.filterOutputValue))
        changed = true;
    if (ImGui::DragDouble("Velocity Cutoff Wn", &p.velFilterCutoff, 0.001f, 0, 1))
        changed = true;
    if (ImGui::Checkbox("Use Software Velocity", &p.useSoftwareVelocity))
        changed = true;
    p.positionKp = posGains[0] / 10e6;
    p.positionKd = posGains[1] / 10e6;
    p.forceKp = frcGains[0] / 10e6;
    p.forceKi = frcGains[1] / 10e6;
    p.forceKd = frcGains[2] / 10e6;
    return changed;
}

inline void ShowHubQuery(const CMHub::Query &Q) {
    ImGui::LabelText("Status", Q.status == CMHub::Idle ? "Idle" : Q.status == CMHub::Running ? "Running" : Q.status == CMHub::Error ? "Error" : "?");
    ImGui::LabelText("Devices", "%d", Q.devices);
    ImGui::LabelText("Time", "%.3f s", Q.time);
    ImGui::LabelText("Tick", "%d", Q.tick);
    ImGui::LabelText("Misses", "%d", Q.misses);
    ImGui::LabelText("Miss Rate", "%.3f", Q.missRate);
    ImGui::LabelText("Wait Ratio", "%.3f", Q.waitRatio);
    ImGui::LabelText("Lock Count", "%d", Q.lockCount);
    ImGui::LabelText("Loop Rate", "%.3f Hz", Q.loopRate);
}

inline void ShowCMQuerey(CM::Query &q)
{
    ImGui::LabelText("Status", q.status == CM::Enabled ? "Enabled" : "Disabled");
    ImGui::LabelText("Encoder Position", "%d", q.counts);
    ImGui::LabelText("Encoder Velocity", "%.3f Hz", q.countsPerSecond);
    ImGui::LabelText("Motor Position", "%.3f deg", q.motorPosition);
    ImGui::LabelText("Motor Velocity", "%.3f deg/s", q.motorVelocity);
    ImGui::LabelText("Torque Command", "%.3f Nm", q.motorTorqueCommand);
    ImGui::LabelText("Spool Position", "%.3f deg", q.spoolPosition);
    ImGui::LabelText("Spool Velocity", "%.3f deg/s", q.spoolVelocity);
    ImGui::LabelText("Force (Raw)", "%.3f V", q.forceRaw);
    ImGui::LabelText("Force", "%.3f N", q.force);
    ImGui::LabelText("Control Mode", q.ctrlMode == CM::Torque ? "Torque" : q.ctrlMode == CM::Position ? "Position" : q.ctrlMode == CM::Force ? "Force" : q.ctrlMode == CM::ForceHybrid ? "ForceHybrid" : q.ctrlMode == CM::ForceErr ? "ForceErr" : "?");
    ImGui::LabelText("Control Value", "%.3f", q.ctrlValue);
    ImGui::LabelText("Control Value (Filtered)", "%.3f", q.ctrlValueFiltered);
    ImGui::LabelText("Control Value (Scaled)", "%.3f", q.ctrlValueScaled);
    ImGui::LabelText("Lock Count", "%d", q.lockCount);
    ImGui::LabelText("Feed Rate", "%.3f Hz", q.feedRate);
}

}; // namespace CMGui