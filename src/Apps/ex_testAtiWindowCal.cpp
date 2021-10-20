#include <iostream>
#include <Mahi/Daq.hpp>
#include <Mahi/Gui.hpp>
#include <Mahi/Robo.hpp>
#include <thread>
#include <mutex>
#include <Mahi/Util.hpp>
#include <Mahi/Robo/Mechatronics/AtiSensor.hpp>
#include "Util/ATI_windowCal.hpp"

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
     MyGui(): Application(1000, 1000, "MyGui")
    {
        q8.enable();
        ati.load_calibration("FT06833.cal");
        ati.set_channels(&q8.AI[0], &q8.AI[1], &q8.AI[2], &q8.AI[3], &q8.AI[4], &q8.AI[5]);
        atiWin.load_calibration("FT06833.cal");
        atiWin.set_channels(&q8.AI[0], &q8.AI[1], &q8.AI[2], &q8.AI[3], &q8.AI[4], &q8.AI[5]);
        
        q8.read_all();
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

            if (ImGui::Button("Zero Ati")){
                    ati.zero();
            }

            if (ImGui::Button("Zero Ati by Window")){
                    atiWin.zero();
            }
        }

        t += ImGui::GetIO().DeltaTime;
        
        atiFData.AddPoint(t, m_atiForceX* 1.0f);
        winFData.AddPoint(t, m_winForceX* 1.0f);
        atiTData.AddPoint(t, m_atiTorqueX* 1.0f);
        winTData.AddPoint(t, m_winTorqueX* 1.0f);

        ImGui::SliderFloat("History",&history,1,30,"%.1f s");

        static ImPlotAxisFlags ft_axis = ImPlotAxisFlags_NoTickLabels;
        ImPlot::SetNextPlotLimitsX(t - history, t, ImGuiCond_Always);
        if (ImPlot::BeginPlot("##Scrolling", NULL, NULL, ImVec2(-1,450), 0, 0, 0)) {
            ImPlot::PlotLine("Ati Data", &atiFData.Data[0].x, &atiFData.Data[0].y, atiFData.Data.size(), atiFData.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Window Data", &winFData.Data[0].x, &winFData.Data[0].y, winFData.Data.size(), winFData.Offset, 2*sizeof(float));
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
                m_atiForceX = ati.get_force(Axis::AxisX);
                m_atiTorqueX = ati.get_torque(Axis::AxisX);
                m_winForceX = atiWin.get_force(Axis::AxisX);
                m_winTorqueX = atiWin.get_torque(Axis::AxisX);
            }

            t = timer.wait();
        }
    }

 bool open = true;

    Q8Usb q8;
    AtiSensor ati;
    AtiWindowCal atiWin;

    double m_atiForceX = 0;
    double m_atiTorqueX = 0;
    double m_winForceX = 0;
    double m_winTorqueX = 0;

    ScrollingBuffer atiFData, winFData, atiTData, winTData;
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



