#include "Vsb.hpp"

struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImPlotPoint> Data;
    ScrollingBuffer() {
        MaxSize = 2000;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(double x, double y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImPlotPoint(x,y));
        else {
            Data[Offset] = ImPlotPoint(x,y);
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

class VsbApp : public Application {
public:
    VsbApp() : 
        Application(500,750,"VSB GUI"),
        m_q8(true),
        m_vsb("vsb", 2000),
        m_ati(&m_q8.AI[0], &m_q8.AI[1], &m_q8.AI[2], &m_q8.AI[3], &m_q8.AI[4], &m_q8.AI[5], "FT26062.cal") // FT26062, FT19612, FT06833
    {
        MahiLogger->set_max_severity(Verbose);
        ImGui::DisableViewports();
        set_vsync(false);
    }

    ~VsbApp() {    }

    void update() override {
        auto [w,h] = get_window_size();
        ImGui::BeginFixed("VSB GUI", ImVec2(0,0), ImVec2(w,h), ImGuiWindowFlags_NoTitleBar);
        VsbGui::ShowControls(m_vsb);
        ImGui::Separator();
        auto q = m_vsb.query();    
        auto x = m_vsb.getDisplacement();   
        VsbGui::ShowQuery(q);
        ImGui::Value("Displacement:", (float)x);
        ImGui::Separator();

        if (ImGui::Button("Zero ATI", ImVec2(-1,0)))
            m_ati.zero();

        double t = time().as_seconds();

        static ScrollingBuffer amp_data1;
        static ScrollingBuffer amp_data2;
        static ScrollingBuffer frc_data1;
        static ScrollingBuffer frc_data2;
        static ScrollingBuffer ati_data;
        static ScrollingBuffer fsc_data;

        amp_data1.AddPoint(t, q.ampsSense);
        amp_data2.AddPoint(t, q.ampsCommand);

        frc_data1.AddPoint(t, q.linearForceSense + 0.06 * mahi::util::G);
        frc_data2.AddPoint(t, q.linearForceCommand + 0.06 * mahi::util::G);

        auto mode = m_vsb.getMode();

        m_q8.read_all();
        ati_data.AddPoint(t, mode == Vsb::Calibrator ? -1 * m_ati.get_force(Axis::AxisZ) : m_ati.get_force(Axis::AxisZ));
        fsc_data.AddPoint(t, m_q8.AI[6]);

        ImPlot::SetNextPlotLimitsX(t - 10, t, ImGuiCond_Always);
        if (ImPlot::BeginPlot("Data",NULL,NULL,ImVec2(-1,300),ImPlotFlags_YAxis2)) {
            ImPlot::SetPlotYAxis(0);
            ImPlot::PlotLine("Force (Sense)", &frc_data1.Data[0].x, &frc_data1.Data[0].y, frc_data1.Data.Size, frc_data1.Offset, 2*sizeof(double));
            ImPlot::PlotLine("Force (Command)", &frc_data2.Data[0].x, &frc_data2.Data[0].y, frc_data2.Data.Size, frc_data2.Offset, 2*sizeof(double));
            ImPlot::PlotLine("Force (ATI)", &ati_data.Data[0].x, &ati_data.Data[0].y, ati_data.Data.Size, ati_data.Offset, 2*sizeof(double));
            ImPlot::SetPlotYAxis(1);
            ImPlot::PlotLine("SingleTact", &fsc_data.Data[0].x, &fsc_data.Data[0].y, fsc_data.Data.Size, fsc_data.Offset, 2*sizeof(double));

            // ImPlot::PlotLine("Amps Sense",   &amp_data1.Data[0].x, &amp_data1.Data[0].y, amp_data1.Data.Size, amp_data1.Offset, 2*sizeof(double));
            // ImPlot::PlotLine("Amps Command", &amp_data2.Data[0].x, &amp_data2.Data[0].y, amp_data2.Data.Size, amp_data2.Offset, 2*sizeof(double));
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("SingleTact","Voltage","Force",ImVec2(-1,300))) {
            ImPlot::PlotScatter("Calibration",&fsc_data.Data[0].y,&frc_data1.Data[0].y,fsc_data.Data.Size, fsc_data.Offset, 2*sizeof(double));
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

    Q8Usb m_q8;
    Vsb   m_vsb;
    AtiSensor m_ati;
};

int main(int argc, char const *argv[])
{
    VsbApp gui;
    gui.run();
    return 0;
}
