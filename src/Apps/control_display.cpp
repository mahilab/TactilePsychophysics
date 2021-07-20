#include <Mahi/Util.hpp>
#include <Mahi/Gui.hpp>

using namespace mahi::gui;
using namespace mahi::util;

#define DRAW ImGui::GetForegroundDrawList()

// utility structure for realtime plot
struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer() {
        MaxSize = 2000;
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

struct Button {
    void draw() {
        DRAW->AddRectFilled(pos - ImVec2(width/2,0), pos + ImVec2(width/2,height), ImGui::GetColorU32(Grays::Gray50));
        DRAW->AddRect(ImVec2(250-50, 250 + height), ImVec2(250+50, 250+100), ImGui::GetColorU32(Grays::Gray50));
        DRAW->AddLine(ImVec2(350,250), ImVec2(350,350), ImGui::GetColorU32(Grays::Gray50));
        DRAW->AddLine(ImVec2(350,250), ImVec2(340,250), ImGui::GetColorU32(Grays::Gray50));

    }

    Vec2 pos = {250,250};
    Vec2 size = {100,200};
    float width = 50;
    float height = 25;
};

void DrawSpring(float y1, float y2, float x, float w, int n, ImU32 col) {
    float dir = 1;
    float ystep = (y2 - y1) / n;
    float y = y1;
    for (int i = 0; i < n; ++i) {
        ImVec2 p1(x - dir * w/2, y);
        y = y + ystep;
        ImVec2 p2(x + dir * w/2, y);
        dir *= -1;
        DRAW->AddLine(p1, p2,col);
    }
}

struct Finger {
    void update() {
        pos = ImGui::GetMousePos();
    }
    void draw(bool showLine, float x) {
        DRAW->AddCircleFilled(pos, radius, ImGui::GetColorU32(col), 20);
        if (showLine) {
            static char buff[16];
            sprintf(buff, "%.3f", x);
            DRAW->AddLine(ImVec2(350, pos.y+radius), ImVec2(340, pos.y+radius), ImGui::GetColorU32(col));
            DRAW->AddText(ImVec2(355, pos.y+radius - ImGui::CalcTextSize(buff).y * 0.5f), ImGui::GetColorU32(col), buff);
        }
    }
    Vec2 pos;
    float radius = 5.0f;
    bool colliding = false;
    Color col;
};

class ControlDisplay : public Application {
public:

    float CD = 2;
    float Kv = 30;
    float M = 0.06f;

    bool criticallyDamp = true;
    float B = 1;
    float F = 0;
    float Kb = 30;

    float x = 0;
    float xd = 0;
    float xdd = 0; 

    float xv;  

    bool drawSprings = false;
    bool fixedPlane  = false;

    ControlDisplay() : Application(500,500,"C/D Playground") { 
        ImGui::DisableViewports();
        finger.col = Browns::Bisque;
        proxy.col  = Blues::DeepSkyBlue;
        set_vsync(false);
        set_frame_limit(1000_Hz);
    }

    void update() {

        auto t = time();

        xv = 0;
        if (proxy.pos.x > 225 && proxy.pos.x < 275) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_None);
            if (fixedPlane)
                xv = clamp(proxy.pos.y + proxy.radius - 250, 0.0f, 1000.0f) * 0.001f;
            else
                xv = clamp(proxy.pos.y + proxy.radius - button.pos.y, 0.0f, 1000.0f) * 0.001f;
        }

        if (fixedPlane)
            Kb = (CD) * Kv;
        else
            Kb = (CD - 1) * Kv;

        F = Kv * xv;
        if (criticallyDamp)
            B = 2*sqrt(Kb*M);

        xdd = (F - Kb*x - B*xd) / M;
        xd  = (float)xdd_xd.update(xdd, t);       
        x   = (float)xd_x.update(xd, t);

        button.pos.y = 250.0f + x*1000;
        proxy.pos = ImGui::GetMousePos();
        
        if (fixedPlane)
            finger.pos = xv > 0 ? ImVec2(proxy.pos.x, button.pos.y - finger.radius) : proxy.pos;
        else
            finger.pos = proxy.pos - ImVec2(0,xv*1000);

        float cd = (x + xv) / x;

        ImGui::BeginFixed("Simulation", ImVec2(5,5), ImVec2(150,490), ImGuiWindowFlags_NoTitleBar);


        ImGui::DragFloat("C/D",&CD,0.1f,1,10);
        ImGui::DragFloat("M",&M,0.01f,0,1);
        ImGui::DragFloat("Kv",&Kv,1,0,100);

        ImGui::BeginDisabled(criticallyDamp);
        ImGui::DragFloat("B",&B,0.001f,0,10);
        ImGui::EndDisabled();
        ImGui::Checkbox("Critically Damp", &criticallyDamp);
        ImGui::Checkbox("Fixed Plane", &fixedPlane);
        ImGui::Checkbox("Show Control",&showControl);
        ImGui::Checkbox("Draw Springs", &drawSprings);
        if (ImGui::Button("Reset Simulation", ImVec2(-1,0))) {
            xdd = 0;
            xd  = 0;
            x   = 0;
            xdd_xd.reset();
            xd_x.reset();
        }

        ImGui::Separator();
        ImGui::Value("Fs",ImGui::GetIO().Framerate);
        ImGui::Value("Kb",Kb);
        ImGui::Value("F",F);
        ImGui::Value("x",x);
        ImGui::Value("xd",xd);
        ImGui::Value("xdd",xdd);
        ImGui::Value("xv",xv);
        ImGui::Value("C/D Actual",cd);

        static ScrollingBuffer buff;
        buff.AddPoint((float)t.as_seconds(), isnan(cd) ? 1 : cd);

        ImPlot::PushStyleVar(ImPlotStyleVar_PlotMinSize, ImVec2(0,0));
        ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(0,0));
        ImPlot::SetNextPlotLimits(t.as_seconds()-2, t.as_seconds(), 0, 10, ImGuiCond_Always);
        if (ImPlot::BeginPlot("##CD",0,0,ImVec2(-1,100), 0, ImPlotAxisFlags_NoDecorations, ImPlotAxisFlags_NoDecorations)) {
            ImPlot::PlotLine("##CD", &buff.Data[0].x, &buff.Data[0].y, buff.Data.Size, buff.Offset, sizeof(ImVec2));
            ImPlot::EndPlot();
        }
        ImPlot::PopStyleVar(2);

        // ImGui::Separator();
        // ImGui::Text("Mouse: %.0f,%.0f", ImGui::GetMousePos().x, ImGui::GetMousePos().y);
        ImGui::End();
    }

    void draw() override {
        button.draw();
        if (drawSprings)
            DrawSpring(button.pos.y + button.height, 350, button.pos.x, 10, 10, ImGui::GetColorU32(Grays::Gray50));
        if (showControl) {
            proxy.draw(xv > 0.001, x + xv);
            if (drawSprings) {
                if (fixedPlane && xv > 0)
                    DrawSpring(250, proxy.pos.y, finger.pos.x, 10, 10, ImGui::GetColorU32(proxy.col));
                else
                    DrawSpring(finger.pos.y, proxy.pos.y, finger.pos.x, 10, 10, ImGui::GetColorU32(proxy.col));
            }
        }
        finger.draw(xv > 0.001, x);
    }

    bool showControl = true;

    Finger finger, proxy;
    Button button;

    Integrator xdd_xd;
    Integrator xd_x;
};

int main(int argc, char const *argv[])
{
    ControlDisplay app;
    app.run();
    return 0;
}
