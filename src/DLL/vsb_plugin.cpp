#include "Vsb.hpp"

#define EXPORT extern "C" __declspec(dllexport)

class VsbApp : public Application {
public:
    VsbApp() : 
        Application(500,250,"VSB GUI"),
        vsb("vsb", 2000)
    {
        MahiLogger->set_max_severity(Verbose);
        ImGui::DisableViewports();
    }

    ~VsbApp() {    }

    void update() override {
        auto [w,h] = get_window_size();
        ImGui::BeginFixed("VSB GUI", ImVec2(0,0), ImVec2(w,h), ImGuiWindowFlags_NoTitleBar);
        VsbGui::ShowControls(vsb);
        ImGui::Separator();
        auto q = vsb.query();     
        VsbGui::ShowQuery(q);
        ImGui::Separator();
    }

    Vsb   vsb;
};

std::mutex g_mtx;
std::shared_ptr<VsbApp> g_app = nullptr;

void app_thread() {
    {
        LOG(Severity::Info) << "Initalizing plugin objects";
        std::lock_guard<std::mutex> lock(g_mtx);
        g_app = std::make_shared<VsbApp>();
        LOG(Severity::Info) << "Initialized plugin objects";
    }
    LOG(Severity::Info) << "Running VSB App";
    g_app->run();
    LOG(Severity::Info) << "Terminating VSB App";
    {
        LOG(Severity::Info) << "Destroying plugin objects";
        std::lock_guard<std::mutex> lock(g_mtx);
        g_app        = nullptr;
        LOG(Severity::Info) << "Destroyed plugin objects";
    }
}

EXPORT bool openVsb() {
    if (g_app == nullptr) {
        auto thrd = std::thread(app_thread);
        thrd.detach();
        return true;
    }
    return false;
}

EXPORT double getDisplacement() {
    std::lock_guard<std::mutex> lock(g_mtx);
    if (g_app != nullptr) 
        return g_app->vsb.getDisplacement();    
    else
        return 0;
}

EXPORT void setStiffness(double kp, bool criticallyDamp) {
    std::lock_guard<std::mutex> lock(g_mtx);
    if (g_app != nullptr)
         g_app->vsb.setStiffness(kp, criticallyDamp);
}

EXPORT void addStiffness(double kp, bool criticallyDamp) {
    std::lock_guard<std::mutex> lock(g_mtx);
    if (g_app != nullptr)
         g_app->vsb.addStiffness(kp, criticallyDamp);
}

EXPORT double getStiffness() {
    std::lock_guard<std::mutex> lock(g_mtx);
    if (g_app != nullptr)
        return g_app->vsb.getStiffness();
    else 
        return -1;
}

EXPORT double getDamping() {
    std::lock_guard<std::mutex> lock(g_mtx);
    if (g_app != nullptr)
        return g_app->vsb.getDamping();
    else 
        return -1;
}