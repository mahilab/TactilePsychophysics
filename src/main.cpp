#include <Mahi/Gui.hpp>

using namespace mahi::gui;

class MyApp : public Application {
public:
    MyApp() : Application(720, 480, "Janelle's Application") { }

    void update() override {
       ImGui::Begin("My Window");       
       ImGui::ShowDemoWindow();
       ImGui::End();
    }

    float x = 1;

};

int main(int argc, char const *argv[])
{
    MyApp app;
    app.run();
    return 0;
}