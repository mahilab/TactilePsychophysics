#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>
#include <stb_image.h>
#include <syntacts>
#include <algorithm>
#include <random>
#include "CapstanModule.hpp"
#include "CMHub.hpp"

using namespace mahi::gui;
using namespace mahi::util;

bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int* out_width, int* out_height);

class IdentificationGui : public Application {
public:

    enum Mode       { PositionControlExp, ForceControlExp };
    enum Gender     { NoGender, Male, Female, Other };
    enum Handedness { Left, Right };

    IdentificationGui(int subject) : Application(500,500,"CM Perceptual Study (Subject " + std::to_string(subject) + ")" ,false), m_subject(subject)
    { 
        m_hub.createDevice(1, 0, 0, 0, 6, 0);
        m_cm = m_hub.getDevice(1);

        std::string calibration_file = "/cm2/calibs/subject_" + std::to_string(m_subject) + ".json";
        m_cm->importParams(calibration_file);

        auto p = m_cm->getParams();
        m_forceMin = p.forceMin;
        m_forceMax = p.forceMax;

        m_hub.start();
        m_cm->setControlMode(CM::Force);
        m_cm->setControlValue(0);
        m_cm->enable();

        /*m_session.open("MOTU Pro Audio", tact::API::ASIO);

        ImGui::DisableViewports();
        int image_width, image_height;
        LoadTextureFromFile("silo_white.png", &m_image_texture, &image_width, &image_height);

        m_vt_positions.push_back({500-122,244});
        m_vt_positions.push_back({500-86,352});
        m_vt_positions.push_back({500-184,429});
        m_vt_positions.push_back({184,429});
        m_vt_positions.push_back({86,352});
        m_vt_positions.push_back({122,244});

        set_frame_limit(90_Hz);*/
    }

    ~IdentificationGui() {
        m_hub.stop();
        //m_session.close();
    }

    void update() override {

        ImGui::BeginFixed("##MainWindow", {0,0},{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::BeginDisabled(m_mode != Mode::Idle);

        // row 1
        // ImGui::SetNextItemWidth(100);
        // ImGui::InputInt("Subject  ", &m_subject);
        // ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        ImGui::InputInt("Age     ", &m_age);
        ImGui::SameLine();
        if (ImGui::RadioButton("Male  ", m_sex == Male))
            m_sex = Male;
        ImGui::SameLine();
        if (ImGui::RadioButton("Female     ", m_sex == Female))
            m_sex = Female;
        ImGui::SameLine();
        if (ImGui::RadioButton("Left  ",m_hand == Left))
            m_hand = Left;
        ImGui::SameLine();
        if (ImGui::RadioButton("Right",m_hand == Right))
            m_hand = Right;
        
        ImGui::Separator();

        // row 2
        ImGui::SetNextItemWidth(100);
        ImGui::InputInt("Width  ",&m_width);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        ImGui::InputInt("Height  ",&m_height,1,100,ImGuiInputTextFlags_NoHorizontalScroll);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        ImGui::InputInt("Circum.  ",&m_circum,1,250,ImGuiInputTextFlags_NoHorizontalScroll);

        ImGui::Separator();

        // studies
        if (ImGui::Button("Position Control Study",ImVec2(-1,0))) {
            buildPosTrials();
            start_coroutine(posStudy());
        }
        if (ImGui::Button("Force Control Study",ImVec2(-1,0))) {
            buildForceTrials();
            start_coroutine(forceStudy());
        }
        ImGui::EndDisabled();

        /*// cm image
        if (m_mode != Mode::Squeeze)
            ImGui::GetWindowDrawList()->AddImage((void*)(intptr_t)m_image_texture, 
                                                 ImVec2(35,85), ImVec2(35+430, 85+430), ImVec2(0,0), ImVec2(1,1), 
                                                 ImGui::ColorConvertFloat4ToU32(ImGui::GetStyleColorVec4(ImGuiCol_FrameBg)));
*/
        /*// idle mode
        if (m_mode == Mode::PositionControlExp) {
            for (int i = 0; i < 6; ++i) {
                if (circleButton(m_vt_positions[i]))
                    m_session.play(i, tact::Sine(170) * tact::Envelope(0.1));
            }
        }*/

        ImGui::End();     
    }

    bool circleButton(const ImVec2& pos) {
        static float r = 20;
        Vec2 mouse = ImGui::GetMousePos();
        bool hovered = magnitude(pos - mouse) < r;
        bool clicked = hovered && ImGui::IsMouseClicked(0);
        auto dl = ImGui::GetForegroundDrawList();
        dl->AddCircleFilled(pos, r, ImGui::GetColorU32(clicked ? Blues::DodgerBlue : (hovered ? Blues::DeepSkyBlue : Blues::DodgerBlue)), 50);
        return clicked;
    }

    // Vibration Identification Study
    static constexpr int n_id_force_levels  = 3;
    static constexpr int n_id_position_levels     = 2;
    //static constexpr int n_id_vts           = 6;
    static constexpr int n_id_repetitions   = 5;
    static constexpr int n_id_windows       = 4;
    static constexpr int n_id_trials_window = n_id_position_levels * n_id_repetitions; //* n_id_vts;
    static constexpr int n_id_trials_total  = n_id_windows * n_id_trials_window;

    double m_id_force_levels[n_id_force_levels] = {0.5, 5, 10};   // 
    double m_id_position_levels[n_id_position_levels]       = {0.05, 0.25};   // 50 ms, 250 ms

    struct IdTrial {
        int num;
        int gen_num;
        int window;
        double force;
        double position;
        //int vt_stimulus;
        //int vt_response;
    };

    std::vector<std::vector<IdTrial>> m_id_trials;

    void buildPosTrials() {
        std::random_device rd;
        std::mt19937 g(rd());
        g.seed(m_subject);
        // m_id_trials.reserve(n_id_trials_total);
        int i = 0;
        for (int f = 0; f < n_id_force_levels; ++f) {
            std::vector<IdTrial> force_trials;
            force_trials.reserve(n_id_trials_total);
            for (int w = 0; w < n_id_windows; ++w) {
                std::vector<IdTrial> window_trials;
                window_trials.reserve(n_id_trials_window);
                for (int d = 0; d < n_id_position_levels; ++d) {
                    //for (int v = 0; v < n_id_vts; ++v) {
                        for (int r = 0; r < n_id_repetitions; ++r) {
                            IdTrial trial;
                            trial.gen_num     = i++;
                            trial.window      = w;
                            //trial.vt_stimulus = v;
                            //trial.vt_response = -1;
                            trial.force       = m_id_force_levels[f];
                            trial.position    = m_id_position_levels[d];
                            window_trials.push_back(trial);
                        }
                    //}
                }            
                // shuffle window trials
                std::shuffle(window_trials.begin(), window_trials.end(), g);
                // append
                force_trials.insert(force_trials.end(), window_trials.begin(), window_trials.end());
            }
            m_id_trials.push_back(force_trials);
        }        
        // assign trial numbrs
        i = 0;
        for (int f = 0; f < n_id_force_levels; ++f) {
            for (auto& trial : m_id_trials[f])
                trial.num = i++;
        }
    };

    std::vector<int> getOrder(int s) {
        static std::vector<std::vector<int>> orders {
            {0,1,2},
            {0,2,1},
            {1,0,2},
            {1,2,0},
            {2,1,0},
            {2,0,1}
        };
        return orders[s%6];
    }

    Enumerator posStudy() {
        m_mode = Mode::PositionControlExp;
        Timestamp ts;
        std::string filename = "/cm2/data/id_subject_" + std::to_string(m_subject) + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
        Csv csv(filename);
        csv.write_row("Trial","Gen","Force","Window","Position","Stimulus","Response");
        auto order = getOrder(m_subject);
        for (int i = 0; i < order.size(); ++i) {
            set_window_title("CM Perceptual Study (Subject " + std::to_string(m_subject) + ") (V" + std::to_string(i+1) + ")");
            auto f = order[i];
            // set force for this force block
            double force = m_id_trials[f][0].force;
            setForce(force);
            co_yield yield_time(5_s); 
            m_cm->disable();                       
            for (auto& trial : m_id_trials[f]) {
                int response = -1;
                tact::Signal stimulus = tact::Sine(170) * tact::Envelope(trial.position);
                m_session.play(trial.vt_stimulus, stimulus);
                co_yield yield_time(1_s);
                while (response == -1) {
                    for (int v = 0; v < 6; ++v) {
                        if (circleButton(m_vt_positions[v]))
                            response = v;
                    }
                    co_yield nullptr;
                }
                trial.vt_response = response;
                csv.write_row(trial.num, trial.gen_num, trial.force, trial.window, trial.position, trial.vt_stimulus, trial.vt_response);
                co_yield yield_time(1_s);
            }
            // give participant a 1 min break
            m_cm->enable();
            setForce(0.5);
            if (i < (order.size() - 1)) {
                double remaining = 60;
                while (remaining > 0) {
                    ImGui::BeginFixed("##MainWindow", {0,0},{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
                    ImGui::Text("Break (%.3f)", remaining);
                    ImGui::End();
                    remaining -= delta_time().as_seconds();
                    co_yield nullptr;
                } 
            }
        }
        m_mode = Mode::Idle;
        set_window_title("CM Perceptual Study (Subject " + std::to_string(m_subject) + ")");
    }

    void saveIdStudy() {
        Timestamp ts;
        std::string filename = "/cm2/data/id_subject_" + std::to_string(m_subject) + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
        Csv csv(filename);
        csv.write_row("Trial","Gen","Force","Window","Position","Stimulus","Response");
        auto order = getOrder(m_subject);
        for (auto f : order) {
            for (auto& trial : m_id_trials[f])
                csv.write_row(trial.num, trial.gen_num, trial.force, trial.window, trial.position, trial.vt_stimulus, trial.vt_response);
        }
    }

    // Squeeze Just Noticable Difference Study
    static constexpr int n_jnd_force_levels  = 11;
    static constexpr int n_jnd_windows       = 5;
    static constexpr int n_jnd_repetitions   = 5; // total = 30 -> 15 reference first, 15 reference second
    static constexpr int n_jnd_trials_window = n_jnd_force_levels * n_jnd_repetitions * 2;
    static constexpr int n_jnd_trials_total  = n_jnd_trials_window * n_jnd_windows;

    double m_jnd_force_levels[n_jnd_force_levels] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    double m_jnd_force_reference                  = 7;
    double m_jnd_force_min                        = 0.5;
    double m_jnd_stim_time                        = 0.33;

    struct JndTrial {
        int num;
        int gen_num;
        int window;
        int level;
        double squeeze1;
        double squeeze2;
        int standard;   // 1 or 2
        int comparison; // 1 or 2
        int correct;     // 0 = same, 1 = first, 2 = second
        int answer;     // 1 or 2
        int greater;    // 0 or 1
    };

    std::vector<std::vector<JndTrial>> m_jnd_trials;

    void buildForceTrials() {
        std::random_device rd;
        std::mt19937 g(rd());
        g.seed(m_subject);
        int i = 0;
        for (int w = 0; w < n_jnd_windows; ++w) {
            std::vector<JndTrial> window_trials;
            window_trials.reserve(n_jnd_trials_window);
            for (int r = 0; r < n_jnd_repetitions; ++r) {
                for (int f = 0; f < n_jnd_force_levels; ++f) {
                    JndTrial trial;
                    //  standard frist
                    trial.gen_num     = i++;
                    trial.window      = w;
                    trial.level       = f-5;
                    trial.squeeze1    = m_jnd_force_reference;
                    trial.squeeze2    = m_jnd_force_levels[f];
                    trial.standard   = 1;
                    trial.comparison = 2;
                    trial.correct    = trial.squeeze1 == trial.squeeze2 ? 0 : trial.squeeze1 > trial.squeeze2 ? 1 : 2;
                    trial.answer     = -1;
                    trial.greater     = -1;
                    window_trials.push_back(trial);
                    // standard second
                    trial.gen_num     = i++;
                    trial.window      = w;
                    trial.level       = f-5;
                    trial.squeeze1    = m_jnd_force_levels[f];
                    trial.squeeze2    = m_jnd_force_reference;
                    trial.standard   = 2;
                    trial.comparison = 1;
                    trial.correct    =  trial.squeeze1 == trial.squeeze2 ? 0 : trial.squeeze1 > trial.squeeze2 ? 1 : 2;
                    trial.answer     = -1;
                    trial.greater    = -1;
                    window_trials.push_back(trial);
                }
            }
            // shuffle window trials
            std::shuffle(window_trials.begin(), window_trials.end(), g);
            // append
            m_jnd_trials.push_back(window_trials);
        }
        // assign trial numbrs
        i = 0;
        for (int w = 0; w < n_jnd_windows; ++w) {
            for (auto& trial : m_jnd_trials[w])
                trial.num = i++;
        }
    }

    void jndWindow(int which) {
        ImGui::BeginFixed("##MainWindow", {0,0},{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::BeginDisabled(true,1.0f);
        ImGui::Button("Which Squeeze Was Greater?", ImVec2(-1,0));
        if (which == 1)
            ImGui::PushStyleColor(ImGuiCol_Button, Blues::DeepSkyBlue);
        ImGui::Button("First", ImVec2(240,-1));
        if (which == 1)
            ImGui::PopStyleColor();
        ImGui::SameLine();
        if (which == 2)
            ImGui::PushStyleColor(ImGuiCol_Button, Blues::DeepSkyBlue);
        ImGui::Button("Second", ImVec2(240,-1));   
        if (which == 2)
            ImGui::PopStyleColor();
        ImGui::EndDisabled();
        ImGui::End();
    }

    Enumerator forceStudy() {
        m_mode = Mode::Squeeze;
        Timestamp ts;
        std::string filename = "/cm2/data/jnd_subject_" + std::to_string(m_subject) + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
        Csv csv(filename);
        csv.write_row("Trial","Gen","Window","Standard","Comparison","Level","Squeeze1","Squeeze2","Correct","Answer","Greater");
        double elapsed;
        for (int w = 0; w < n_jnd_windows; ++w) {
            set_window_title("CM Perceptual Study (Subject " + std::to_string(m_subject) + ") (S" + std::to_string(w+1) + ")");
            // second delay
            elapsed = 0;
            while (elapsed < 2) {
                jndWindow(0);
                elapsed += delta_time().as_seconds();
                co_yield nullptr;
            }
            for (auto& trial : m_jnd_trials[w]) {
                // render first squeeze
                elapsed = 0;
                while (elapsed < m_jnd_stim_time) {
                    double force = Tween::Linear(m_jnd_force_min, trial.squeeze1, (float)(elapsed / m_jnd_stim_time));
                    setForce(force);
                    jndWindow(1);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }  
                setForce(trial.squeeze1);
                elapsed = 0;
                while (elapsed < m_jnd_stim_time) {
                    jndWindow(1);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                elapsed = 0;
                while (elapsed < m_jnd_stim_time) {
                    double force = Tween::Linear(trial.squeeze1, m_jnd_force_min, (float)(elapsed / m_jnd_stim_time));
                    setForce(force);
                    jndWindow(1);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                setForce(m_jnd_force_min);
                // delay
                elapsed = 0;
                while (elapsed < 0.25) {
                    jndWindow(0);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                // render second squeeze
                elapsed = 0;
                while (elapsed < m_jnd_stim_time) {
                    double force = Tween::Linear(m_jnd_force_min, trial.squeeze2, (float)(elapsed / m_jnd_stim_time));
                    setForce(force);
                    jndWindow(2);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }  
                setForce(trial.squeeze2);
                elapsed = 0;
                while (elapsed < m_jnd_stim_time) {
                    jndWindow(2);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
                elapsed = 0;
                while (elapsed < m_jnd_stim_time) {
                    double force = Tween::Linear(trial.squeeze2, m_jnd_force_min, (float)(elapsed / m_jnd_stim_time));
                    setForce(force);
                    jndWindow(2);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                } 
                setForce(m_jnd_force_min);

                // collect response
                while (true) {
                    int answer = -1;
                    ImGui::BeginFixed("##MainWindow", {0,0},{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
                    ImGui::BeginDisabled(true,1.0f);
                    ImGui::Button("Which Squeeze Was Harder?", ImVec2(-1,0));
                    ImGui::EndDisabled();
                    if (ImGui::Button("First", ImVec2(240,-1)))
                        answer = 1;
                    ImGui::SameLine();
                    if (ImGui::Button("Second", ImVec2(240,-1)))
                        answer = 2;
                    ImGui::End();
                    if (answer != -1) {
                        trial.answer = answer;
                        trial.greater = trial.comparison == trial.answer ? 1 : 0;
                        csv.write_row(trial.num, trial.gen_num, trial.window, trial.standard, trial.comparison, trial.level, trial.squeeze1, trial.squeeze2, trial.correct, trial.answer, trial.greater);
                        break;
                    }
                    co_yield nullptr;
                }
                // second delay
                elapsed = 0;
                while (elapsed < 1) {
                    jndWindow(0);
                    elapsed += delta_time().as_seconds();
                    co_yield nullptr;
                }
            }            
            // break
            if (w < (n_jnd_windows - 1)) {
                double remaining = 60;
                while (remaining > 0) {
                    ImGui::BeginFixed("##MainWindow", {0,0},{500,500}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
                    ImGui::Text("Break (%.3f)", remaining);
                    ImGui::End();
                    remaining -= delta_time().as_seconds();
                    co_yield nullptr;
                } 
            }
        }
        m_mode = Mode::Idle;
        set_window_title("CM Perceptual Study (Subject " + std::to_string(m_subject) + ")");
    }

    void saveJndStudy() {
        Timestamp ts;
        std::string filename = "/cm2/data/jnd_subject_" + std::to_string(m_subject) + "_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
        Csv csv(filename);
        csv.write_row("Trial","Gen","Window","Standard","Comparison","Level","Squeeze1","Squeeze2","Correct","Answer","Greater");
        for (int w = 0; w < n_jnd_windows; ++w) {
            for (auto& trial : m_jnd_trials[w])
                csv.write_row(trial.num, trial.gen_num, trial.window, trial.standard, trial.comparison, trial.level, trial.squeeze1, trial.squeeze2, trial.correct, trial.answer, trial.greater);
        }
    }

    void setForce(double N) {
        double cv = remap(N, m_forceMin, m_forceMax, 0.0, 1.0);
        m_cm->setControlValue(cv);
    }

    // Subject
    int m_subject     = 0; 
    Gender m_sex      = NoGender;  
    Handedness m_hand = Left;
    int m_age         = 20;           
    int m_width       = 60;
    int m_height      = 60;
    int m_circum      = 190;

    double m_forceMin;
    double m_forceMax;

    // Status
    Mode m_mode       = Mode::Idle;

    // CM
    CMHub m_hub;
    std::shared_ptr<CM> m_cm;  

    // Syntacts
    tact::Session m_session;

    // CM Image
    GLuint m_image_texture = 0;
    std::vector<ImVec2> m_vt_positions;  
};

int main(int argc, char *argv[])
{

    Options options("identification.exe","CM Perceptual Study");
    options.add_options()
        ("s,subject","Subject ID Number",value<int>());
    
    auto result = options.parse(argc, argv);

    if (result.count("s")) {
        IdentificationGui gui(result["s"].as<int>());
        gui.run();
    }
    else {
        fmt::print("{}",options.help());
    }

    return 0;
}


///////////////////////////////////////////////////////////////////////////////

/*bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int* out_width, int* out_height)
{
    // Load from file
    int image_width = 0;
    int image_height = 0;
    unsigned char* image_data = stbi_load(filename, &image_width, &image_height, NULL, 4);
    if (image_data == NULL)
        return false;

    // Create a OpenGL texture identifier
    GLuint image_texture;
    glGenTextures(1, &image_texture);
    glBindTexture(GL_TEXTURE_2D, image_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Upload pixels into texture
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
    stbi_image_free(image_data);

    *out_texture = image_texture;
    *out_width = image_width;
    *out_height = image_height;

    return true;
}*/