// MIT License
//
// Copyright (c) 2020 Mechatronics and Haptic Interfaces Lab - Rice University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// Author(s): Evan Pezent (epezent@rice.edu)

#define MAHI_GUI_NO_CONSOLE
#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>
#include <fstream>
#include <algorithm>
#include <random>


#if defined(__linux__)
    #include <experimental/filesystem>
    namespace fs = std::experimental::filesystem;
#else
    #include <filesystem>
    namespace fs = std::filesystem;
#endif

using namespace mahi::gui;
using namespace mahi::util;

/// Basic Likert survey application
class Likert : public Application {
public:

    enum Gender { NoGender, Male, Female, Other };

    enum Response {
        NoResponse = -3,
        StronglyDisagree = -2,
        Disagree = -1,
        Neutral = 0,
        Agree = 1,
        StronglyAgree = 2
    };

    /// Constructor
    Likert() : Application(500,500,"",false) { 
        ImGui::DisableViewports();
        std::cout << "constructor before" << std::endl;
        loaded = load();
        std::cout << "constructor after" << std::endl;
        ImGui::StyleColorsMahiDark3();
    }

    /// GUI code
    void update() override {
        ImGui::BeginFixed("##Likert", ImGui::GetMainViewport()->Pos, {width, height}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings);
        if (loaded) {
            // Subject Info
            ImGui::SetNextItemWidth(100);
            if (ImGui::BeginCombo("ID", subject != -1 ? std::to_string(subject).c_str() : "")) {
                    for (int i = 0; i < 100; ++i) {
                    if (ImGui::Selectable(std::to_string(i).c_str(), i == subject))
                        subject = i;
                    if (subject == i)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(50);
            if (ImGui::BeginCombo("Age     ", age != -1 ? std::to_string(age).c_str() : "")) {
                for (int i = 18; i < 100; ++i) {
                    if (ImGui::Selectable(std::to_string(i).c_str(), i == age))
                        age = i;
                    if (age == i)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Male", sex == Male))
                sex = Male;
            ImGui::SameLine();
            if (ImGui::RadioButton("Female", sex == Female))
                sex = Female;
            ImGui::SameLine();
            if (ImGui::RadioButton("Other", sex == Other))
                sex = Other;
            ImGui::SameLine(0, 50);
            ImGui::Text("Wrist:");            
            ImGui::SameLine();
            ImGui::SetNextItemWidth(50);
            ImGui::InputFloat("Width",&wristW);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(50);
            ImGui::InputFloat("Height",&wristH);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(50);
            ImGui::InputFloat("Circumference",&wristC);

            ImGui::SameLine(ImGui::GetWindowWidth() - 105);
            if (ImGui::ButtonColored("Submit", Blues::DodgerBlue, {100,0})) {
                bool result = saveResponse();
                if (result && autoClose)
                    quit();
            }
            // Header
            ImGui::Separator();
            ImGui::Separator();
            ImGui::Text("\nQuestion");
            ImGui::SameLine(qWidth - (continuous ? 0 : 20));
            ImGui::Text("Strongly\nDiagree");
            if (!continuous) {
                ImGui::SameLine(qWidth + 65);
                ImGui::Text("\nDisagree");
                ImGui::SameLine(qWidth + 145);
                ImGui::Text("\nNeutral");
                ImGui::SameLine(qWidth + 230);
                ImGui::Text("\nAgree");
            }
            ImGui::SameLine(qWidth + (continuous ? 320 : 305));
            ImGui::Text("Strongly\n   Agree");
            // render questions
            float initialY = ImGui::GetCursorPos().y;
            for (unsigned int i = 0; i < questions.size(); ++i) {

                int q = order[i];

                ImGui::PushID(i);
                ImGui::SetCursorPosY(initialY + rowHeight * i);
                float ly = ImGui::GetCursorPosY();
                auto [w,h] = get_window_size();
                Rect rect(0,ly,w,rowHeight);
                bool hovered = rect.contains(ImGui::GetMousePos());
                if(hovered) {
                    // ImGui::PushStyleColor(ImGuiCol_Text, Blues::DeepSkyBlue);
                    // ImGui::PushStyleColor(ImGuiCol_FrameBg, Blues::DeepSkyBlue);
                    ImGui::GetForegroundDrawList()->AddRectFilled(rect.tl(), rect.br(), IM_COL32(0,0,0,64));
                }

                ImGui::Separator();
                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.5);
                ImGui::Text("[Q.%02d]",i+1); 
                ImGui::PopStyleVar();
                ImGui::SameLine(); 
                ImGui::TextUnformatted(questions[q].c_str());
                ImGui::SameLine(qWidth);

                if (continuous) {
                    ImGui::SetNextItemWidth(375);
                    ImGui::SliderFloat("##Slider",&responsesC[q], 0, 1, "");
                }
                else  {
                    if (ImGui::RadioButton("##SD",responses[q] == StronglyDisagree))
                        responses[q] = StronglyDisagree;
                    ImGui::SameLine(qWidth+80);
                    if (ImGui::RadioButton("##D",responses[q] == Disagree))
                        responses[q] = Disagree;
                    ImGui::SameLine(qWidth+160);
                    if (ImGui::RadioButton("##N",responses[q] == Neutral))
                        responses[q] = Neutral;
                    ImGui::SameLine(qWidth+240);
                    if (ImGui::RadioButton("##A",responses[q] == Agree))
                        responses[q] = Agree;
                    ImGui::SameLine(qWidth+320);
                    if (ImGui::RadioButton("##SA",responses[q] == StronglyAgree))
                        responses[q] = StronglyAgree;
                }
                ImGui::PopID();

                // if (hovered)
                //     ImGui::PopStyleColor(2);
            }
            // begin message modal if opened this frame
            bool dummy = true;
            if (ImGui::BeginPopupModal("Message", &dummy, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::TextUnformatted(message.c_str());
                ImGui::EndPopup();
            }
        }
        else {
            ImGui::Text("Likert survey failed to load! :(");
        }
        ImGui::End();
    }

    /// Load in Likert config file
    bool load() {
        if (fs::exists("survey_config.json")) {
            std::cout << "sono qui" << std::endl;
            try {
                std::ifstream file("survey_config.json");
                json j;
                file >> j;
                title = j["title"].get<std::string>();
                questions = j["questions"].get<std::vector<std::string>>();
                autoClose = j["autoClose"].get<bool>();
                continuous = j["continuous"].get<bool>();
                randomize = j["randomize"].get<bool>();
                rowHeight = j.count("rowHeight") > 0 ? j["rowHeight"].get<float>() : 30;
                for (auto& q : questions)
                    qWidth = 7 * q.length() > qWidth ? 7 * q.length() : qWidth;
                qWidth += 75;
                width = qWidth + 385;
                height = 85 + rowHeight * questions.size();
                responses = std::vector<Response>(questions.size(), NoResponse); 
                responsesC = std::vector<float>(questions.size(), 0.5f);
                order.resize(questions.size());
                std::iota(order.begin(), order.end(), 0);
                if (randomize) {
                    std::random_device rd;
                    std::mt19937 g(rd());
                    std::shuffle(order.begin(), order.end(), g);
                }
                set_window_title(title);
                set_window_size((int)width, (int)height);
                center_window();
            }
            catch(...) {
                std::cout << "sono qui 2" << std::endl;
                return false;
            }
            std::cout << "sono qui 3" << std::endl;
            return true;
        }
        std::cout << "sono qui 4" << std::endl;
        return false;
    }

    bool saveResponse() 
    {
        // make sure subject has value
        if (subject == -1) {
            message = "Please enter your subject identifier";
            ImGui::OpenPopup("Message");
            return false;
        }
        // make sure subject has age
        if (age == -1) {
            message = "Please enter your age";
            ImGui::OpenPopup("Message");
            return false;
        }
        if (sex == NoGender) {
            message = "Please enter your gender";
            ImGui::OpenPopup("Message");
            return false;
        }
        if (wristH <= 0 || wristC <= 0 || wristW <= 0) {
            message = "Please enter your wrist dimensions";
            ImGui::OpenPopup("Message");
            return false;
        }
        // make sure every question answered
        for (unsigned int i = 0; i < questions.size();  ++i) {
            if (continuous) {
                // if (responsesC[order[i]] == -1) {
                //     message = "Please respond to Question " + std::to_string(i+1);
                //     ImGui::OpenPopup("Message");
                //     return false;
                // }
            }
            else {
                if (responses[order[i]] == NoResponse) {
                    message = "Please respond to Question " + std::to_string(i+1);
                    ImGui::OpenPopup("Message");
                    return false;
                }
            }
        }
    
        // generate text responses
        std::vector<std::string> responsesText(responses.size());
        static std::map<Response, std::string> responseMap = {
            {StronglyDisagree, "Strongly Disagree"}, 
            {Disagree, "Disagree"},
            {Neutral, "Neutral"},
            {Agree, "Agree"},
            {StronglyAgree, "Strongly Agree"}
        };
        for (unsigned int i = 0; i < responses.size(); ++i)
            responsesText[i] = responseMap[responses[i]];
        // save data
        json j;
        j["subject"] = subject;
        j["age"] = age;
        j["gender"] = (sex == Male ? "Male" : sex == Female ? "Female" : sex == Other ? "Other" : "None");
        if (continuous) {
            j["responses"] = responsesC;
        }
        else {
            j["responses"] = responses;
            j["responsesText"] = responsesText;
        }
        j["order"] = order;
        j["wristW"] = wristW;
        j["wristH"] = wristH;
        j["wristC"] = wristC;
        std::ofstream file("subject_" + std::to_string(subject) + ".json");
        if (file.is_open())
            file << std::setw(4) << j << std::endl;
        // reset state
        subject = -1;
        sex = NoGender;
        age = -1;
        responses = std::vector<Response>(responses.size(), NoResponse);
        message = "Thank you for participating!";
        ImGui::OpenPopup("Message");
        return true;
    }

    int subject = -1;                    ///< subject input text
    bool loaded = false;                 ///< was the Likert config loaded?
    std::string title;                   ///< survey title
    std::vector<std::string> questions;  ///< survey questions
    std::vector<Response> responses;     ///< survey responses
    std::vector<float> responsesC;       ///< survey responses
    std::vector<int> order;              ///< question order
    Gender sex = NoGender;               ///< is subject male?
    int age = -1;                        ///< subject age
    bool autoClose = false;              ///< should the app close when the user submits a response?
    float width, height;                 ///< window width/height
    float qWidth, rowHeight;;            ///< row width/height
    std::string message;                 ///< error message to dispaly
    bool continuous;
    bool randomize;
    float wristH = 0, wristW = 0, wristC = 0; ///< wrist dims
};

int main(int argc, char const *argv[])
{
    // if there doesn't exist a "survey_config.json" file, make a default one
    if (!fs::exists("survey_config.json")) {
        json j;
        j["title"] = "My Likert Survey";
        j["questions"] = {"Making GUIs with mahi-gui is easy", 
                          "I found it difficult to make GUIs with mahi-gui",
                          "Jelly beans are the best candy",
                          "Jelly beans are disgusting",
                          "I like turtles",
                          "Turtles are disappointing",
                          "These questions are ridiculous",
                          "These questions are thought provoking"};
        j["autoClose"] = true;
        j["randomize"] = true;
        j["rowHeight"] = 30;
        j["continuous"] = false;
        std::ofstream file("survey_config.json");
        if (file.is_open())
            file << std::setw(4) << j;
    }
    // run the GUI
    Likert likert;
    likert.run();
    return 0;
}