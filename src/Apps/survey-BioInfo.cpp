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

    std::string filename = "C:/Git/TactilePsychophysics/src/Apps/survey_config.json";
    enum Gender { NA, Male, Female, Other };

    /// Constructor
    Likert() : Application(600,200,"",false) { 
        ImGui::DisableViewports();
        ImGui::StyleColorsMahiDark3();
    }

    /// GUI code
    void update() override {
        ImGui::BeginFixed("##Likert", ImGui::GetMainViewport()->Pos, {600, 200}, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings);

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

        ImGui::SameLine(ImGui::GetWindowWidth() - 105);
        if (ImGui::ButtonColored("Submit", Blues::DodgerBlue, {100,0})) {
            bool result = saveResponse();
            if (result && autoClose)
                quit();
        }

        ImGui::Separator();

        
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

        ImGui::Separator();
        ImGui::Separator();

        ImGui::Text("Body Measures:");            
        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("BMI",&bmi);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("Arm Circumference",&armC);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("Skin Fold",&skinFold);

        ImGui::Separator();
        ImGui::Separator();

        ImGui::Text("Hairiness Measures:");            
        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("Hair Length",&hairL);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("Hair Thickness",&hairT);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("Hair Density [no./in^2]",&hairDensity);

        // begin message modal if opened this frame
        bool dummy = true;
        if (ImGui::BeginPopupModal("Message", &dummy, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted(message.c_str());
            ImGui::EndPopup();
        }
        ImGui::End();
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
        if (sex == NA) {
            message = "Please enter your gender";
            ImGui::OpenPopup("Message");
            return false;
        }
        if (bmi == -1) {
            message = "Please enter your BMI";
            ImGui::OpenPopup("Message");
            return false;
        }
        if (armC == -1) {
            message = "Please enter your arm circumference";
            ImGui::OpenPopup("Message");
            return false;
        }
        if (skinFold == -1) {
            message = "Please enter your skin fold measurement";
            ImGui::OpenPopup("Message");
            return false;
        }
        if (hairL == -1) {
            message = "Please enter your hair length";
            ImGui::OpenPopup("Message");
            return false;
        }
        if (hairT == -1) {
            message = "Please enter your hair thickness";
            ImGui::OpenPopup("Message");
            return false;
        }
        if (hairDensity == -1) {
            message = "Please enter your hair density in hairs per in^2";
            ImGui::OpenPopup("Message");
            return false;
        }

        // save data
        json j;
        j["subject"] = subject;
        j["age"] = age;
        j["BMI"] = bmi; 
        j["Arm Circumference"] = armC; 
        j["Skin Fold"] = skinFold; 
        j["Hair Length"] = hairL;
        j["Hair Thickness"] = hairT; 
        j["Hair Density"] = hairDensity; 
        std::ofstream file("C:/Git/TactilePsychophysics/data/BioInfo/bio_measures_subject_" + std::to_string(subject) + ".json");
        if (file.is_open())
            file << std::setw(4) << j << std::endl;
        // reset state
        subject = -1;
        sex = NA;
        age = -1;
        bmi = -1;
        armC = -1;
        skinFold = -1;
        hairL = -1;
        hairT = -1;
        hairDensity = -1;
        message = "Thank you for participating!";
        ImGui::OpenPopup("Message");
        return true;
    }

    int subject = -1;                    ///< subject input text
    Gender sex = NA;               ///< is subject male?
    int age = -1;                        ///< subject age
    float bmi = -1;
    float armC = -1;
    float skinFold = -1;
    float hairL = -1;
    float hairT = -1;
    float hairDensity = -1;
    bool autoClose = false;              ///< should the app close when the user submits a response?
    std::string message;                 ///< error message to dispaly
};

int main(int argc, char const *argv[])
{
    Likert likert;
    likert.run();
    return 0;
}