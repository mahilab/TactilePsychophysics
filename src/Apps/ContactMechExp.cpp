#include "ContactMechGui.hpp"

int main(int argc, char *argv[])
{
    // Exp Specific
    Options options("capstan2DofPsychophysicalExp.exe","CM Contact Mechanics Study"); 
    options.add_options()
        ("s,subject","Subject ID Number: -s subnum",value<int>())
        ("i,indentation","Indentation Experiment: -i")
        ("c,creep","Creep Experiment: -c")
        ("r,relaxation","Stress Relaxation Experiment: -r")
        ("b,cycle","Cycle Between Bounds Experiment: -b")
        ("n,normal","Test the normal direction: -n")
        ("t,tangential","Test the tangential/shear direction: -t")
        ("h,help","print help");
    
    auto result = options.parse(argc, argv);

    if (result.count("help") > 0)
        print("{}",options.help());
    
    int subject_num;
    if (result.count("s"))
        subject_num = result["s"].as<int>();
    else {
        LOG(Error) << "Missing Subject Number Inputs. Exiting code.";
        print("{}",options.help());
        return 0;
    }

    ContactMechGui::WhichExp whichExp;
    if (result.count("i") && !result.count("c") && !result.count("r") && !result.count("b")){
        whichExp = ContactMechGui::Ind;
    }else if (result.count("c") && !result.count("r") && !result.count("b") && !result.count("i"))
        whichExp = ContactMechGui::Creep;
    else if (result.count("r") && !result.count("b") && !result.count("i") && !result.count("c"))
        whichExp = ContactMechGui::Relax;
    else if (result.count("b") && !result.count("i") && !result.count("c") && !result.count("r"))
        whichExp = ContactMechGui::Cycle;
    else{
        LOG(Error) << "Missing Experiment Type or Too Many Inputs. Exiting code.";
        print("{}",options.help());
        return 0;
    }
    
    ContactMechGui::WhichDof active_dof;
    if (result.count("n") && !result.count("t"))
        active_dof = ContactMechGui::Normal;
    else if (result.count("t") && !result.count("n"))
        active_dof = ContactMechGui::Shear;
    else{
        LOG(Error) << "Missing Dof to Test, Or Too Many Inputs. Exiting code.";
        print("{}",options.help());
        return 0;
    }

    ContactMechGui gui(subject_num, whichExp, active_dof);
    gui.run();
   
    return 0;
}