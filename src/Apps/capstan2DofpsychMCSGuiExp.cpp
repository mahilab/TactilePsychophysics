#include "PsychophysicalTesting.hpp"
#include "PsychGui.hpp"

int main(int argc, char *argv[])
{
    // Exp Specific
    Options options("capstan2DofPsychophysicalExp.exe","CM Method of Constant Stimuli Study"); 
    options.add_options()
        ("s,subject","Subject ID Number: -s subnum",value<int>())
        ("c,constants","Method of Constant Stimuli Experiment: -c")
        ("w,staircase","Staircase Method Experiment: -w")
        ("n,normal","Test the normal direction: -n")
        ("t,tangential","Test the tangential/shear direction: -t")
        ("p,position","Test using position control: -p")
        ("f,force","Test using position control: -f")
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

    PsychTest::WhichExp whichExp;
    if (result.count("c") && !result.count("w")){
        whichExp = PsychTest::MCS;
    }else if (result.count("w") && !result.count("c"))
        whichExp = PsychTest::SM;
    else{
        LOG(Error) << "Missing Experiment Type or Too Many Inputs. Exiting code.";
        print("{}",options.help());
        return 0;
    }
    
    PsychTest::WhichDof active_dof;
    if (result.count("n") && !result.count("t"))
        active_dof = PsychTest::Normal;
    else if (result.count("t") && !result.count("n"))
        active_dof = PsychTest::Shear;
    else{
        LOG(Error) << "Missing Dof to Test, Or Too Many Inputs. Exiting code.";
        print("{}",options.help());
        return 0;
    }

    PsychTest::ControlType active_control;
    if (result.count("p") && !result.count("f"))
        active_control = PsychTest::Position;
    else if (result.count("f") && !result.count("p"))
        active_control = PsychTest::Force;
    else{
        LOG(Error) << "Missing Controller Type, Or Too Many Inputs. Exiting code.";
        print("{}",options.help());
        return 0;
    }

    PsychGui gui(subject_num, whichExp, active_dof, active_control);
    gui.run();
   
    return 0;
}