// Pezent, Cambio
// CM - ATI Testing
// 05/06/2019

#include <MEL/Core/Console.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Math/Waveform.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/Csv.hpp>
#include <MEL/Devices/Windows/Keyboard.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Core.hpp>
#include <MEL/Mechatronics.hpp>
#include <MEL/Devices/Windows/XboxController.hpp>
#include <MEL/Math/Butterworth.hpp>
#include <MEL/Devices/AtiSensor.hpp>
#include <MEL/Math.hpp>
#include <MEL/Utility/Options.hpp>
#include <array>
#include <vector>
#include <string>
#include <MEL/Math/Filter.hpp>

using namespace mahi::daq;
using namespace mahi::util;
using namespace std;

// atomic stop flag
ctrl_bool STOP(false);

// console control event handler
bool my_handler(CtrlEvent event) {
	if (event == CtrlEvent::CtrlC) {
		LOG(Warning) << "CTRL+C pressed";
		STOP = true;
	}
	else if (event == CtrlEvent::Close) {
		LOG(Warning) << "Console window closed";
		STOP = true;
	}
	return 1;
}


int main(int argc, char* argv[]) {

	// register ctrl-c handler
	register_ctrl_handler(my_handler);

	// make relative filepath for saving data
	string filepath = "ATI_testing/example_files";

	// ensure not overwriting current file
	ifstream file(filepath);
	if (file.is_open())
	{
		LOG(Fatal) << "Change file path or delete existing file";
		return 1;
	};

	//==========================================================================
	// HARDWARE INIT
	//==========================================================================

	// console options
	Options options("ATI_testing.exe");
	// options.add_options()("v,vt", "VT Number", value<std::size_t>());
	// auto input = options.parse(argc, argv);

	// make Xbox One controller
	XboxController xbox;
	if (!xbox.is_connected()) {
		LOG(Fatal) << "Connect Xbox controller to run program";
		return 1;
	}

	Q8Usb q8;

	// open Q8-USB
	if (!q8.open()) {
		LOG(Fatal) << "Failed to open Q8-USB";
		return 1;
	}

	// create ATI sensor(s)
	AtiSensor ati;
	if (!ati.load_calibration("FT19612.cal")) {
		LOG(Fatal) << "Failed to load ATI calibration file";
		return 1;
	}

	// set ATI channels
	ati.set_channels(q8.AI[{2, 3, 4, 5, 6, 7}]);

	
	//===========================================================================
	// FORCE ANALYSIS VARIABLES
	//===========================================================================

	// other variables
	Time state_time = Time::Zero;
	Time test_time = Time::Zero;
	size_t current_iteration = 0;
	int state = 0; 
	Time state_duration = seconds(5);
	bool testing = false;
						   
	// make melshares for force
	MelShare ms_ati_f("ati_force");
	MelShare ms_ATI_test("ati_test_data");

	// control flags
	bool atis_zeroed = false;

	//==========================================================================
	// DATA LOGGING
	//==========================================================================

	array<string, 14> header =  
    { 
		"Test Time [s]",
		"Iteration",
		"State",
		"State [N]",
		"State Time [s]",
		"ATI Fx [N]",
		"ATI Fy [N]",
		"ATI Fz [N]",
		"ATI Tx [Nmm]",
		"ATI Ty [Nmm]",
		"ATI Tz [Nmm]",
		"Filtered ATI Fx [N]",
		"Filtered ATI Fy [N]",
		"Filtered ATI Fz [N]",
	};

	array<double,14> data;
	vector<array<double, 14>> logging;
	//==========================================================================
	// CONTROL LOOP
	//==========================================================================

	// enable Q8-USB
	q8.enable();

	// create timer
	Timer timer(hertz(1000), Timer::Hybrid);

		// create filters

	Butterworth buttX(4, hertz(10), timer.get_frequency());
	Butterworth buttY(4, hertz(10), timer.get_frequency());
	Butterworth buttZ(4, hertz(10), timer.get_frequency());
	
	// start loop
	while (!STOP) {

		//======================================================================
		// DAQ INPUT
		//======================================================================

		// sync DAQ input with real world
		q8.update_input();

		//======================================================================
		// XBOX CONTROLLER INPUT
		//======================================================================

		// stop if back pressed
		if (xbox.is_button_pressed(XboxController::Start)) {
			testing = true;
		}

		// stop if back pressed
		if (xbox.is_button_pressed(XboxController::Back)) {
			STOP = true;
		}

		// zero sensors
		if (xbox.is_button_pressed(XboxController::RS)) {
			ati.zero();
			LOG(Info) << "ATI Nano17 Zeroed";
		}


		//======================================================================
		// FORCE ANALYSIS
		//======================================================================
		if (testing == true){

			data = { 
				test_time.as_seconds(),           // Test Time [s]
				(double)current_iteration,        // Iteration
				(double)state,                    // State [gm]
				(double)state * 0.0098,						// State [N]
				state_time.as_seconds(),          // State Time [s]
				ati.get_force(AxisX),           // ATI Fx [N]
				ati.get_force(AxisY),           // ATI Fy [N]
				ati.get_force(AxisZ),           // ATI Fz [N]
				ati.get_torque(AxisZ),          // ATI Tz [Nmm]
				ati.get_torque(AxisX),          // ATI Tx [Nmm]
				ati.get_torque(AxisY),          // ATI Ty [Nmm]
				buttX.update(ati.get_force(AxisX)),           // Filtered ATI Fx [N]
				buttY.update(ati.get_force(AxisY)),           // Filtered ATI Fy [N]
				buttZ.update(ati.get_force(AxisZ)),           // Filtered ATI Fz [N]
			};

			// add data to logging for this iteration
			logging.push_back(data);

			// increment times
			state_time += timer.get_period();
			test_time += timer.get_period();

			// determine next state if time elapsed
			if (state_time == state_duration) {
				state_time = Time::Zero;
				LOG(Info) << "Current Iteration " << current_iteration << " Completed";
				current_iteration++;
				csv_write_row(filepath + "_mass_" + to_string(state) + ".csv", header);
				csv_append_rows(filepath + "_mass_" + to_string(state) + ".csv", logging);
				logging.clear();
				prompt("Place weight on sensor, enter mass in grams, and press ENTER");
				cin >> state;
			}

			// exit condition
			if (xbox.is_button_pressed(XboxController::X)) {
				LOG(Info) << "Force Analysis Completed";
				STOP = true;
				
			}

		//======================================================================
		// MELSCOPE
		//======================================================================

		// scope ati force/torque
		ms_ati_f.write_data(ati.get_forces());
		ms_ATI_test.write_data({buttY.update(ati.get_force(AxisY)), (double)state * 0.0098});
	
		//======================================================================
		// DAQ OUTPUT
		//======================================================================

		// sync DAQ outputs with real world
		q8.update_output();	

		// wait contro/ loop timer
		timer.wait();

		}
	}

	// disable and close Q8-USB
	q8.disable();
	q8.close();

	return 0;
 }


