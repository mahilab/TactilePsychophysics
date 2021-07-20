// Pezent, Cambio
// CM - Force Verification
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
#include "CapstanModule.hpp"
#include <MEL/Utility/Options.hpp>
#include <array>
#include <vector>
#include <string>

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
	string filepath = "force_results/example_files_long";

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
	Options options("force_verify.exe");
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
	
	// make CM
	CM cm(q8.DO[0], q8.DI[0], q8.AO[0], q8.AI[0], q8.AI[1], q8.encoder[0]);
	
	//===========================================================================
	// FORCE ANALYSIS VARIABLES
	//===========================================================================

	// other variables
	std::size_t num_levels = 20;
	std::size_t num_iterations = 20;
	auto torque_levels = linspace(cm.motor_stall_torque * 0.05, cm.motor_stall_torque, num_levels);
	std::vector<Time> state_durations = { seconds(1), seconds(3), seconds(6) };
	Time state_time = Time::Zero;
	Time test_time = Time::Zero;
	std::size_t current_torque_index = 0;
	std::size_t current_iteration = 0;
	std::size_t state = 0; // 0 returning to zero, 1 tightening, 2 cool-down
	double cm_force_offset = 0.0;

	
						   
	// make melshares for force
	MelShare ms_ati_f("ati_force");
	MelShare ms_testing("testing_state");
	MelShare ms_cm_v("cm_volts");
	MelShare melshare_("cm");

	// control flags
	bool atis_zeroed = false;
	bool cm_zeroed = false;
	bool test_running = false;

	//==========================================================================
	// DATA LOGGING
	//==========================================================================

	array<string, 16> header =  
    { 
		"Test Time [s]",
		"Torque Index",
		"Iteration",
		"State",
		"State Time [s]",
		"Motor Torque Command [Nm]",
		"Motor Torque Sense [Nm]",
		"Motor Position [deg]",
		"Motor Velocity [deg/s]",
		"Force Sensor [volts]",
		"ATI Fx [N]",
		"ATI Fy [N]",
		"ATI Fz [N]",
		"ATI Tx [Nmm]",
		"ATI Ty [Nmm]",
		"ATI Tz [Nmm]",
	};

	array<double,16> data;
	vector<array<double, 16>> logging;
	//==========================================================================
	// CONTROL LOOP
	//==========================================================================

	// enable Q8-USB
	q8.enable();

	// create timer
	Timer timer(hertz(1000), Timer::Hybrid);
	
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

		// enable if A pressed
		if (xbox.is_button_pressed(XboxController::A) && !cm.is_enabled()) {
			cm.enable();
			LOG(Info) << "CM Enabled";
		}

		// disable if X pressed
		if (xbox.is_button_pressed(XboxController::X) && cm.is_enabled()) {
			cm.disable();
			LOG(Info) << "CM Disabled";
		}

		// stop if back pressed
		if (xbox.is_button_pressed(XboxController::Back)) {
			STOP = true;
		}

		// manual control
		if (!test_running) {
			if (xbox.is_button_pressed(XboxController::Left))
				cm.set_motor_position(-1000);
			else if (xbox.is_button_pressed(XboxController::Right))
				cm.set_motor_position(1000);
			else if (xbox.is_button_pressed(XboxController::Up))
				cm.set_motor_position(0);
			else if (xbox.is_button_pressed(XboxController::RB))
				cm.set_motor_torque(cm.motor_stall_torque);
			else if (xbox.is_button_pressed(XboxController::LB))
				cm.set_motor_torque(-cm.motor_stall_torque);
			else {
				double torque = cm.motor_stall_torque * (xbox.get_axis(XboxController::RT) - xbox.get_axis(XboxController::LT));
				cm.set_motor_torque(torque);
			}

			// zero cm encoder 
			if (xbox.is_button_pressed(XboxController::LS)) {
				cm.zero();
				LOG(Info) << "CM Encoder Zeroed";
			}

			// zero cm force 
			if (xbox.is_button_pressed(XboxController::B)) {
				cm_force_offset = cm.get_volts();
				LOG(Info) << "CM Force Sensor Zeroed";
			}

			// zero sensors
			if (xbox.is_button_pressed(XboxController::RS)) {
				ati.zero();
				LOG(Info) << "ATI Nano17 Zeroed";
			}

			// start testing
			if (xbox.is_button_pressed(XboxController::Start)) {
				LOG(Info) << "Force Analysis Started";
				test_running = true;

			}
		}

		//======================================================================
		// FORCE ANALYSIS
		//======================================================================

		if (test_running) {

			// set appropriate control torque
			if (state == 0)      // return to zero
				cm.set_motor_position(0);
			else if (state == 1) // tighten
				cm.set_motor_torque(torque_levels[current_torque_index]);
			else if (state == 2) // cool down
				cm.set_motor_torque(0);

			data = { 
				test_time.as_seconds(),           // Test Time [s]
				(double)current_torque_index,     // Torque Index
				(double)current_iteration,        // Iteration
				(double)state,                    // State
				state_time.as_seconds(),          // State Time [s]
				cm.get_motor_torque_command(), // Motor Torque Command [Nm]
				cm.get_motor_torque_sense(),   // Motor Torque Sense [Nm]
				cm.get_motor_position(),       // Motor Position [deg]
				cm.get_motor_velocity(),       // Motor Velocity [deg/s]
				cm.get_volts() - cm_force_offset, 	 // Force Sensor Output [volts]
				ati.get_force(AxisX),           // ATI Fx [N]
				ati.get_force(AxisY),           // ATI Fy [N]
				ati.get_force(AxisZ),           // ATI Fz [N]
				ati.get_torque(AxisZ),          // ATI Tz [Nmm]
				ati.get_torque(AxisX),          // ATI Tx [Nmm]
				ati.get_torque(AxisY),          // ATI Ty [Nmm]

			};

			// add data to logging for this iteration
			logging.push_back(data);

			// increment times
			state_time += timer.get_period();
			test_time += timer.get_period();

			// determine next state if time elapsed
			if (state_time == state_durations[state]) {
				state_time = Time::Zero;
				if (state == 0 || state == 1)
					state++;
				else if (state == 2) {
					LOG(Info) << "Current Iteration " << current_iteration << " Completed";
					current_iteration++;
					if (current_iteration == num_iterations) {
						LOG(Info) << "Current Torque Level " << current_torque_index << " Completed";
						csv_write_row(filepath + "_torque_level_" + to_string(current_torque_index) + ".csv", header);
						csv_append_rows(filepath + "_torque_level_" + to_string(current_torque_index) + ".csv", logging);
						logging.clear();
						current_iteration = 0;
						current_torque_index++;
						test_time = Time::Zero;
					}
					state = 0;
				}
			}

			// exit condition
			if (current_torque_index == torque_levels.size()) {
				LOG(Info) << "Force Analysis Completed";
				STOP = true;
				
			}
		}

		//======================================================================
		// MELSCOPE
		//======================================================================

		// scope ati force/torque
		ms_ati_f.write_data(ati.get_forces());
		ms_testing.write_data({ (double)current_torque_index, (double)current_iteration, (double)state });
		ms_cm_v.write_data({(double)cm.get_volts() - cm_force_offset});

		// broadcast TASBI data
		cm.broadcast();
	
		//======================================================================
		// DAQ OUTPUT
		//======================================================================

		// sync DAQ outputs with real world
		q8.update_output();	

		// wait contro/ loop timer
		timer.wait();
	}

	// disable and close Q8-USB
	q8.disable();
	q8.close();

	return 0;
 }


