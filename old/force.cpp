// Evan Pezent
// Project CM - Force Analysis
// 10/04/2018

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
#include <MEL/Communications/UdpSocket.hpp>
#include <MEL/Communications/Packet.hpp>
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

//int main(void) {
//
//	UdpSocket udp;
//	udp.bind(55001);
//
//	Packet packet;
//

//	char str[] = "7 1 20 120 20 0.5";	
//
//	udp.send(str, sizeof(str), "127.0.0.1", 55002);//
//	return 0;
//
//}

int main(int argc, char* argv[]) {

	// register ctrl-c handler
	register_ctrl_handler(my_handler);

	// make relative filepath for saving data
	string filepath = "force_results_multi_ati/ex_files";

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
	Options options("force.exe");
	options.add_options()("v,vt", "VT Number", value<std::size_t>());
	auto input = options.parse(argc, argv);

	// make Xbox One controller
	XboxController xbox;
	if (!xbox.is_connected()) {
		LOG(Fatal) << "Connect Xbox controller to run program";
		return 1;
	}

	// create and open two Q8-USB
	QuanserOptions qoptions;
	qoptions.set_update_rate(QuanserOptions::UpdateRate::Normal);

	Q8Usb q8_0(qoptions);
	Q8Usb q8_1(qoptions);

	// open both Q8-USBs
	if (!q8_0.open() || !q8_1.open()) {
		LOG(Fatal) << "Failed to open Q8-USBs";
		return 1;
	}

	// identify Q8-USBs
	Q8Usb* q8_A;
	Q8Usb* q8_B;

	if (q8_0.identify(6) && q8_1.identify(7)) {
		q8_A = &q8_0;
		q8_B = &q8_1;
		LOG(Info) << "Q8-USB A has ID 0 and Q8-USB B has ID 1";
	}
	else if (q8_0.identify(7) && q8_1.identify(6)) {
		q8_A = &q8_1;
		q8_B = &q8_0;
		LOG(Info) << "Q8-USB A has ID 1 and Q8-USB B has ID 0";
	}
	else {
		LOG(Fatal) << "Failed to identify Q8 USBs. Try again.";
		return 1;
	}

	// create ATI sensor(s)
	AtiSensor ati_A, ati_B;
	if (!ati_A.load_calibration("FT21875.cal") || !ati_B.load_calibration("FT18021.cal")) {
		LOG(Fatal) << "Failed to load one or borth ATI calibration files";
		return 1;
	}

	// set ATI channels
	ati_A.set_channels(q8_A->AI[{2, 3, 4, 5, 6, 7}]);
	ati_B.set_channels(q8_B->AI[{2, 3, 4, 5, 6, 7}]);

	// make CM
	CM cm(q8_A->DO[0], q8_A->DI[0], q8_A->AO[0], q8_A->AI[0], q8_A->AI[1], q8_A->encoder[0]);

	//===========================================================================
	// FORCE ANALYSIS VARIABLES
	//===========================================================================

	// get VT number from console input
	std::size_t vt_num = 0;
	if (input.count("v"))
		vt_num = input["v"].as<std::size_t>();
	else {
		LOG(Fatal) << "VT Number must be entered with -v command";
		return 1;
	}

	// other variables
	std::size_t num_levels = 10;
	std::size_t num_iterations = 10;
	auto torque_levels = linspace(cm.motor_stall_torque * 0.1, cm.motor_stall_torque, num_levels);
	std::vector<Time> state_durations = { seconds(1), seconds(3), seconds(16) };
	Time state_time = Time::Zero;
	Time test_time = Time::Zero;
	std::size_t current_torque_index = 0;
	std::size_t current_iteration = 0;
	std::size_t state = 0; // 0 returning to zero, 1 tightening, 2 cool-down
	double cm_force_offset = 0.0;
						   
	// make melshares for force
	MelShare ms_ati_A_f("ati_A_force");
	MelShare ms_ati_B_f("ati_B_force");
	MelShare ms_testing("testing_state");
	MelShare melshare_("cm");

	// control flags
	bool atis_zeroed = false;
	bool cm_zeroed = false;
	bool test_running = false;

	//==========================================================================
	// DATA LOGGING
	//==========================================================================

	array<string, 22> header = 
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
			"CM Volts [volts]"
		    "ATI A Fx [N]",
		    "ATI A Fy [N]",
		    "ATI A Fz [N]",
		    "ATI A Tx [Nmm]",
		    "ATI A Ty [Nmm]",
		    "ATI A Tz [Nmm]",
		    "ATI B Fx [N]",
		    "ATI B Fy [N]",
		    "ATI B Fz [N]",
		    "ATI B Tx [Nmm]",
		    "ATI B Ty [Nmm]",
		    "ATI B Tz [Nmm]"
		};

	array<double,22> data;
	vector<array<double, 22>> logging;

	//==========================================================================
	// CONTROL LOOP
	//==========================================================================

	// enable Q8-USB
	q8_A->enable();
	q8_B->enable();

	// create timer
	Timer timer(hertz(1000), Timer::Hybrid);
	
	// start loop
	while (!STOP) {

		//======================================================================
		// DAQ INPUT
		//======================================================================

		// sync DAQ inputs with real world
		q8_A->update_input();
		q8_B->update_input();

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

			// zero CM
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
				ati_A.zero();
				ati_B.zero();
				LOG(Info) << "ATI Nano17's Zeroed";
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

			// // log data
			
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
					cm.get_volts(),				  // CM Force Sensor Voltage [volts]
					ati_A.get_force(AxisX),           // ATI A Fx [N]
					ati_A.get_force(AxisY),           // ATI A Fy [N]
					ati_A.get_force(AxisZ),           // ATI A Fz [N]
					ati_A.get_torque(AxisX),          // ATI A Tx [Nmm]
					ati_A.get_torque(AxisY),          // ATI A Ty [Nmm]
					ati_A.get_torque(AxisZ),          // ATI A Tz [Nmm]
					ati_B.get_force(AxisX),           // ATI B Fx [N]
					ati_B.get_force(AxisY),           // ATI B Fy [N]
					ati_B.get_force(AxisZ),           // ATI B Fz [N]
					ati_B.get_torque(AxisX),          // ATI B Tx [Nmm]
					ati_B.get_torque(AxisY),          // ATI B Ty [Nmm]
					ati_B.get_torque(AxisZ)           // ATI B Tz [Nmm
				};

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
						//log.save_and_clear_data("force_analysis_vt" + std::to_string(vt_num) + "_torque" + std::to_string(current_torque_index), "force_analysis_vt" + std::to_string(vt_num));
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
		ms_ati_A_f.write_data(ati_A.get_forces());
		ms_ati_B_f.write_data(ati_B.get_forces());
		ms_testing.write_data({ (double)vt_num, (double)current_torque_index, (double)current_iteration, (double)state });

		// broadcast cm data
		cm.broadcast();
	
		//======================================================================
		// DAQ OUTPUT
		//======================================================================

		// sync DAQ outputs with real world
		q8_A->update_output();	

		// wait contro/ loop timer
		timer.wait();
	}

	// disable and close Q8-USB
	q8_A->disable();
	q8_B->disable();
	q8_A->close();
	q8_B->close();

	return 0;
 }


