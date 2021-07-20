// Evan Pezent
// Project Poppy - Force Analysis
// 10/04/2018

#include <MEL/Core/Console.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Math/Waveform.hpp>
#include <MEL/Logging/Log.hpp>
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
#include <MEL/Logging/DataLogger.hpp>
#include "Poppy.hpp"
#include <MEL/Utility/Options.hpp>
#include <MEL/Math/Waveform.hpp>
#include <MEL/Math/Chirp.hpp>

using namespace mahi::daq;
using namespace mahi::util;

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

	//==========================================================================
	// HARDWARE INIT
	//==========================================================================

	// console options
	Options options("poppy.exe");
	options.add_options()("h,help", "Get help info", value<std::size_t>());
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
	Q8Usb q8(qoptions);

	// open both Q8-USBs
	if (!q8.open()) {
		LOG(Fatal) << "Failed to open Q8-USBs";
		return 1;
	}

	// make Poppy
	Poppy poppy(q8.DO[0], q8.DI[0], q8.AO[0], q8.AI[7], q8.encoder[0]);

	//===========================================================================
	// BANDWIDTH ANALYSIS VARIABLES
	//===========================================================================

	// control flags
	bool tuning = false;

	Time t = Time::Zero;

	Waveform sqrwave(Waveform::Square, seconds(2), 1000);
	Waveform sinwave(Waveform::Sin, seconds(0.2), 1000);
	Chirp chirp(hertz(1), hertz(20), seconds(10), 1000);

	TimeFunction* f = &sqrwave;

	MelShare ms_traj("traj");

	std::vector<double> preset_torques = { 0.14, 0.4, 0.4, 0.6 };
	MelShare ms_presets("presets");

	Clock clock;
	Time click_time = milliseconds(200);

	//==========================================================================
	// DATA LOGGING
	//==========================================================================

	DataLogger log(WriterType::Buffered);
	log.set_header(
		{
			"Test Time [s]",
			"Motor Torque Command [Nm]",
			"Motor Torque Sense [Nm]",
			"Motor Position [deg]",
			"Motor Velocity [deg/s]",
		}
	);

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

		// sync DAQ inputs with real world
		q8.update_input();

		//======================================================================
		// XBOX CONTROLLER INPUT
		//======================================================================

		// enable if A pressed
		if (xbox.is_button_pressed(XboxController::A) && !poppy.is_enabled()) {
			poppy.enable();
			LOG(Info) << "Poppy Enabled";
		}

		// disable if X pressed
		if (xbox.is_button_pressed(XboxController::X) && poppy.is_enabled()) {
			poppy.disable();
			LOG(Info) << "Poppy Disabled";
		}

		// stop if back pressed
		if (xbox.is_button_pressed(XboxController::B)) {
			STOP = true;
		}

		// manual control
		if (!tuning) {

			if (xbox.is_button_pressed(XboxController::Up))
				poppy.set_motor_torque(preset_torques[0] * poppy.motor_stall_torque);
			else if (xbox.is_button_pressed(XboxController::Right))
				poppy.set_motor_torque(preset_torques[1] * poppy.motor_stall_torque);
			else if (xbox.is_button_pressed(XboxController::Down))
				poppy.set_motor_torque(preset_torques[2] * poppy.motor_stall_torque);
			else if (xbox.is_button_pressed(XboxController::Left))
				poppy.set_motor_torque(preset_torques[3] * poppy.motor_stall_torque);


			else if (xbox.is_button_pressed(XboxController::Y))
				poppy.set_motor_position(0);

			else {
				double torque = poppy.motor_stall_torque * (xbox.get_axis(XboxController::RT) - xbox.get_axis(XboxController::LT));
				poppy.set_motor_torque(torque);
			}




			// zero sensors
			if (xbox.is_button_pressed(XboxController::LS)) {
				poppy.zero();
				LOG(Info) << "Poppy Zeroed";
			}

			// start testing
			if (xbox.is_button_pressed(XboxController::Start)) {
				t = Time::Zero;
				LOG(Info) << "Entered Tuning Mode";
				tuning = true;

			}
		}

		else {

			int i = -1;

			if (xbox.is_button_pressed(XboxController::Up))
				i = 0;
			else if (xbox.is_button_pressed(XboxController::Right))
				i = 1;
			else if (xbox.is_button_pressed(XboxController::Down))
				i = 2;
			else if (xbox.is_button_pressed(XboxController::Left))
				i = 3;

			if (i > -1) {
				if (xbox.is_button_pressed(XboxController::LB) && clock.get_elapsed_time() > click_time)
				{
					preset_torques[i] -= 0.01;
					clock.restart();
				}
				if (xbox.is_button_pressed(XboxController::RB) && clock.get_elapsed_time() > click_time)
				{
					preset_torques[i] += 0.01;
					clock.restart();
				}
				preset_torques[i] = saturate(preset_torques[i], 0.0, 1.0);
			}




			if (xbox.is_button_pressed(XboxController::Back)) {
				LOG(Info) << "Exited Tuning Mode";
				tuning = false;

			}

			t += timer.get_period();

		}

		//======================================================================
		// MELSCOPE
		//======================================================================

		// broadcast poppy data
		poppy.broadcast();

		ms_presets.write_data(preset_torques);

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

	// wait for logging thread to end
	log.wait_for_save();

	return 0;
}


