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
#include "CapstanModule.hpp"
#include <MEL/Utility/Options.hpp>
#include <MEL/Math/Waveform.hpp>
#include <MEL/Math/Chirp.hpp>
#include <MEL/Math/Butterworth.hpp>

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
	//qoptions.set_encoder_filter(0, QuanserOptions::EncoderFilter::Filtered);
	qoptions.set_update_rate(QuanserOptions::UpdateRate::Normal);
	Q8Usb q8(qoptions, false);

	// open both Q8-USBs
	if (!q8.open()) {
		LOG(Fatal) << "Failed to open Q8-USBs";
		return 1;
	}

	// make CM
	CM cm(q8.DO[0], q8.DI[0], q8.AO[0], q8.AI[0], q8.AI[1], q8.encoder[0]);

	//==========================================================================
	// SERVER VARIABLES
	//==========================================================================

	bool serving = false;
	double loose_position = 0;
	double tight_position = -10000;
	double pretension = 0.15;
	bool unplugged = true;

	MelShare ms_squeeze("squeeze");
	ms_squeeze.write_data({ 0.0 });

	MelShare ms_postion("position");
	MelShare ms_debug("debug");

	// VR signal filter
	Butterworth butt(2, hertz(10), hertz(1000), Butterworth::Lowpass);

	//==========================================================================
	// CONTROL LOOP
	//==========================================================================

	// enable Q8-USB
	q8.enable();

	// create timer
	Timer timer(hertz(1000), Timer::Hybrid);

	Clock clock;
	Time click_time = milliseconds(200);

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
		if (xbox.is_button_pressed(XboxController::A) && !cm.is_enabled()) {
			cm.enable();
			// cm.set_motor_torque(cm.motor_stall_torque);
			LOG(Info) << "CM enabled.";
			if (unplugged) {
				cm.zero();
				LOG(Info) << "CM zeroed.";
				unplugged = false;
			}
		}

		// disable if X pressed
		if (xbox.is_button_pressed(XboxController::X) && cm.is_enabled()) {
			cm.disable();
			// cm.set_motor_torque(0.0);
			LOG(Info) << "CM disabled.";
		}

		// stop if back pressed
		if (xbox.is_button_pressed(XboxController::B)) {
			STOP = true;
		}

		if (cm.is_enabled() && std::abs(cm.get_encoder_counts()) > 100000) {
			LOG(Error) << "CM may have been unplugged.";
			cm.disable();
			LOG(Info) << "CM disabled.";
			if (serving) {
				LOG(Info) << "Exited serving mode.";
				serving = false; 
			}
			unplugged = true;
		}

		// manual control
		if (!serving) {
			// set loose and tight positions
			if (xbox.is_button_pressed(XboxController::Up) && clock.get_elapsed_time() > click_time) {
				pretension += 0.005;
				LOG(Info) << "Pretension set to " << pretension << ".";				
				clock.restart();
			}
			if (xbox.is_button_pressed(XboxController::Down) && clock.get_elapsed_time() > click_time) {
				pretension -= 0.005;		
				LOG(Info) << "Pretension set to " << pretension << ".";						
				clock.restart();
			}
			if (xbox.is_button_pressed(XboxController::RB)) {
				cm.set_motor_torque(pretension * cm.motor_stall_torque);
			}
			else if (xbox.is_button_pressed(XboxController::LB)) {
				cm.set_motor_torque(-pretension * cm.motor_stall_torque);
			}
			else {
				double torque = cm.motor_stall_torque * (xbox.get_axis(XboxController::RT) - xbox.get_axis(XboxController::LT));
				cm.set_motor_torque(torque);
			}			
			if (xbox.is_button_pressed(XboxController::Left) && clock.get_elapsed_time() > click_time) {
				loose_position = cm.get_motor_position();
				LOG(Info) << "Loose position set to " << loose_position << ".";
				clock.restart();
			}
			if (xbox.is_button_pressed(XboxController::Right) && clock.get_elapsed_time() > click_time) {
				tight_position = cm.get_motor_position();
				LOG(Info) << "Tight position set to " << tight_position << ".";
				clock.restart();
			}
			// zero sensors
			if (xbox.is_button_pressed(XboxController::Y) && clock.get_elapsed_time() > click_time) {
				cm.zero();
				LOG(Info) << "CM zeroed.";
				clock.restart();
			}
			// start testing
			if (xbox.is_button_pressed(XboxController::Start)) {
					cm.zero();
					LOG(Info) << "CM zeroed.";
					LOG(Info) << "Entered serving mode.";
					serving = true;
			}
		}
		else {		

			// start serving
			double squeeze = ms_squeeze.read_data()[0];
			// double squeeze = std::sin(2*PI*0.5*timer.get_elapsed_time().as_seconds());
			double position = tight_position * squeeze;
			double filt = butt.update(position);

			cm.set_motor_position(filt);

			ms_postion.write_data({ position, filt });

			// return to setting mode
			if (xbox.is_button_pressed(XboxController::Back)) {
				LOG(Info) << "Exited serving mode.";
				serving = false;
			}
		}

		//======================================================================
		// MELSCOPE
		//======================================================================

		// broadcast cm data
		cm.broadcast();

		ms_debug.write_data({q8.AI[0], q8.AI[1]});

		//======================================================================
		// DAQ OUTPUT
		//======================================================================

		// sync DAQ outputs with real world
		q8.update_output();

		// wait contro/ loop timer
		timer.wait();
	}

	cm.disable();
	cm.broadcast();

	// disable and close Q8-USB
	q8.disable();
	q8.close();

	return 0;
}


