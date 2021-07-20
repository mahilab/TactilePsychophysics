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
// #include <MEL/Logging/DataLogger.hpp>
#include "CapstanModule.hpp"
#include <MEL/Utility/Options.hpp>
#include <MEL/Math/Waveform.hpp>
#include <MEL/Math/Chirp.hpp>
#include <sstream>

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

	// enable realtime
	//enable_realtime();

	// console options
	Options options("bandwidth.exe");
	options.add_options()
		("a,amp", "Set Amplitude (deg)", value<double>())
		("f,freq", "Set Frequency (Hz)", value<double>())
		("d,dur", "Set Duration (s)", value<double>())
		("s,sin", "Sets Sinwave as Default")
		("c,chirp", "Sets Chirp as Default")
		("q,square", "Sets Squarewave as  Default")
		("h,help", "Prints Help Message");

	auto input = options.parse(argc, argv);
	if (input.count("h")) {
		print(options.help());
		return 0;
	}

	//==========================================================================
	// HARDWARE INIT
	//==========================================================================

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

	// make CM
	CM cm(q8.DO[0], q8.DI[0], q8.AO[0], q8.AI[7], q8.encoder[0]);

	//poppy.set_pd_gains(0.000030, 0.00000020);

	//===========================================================================
	// BANDWIDTH ANALYSIS VARIABLES
	//===========================================================================
	
	// control flags
	bool poppy_zeroed = false;
	bool test_running = false;

	Time t = Time::Zero;

	double amplitude = 1000;
	Frequency freq= hertz(10);
	Time dur = seconds(10);

	MelShare ms_traj("traj");

	// override defaults

	if (input.count("a"))
		amplitude = input["a"].as<double>();
	if (input.count("f"))
		freq = hertz(input["f"].as<double>());
	if (input.count("d"))
		dur = seconds(input["d"].as<double>());

	Waveform sqrwave(Waveform::Square, seconds(2), amplitude);
	Waveform sinwave(Waveform::Sin, freq.to_time(), amplitude);
	Chirp chirp(hertz(1), freq, dur, amplitude);

	TimeFunction* f = &sinwave;


	if (input.count("s"))
		f = &sinwave;
	else if (input.count("c"))
		f = &chirp;
	else if (input.count("q"))
		f = &sqrwave;

	LOG(Info) << "Amplitude: " << amplitude << " deg";
	LOG(Info) << "Frequency: " << freq.as_hertz() << " Hz";
	LOG(Info) << "Duration:  " << dur.as_seconds() << " s";

	if (f == &sinwave)
		LOG(Info) << "Waveform:  Sinwave";
	else if (f == &sqrwave)
		LOG(Info) << "Waveform:  Squarewave";
	else if (f == &chirp)
		LOG(Info) << "Waveform:  Chirp";

	//==========================================================================
	// DATA LOGGING
	//==========================================================================

	// DataLogger log(WriterType::Buffered, false);
	// log.set_header(
	// 	{
	// 		"Test Time [s]",
	// 		"Motor Position Command [deg]",
	// 		"Motor Position [deg]",
	// 		"Motor Velocity [deg/s]",
	// 		"Motor Torque Command [Nm]",
	// 		"Motor Torque Sense [Nm]"
	// 	}
	// );

	//==========================================================================
	// CONTROL LOOP
	//==========================================================================

	// enable Q8-USB
	q8.enable();

	// create timer
	Timer timer(hertz(1000), Timer::Busy);

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
		if (!test_running) {
			if (xbox.is_button_pressed(XboxController::Left))
				poppy.set_motor_position(-1000);
			else if (xbox.is_button_pressed(XboxController::Right))
				poppy.set_motor_position(1000);
			else if (xbox.is_button_pressed(XboxController::Up))
				poppy.set_motor_position(0);
			else if (xbox.is_button_pressed(XboxController::RB))
				poppy.set_motor_torque(poppy.motor_stall_torque);
			else if (xbox.is_button_pressed(XboxController::LB))
				poppy.set_motor_torque(-poppy.motor_stall_torque);
			else {
				double torque = poppy.motor_stall_torque * (xbox.get_axis(XboxController::RT) - xbox.get_axis(XboxController::LT));
				poppy.set_motor_torque(torque);
			}

			// zero sensors
			if (xbox.is_button_pressed(XboxController::LS) && !poppy_zeroed) {
				poppy.zero();
				LOG(Info) << "Poppy Zeroed";
			}

			// start testing
			if (xbox.is_button_pressed(XboxController::Start)) {
				t = Time::Zero;
				LOG(Info) << "Bandwidth Analysis Started";
				//log.clear_data();
				test_running = true;

			}
		}

		//======================================================================
		// BANDWIDTH ANALYSIS
		//======================================================================

		if (test_running) {

			if (xbox.is_button_pressed(XboxController::Left))
				f = &sqrwave;
			else if (xbox.is_button_pressed(XboxController::Right))
				f = &sinwave;
			else if (xbox.is_button_pressed(XboxController::Up))
				f = &chirp;

			double pos_com = f->evaluate(t);

			ms_traj.write_data({ chirp(t), sinwave(t), sqrwave(t), pos_com, poppy.get_motor_position() });
			poppy.set_motor_position(pos_com);

			// log.buffer(
			// 	{
			// 		t.as_seconds(),
			// 		pos_com,
			// 		poppy.get_motor_position(),
			// 		poppy.get_motor_velocity(),
			// 		poppy.get_motor_torque_command(),
			// 		poppy.get_motor_torque_sense()
			// 	}
			// );

			t += timer.get_period();

			// test over, save data
			if (t > dur) {

				std::stringstream ss;
				ss << "bandwidth_";
				if (f == &sqrwave)
					ss << "sqr_";
				else if (f == &sinwave)
					ss << "sin_";
				else if (f == &chirp)
					ss << "chp_";

				ss << "a" << std::fixed << std::setprecision(0) << amplitude << "_";
				ss << "f" << std::fixed << std::setprecision(0) << freq.as_hertz() << "_";
				ss << "d" << std::fixed << std::setprecision(0) << dur.as_seconds();

				//log.save_data(ss.str(),"logs");
				test_running = false;
				t = Time::Zero;
			}

			if (xbox.is_button_pressed(XboxController::Back)) {
				LOG(Info) << "Bandwidth Analysis Canceled";
				//log.clear_data();
				test_running = false;

			}

		}

		//======================================================================
		// MELSCOPE
		//======================================================================

		// broadcast poppy data
		poppy.broadcast();

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
	//log.wait_for_save();

	return 0;
}


