// Evan Pezent
// Project cm - Force Analysis
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
	Options options("cm.exe");
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
	qoptions.set_encoder_filter(0, QuanserOptions::EncoderFilter::Filtered);
	qoptions.set_update_rate(QuanserOptions::UpdateRate::Normal);
	// qoptions.set_decimation(1);
	qoptions.set_special_option("decimation=1");
	Q8Usb q8(qoptions);

	// open both Q8-USBs
	if (!q8.open()) {
		LOG(Fatal) << "Failed to open Q8-USBs";
		return 1;
	}

	// make cm
	CM cm("cm", q8.DO[0], q8.DI[0], q8.AO[0], q8.AI[0], q8.AI[1], q8.encoder[0], CM::Config());
    cm.enableBroadcast(true);
    cm.enableControlValueFilter(false);
    auto q = cm.getQuery();

	//==========================================================================
	// TUNING VARIABLES
	//==========================================================================

	bool testing = false;

	double kp = q.positionKp * 10e6;
	double kd = q.positionKd * 10e6;

    print(kp);
    print(kd);

	Clock clock;
	Time click_time = milliseconds(200);

	MelShare ms_pd("pid_gains");

	Waveform sinwave(Waveform::Sin, seconds(1));
	double position = 0.0;

	//==========================================================================
	// CONTROL LOOP
	//==========================================================================

	// enable Q8-USB
	q8.enable();

	// create timer
	Timer timer(hertz(1000), Timer::Hybrid);
	Time t;

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
			LOG(Info) << "CM Enabled";
		}

		// disable if X pressed
		if (xbox.is_button_pressed(XboxController::X) && cm.is_enabled()) {
			cm.disable();
			LOG(Info) << "CM Disabled";
		}

		// stop if B pressed
		if (xbox.is_button_pressed(XboxController::B)) {
			STOP = true;
		}

		// manual control
		if (!testing) {
			// set loose and tight positions
			if (xbox.is_button_pressed(XboxController::Up) && clock.get_elapsed_time() > click_time) {
				kp += 10;
                cm.setPositionGains(kp / 10e6, 0, kd / 10e6);
				clock.restart();
			}
			else if (xbox.is_button_pressed(XboxController::Down) && clock.get_elapsed_time() > click_time) {
				kp -= 10;
                cm.setPositionGains(kp / 10e6, 0, kd / 10e6);
				clock.restart();
			}
			else if (xbox.is_button_pressed(XboxController::Left) && clock.get_elapsed_time() > click_time) {
				kd -= 1;
                cm.setPositionGains(kp / 10e6, 0, kd / 10e6);
				clock.restart();
			}
			else if (xbox.is_button_pressed(XboxController::Right) && clock.get_elapsed_time() > click_time) {
				kd += 1;
                cm.setPositionGains(kp / 10e6, 0, kd / 10e6);
				clock.restart();
			}
			else if (xbox.is_button_pressed(XboxController::LS)) {
				cm.zeroPosition();
				LOG(Info) << "CM Zeroed";
			}
			else if (xbox.is_button_pressed(XboxController::Start)) {
					LOG(Info) << "Entered Testing Mode";
					testing = true;
                    cm.zeroPosition();
                    cm.setControlMode(CM::ControlMode::Position);

			}
			else {
				double torque = (xbox.get_axis(XboxController::RT) - xbox.get_axis(XboxController::LT));
				cm.setControlValue(torque);
			}
		}
		else {
			position = 0.5 * sinwave.evaluate(t) + 0.5;
			t += timer.get_period();
			cm.setControlValue(position);
			if (xbox.is_button_pressed(XboxController::Back)) {
				LOG(Info) << "Entered Tuning Mode";
				testing = false;
                cm.setControlMode(CM::ControlMode::Torque);
			}
		}

        cm.update(timer.get_elapsed_time());

		//======================================================================
		// MELSCOPE
		//======================================================================

		ms_pd.write_data({ kp, kd, position});

		//======================================================================
		// DAQ OUTPUT
		//======================================================================

		// sync DAQ outputs with real world
		q8.update_output();

		// wait contro/ loop timer
		timer.wait();
	}

	cm.disable();

	// disable and close Q8-USB
	q8.disable();
	q8.close();

	return 0;
}


