/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file heater.h
 *
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Jake Dahl <dahl.jakejacob@gmail.com>
 */

#pragma once

#include <px4_workqueue.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_config.h>
#include <px4_getopt.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>

#include <mathlib/mathlib.h>

#define CONTROLLER_PERIOD_DEFAULT 100000

/**
 * @brief IMU Heater Controller driver used to maintain consistent
 *        temparature at the IMU.
 */
extern "C" __EXPORT int heater_main(int argc, char *argv[]);


class Heater : public ModuleBase<Heater>, public ModuleParams
{
public:
	Heater();

	virtual ~Heater();

	/**
	 * @see ModuleBase::custom_command().
	 * @brief main Main entry point to the module that should be
	 *        called directly from the module's main method.
	 * @param argc The input argument count.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage().
	 * @brief Prints the module usage to the nuttshell console.
	 * @param reason The requested reason for printing to console.
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @see ModuleBase::task_spawn().
	 * @brief Initializes the class in the same context as the work queue
	 *        and starts the background listener.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief Sets and/or reports the heater controller time period value in microseconds.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	int controller_period(char *argv[]);

	/**
	 * @brief Reports the average heater on duty cycle as a percent.
	 * @return Returns the average heater on cycle duty cycle as a percent.
	 */
	float duty_cycle();

	/**
	 * @brief Sets and/or reports the heater controller feed fordward value.
	 * @param argv Pointer to the input argument array.
	 * @return Returns the heater feed forward value iff successful, 0.0f otherwise.
	 */
	float feed_forward(char *argv[]);

	/**
	 * @brief Sets and/or reports the heater controller integrator gain value.
	 * @param argv Pointer to the input argument array.
	 * @return Returns the heater integrator gain value iff successful, 0.0f otherwise.
	 */
	float integrator(char *argv[]);

	/**
	 * @brief Sets and/or reports the heater controller proportional gain value.
	 * @param argv Pointer to the input argument array.
	 * @return Returns the heater proportional gain value iff successful, 0.0f otherwise.
	 */
	float proportional(char *argv[]);

	/**
	 * @brief Reports the heater target sensor.
	 * @return Returns the id of the target sensor
	 */
	uint32_t sensor_id();

	/**
	 * @brief Initiates the heater driver work queue, starts a new background task,
	 *        and fails if it is already running.
	 * @return Returns 1 iff start was successful.
	 */
	int start();

	/**
	 * @brief Reports curent status and diagnostic information about the heater driver.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	int print_status();

	/**
	 * @brief Reports the current heater temperature.
	 * @return Returns the current heater temperature value iff successful, -1.0f otherwise.
	 */
	float sensor_temperature();

	/**
	 * @brief Sets and/or reports the heater target temperature.
	 * @param argv Pointer to the input argument array.
	 * @return Returns the heater target temperature value iff successful, -1.0f otherwise.
	 */
	float temperature_setpoint(char *argv[]);

protected:

	/**
	 * @brief Called once to initialize uORB topics.
	 */
	void initialize_topics();

	/**
	 * @see ModuleBase::initialize_trampoline().
	 * @brief Trampoline initialization.
	 * @param argv Pointer to the task startup arguments.
	 */
	static void initialize_trampoline(void *argv);

private:

	/**
	 * @brief Checks for new commands and processes them.
	 */
	void process_commands();

	/**
	 * @brief Trampoline for the work queue.
	 * @param argv Pointer to the task startup arguments.
	 */
	static void cycle_trampoline(void *argv);

	/**
	 * @brief Calculates the heater element on/off time, carries out
	 *        closed loop feedback and feedforward temperature control,
	 *        and schedules the next cycle.
	 */
	void cycle();

	/**
	 * @brief Updates the uORB topics for local subscribers.
	 * @param meta The uORB metadata to copy.
	 * @param handle The uORB handle to obtain data from.
	 * @param buffer The data buffer to copy data into.
	 * @return Returns true iff update was successful.
	 */
	int orb_update(const struct orb_metadata *meta, int handle, void *buffer);

	/**
	 * @brief Updates and checks for updated uORB parameters.
	 * @param force Boolean to determine if an update check should be forced.
	 */
	void update_params(const bool force = false);

	/** Work queue struct for the RTOS scheduler. */
	static struct work_s _work;

	int _controller_period_usec = CONTROLLER_PERIOD_DEFAULT;

	float _duty_cycle = 0.0f;

	bool _heater_on = false;

	float _integrator_value = 0.0f;

	int _params_sub = 0;

	float _proportional_value = 0.0f;

	struct sensor_accel_s _sensor_accel = {};

	int _sensor_accel_sub = -1;

	float _sensor_temperature = 0.0f;

	/** @note Declare local parameters using defined parameters. */
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SENS_IMU_TEMP_FF>)  _p_feed_forward_value,
		(ParamFloat<px4::params::SENS_IMU_TEMP_I>)  _p_integrator_gain,
		(ParamFloat<px4::params::SENS_IMU_TEMP_P>)  _p_proportional_gain,
		(ParamInt<px4::params::SENS_TEMP_ID>) _p_sensor_id,
		(ParamFloat<px4::params::SENS_IMU_TEMP>) _p_temperature_setpoint
	)
};
